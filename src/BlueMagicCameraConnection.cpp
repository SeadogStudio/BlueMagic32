#include "BlueMagicCameraConnection.h"

BLERemoteCharacteristic *BlueMagicCameraConnection::_cameraStatus;
BLERemoteCharacteristic *BlueMagicCameraConnection::_deviceName;
BLERemoteCharacteristic *BlueMagicCameraConnection::_timecode;
BLERemoteCharacteristic *BlueMagicCameraConnection::_outgoingCameraControl;
BLERemoteCharacteristic *BlueMagicCameraConnection::_incomingCameraControl;

// ... (UUID definitions remain the same) ...

// ... (Notification callbacks remain the same) ...

// ... (MyAdvertisedDeviceCallbacks and MySecurity remain the same) ...

BlueMagicCameraConnection::BlueMagicCameraConnection() {}

BlueMagicCameraConnection::~BlueMagicCameraConnection() {
    if (_client && _client->isConnected()) {
        _client->disconnect();
    }
    if (_cameraControl) {
        delete _cameraControl;
    }
    if (_device) {
        _device.deinit(true);
    }
    if (PREF_INCLUDED && _pref) {
        delete _pref;
    }
}

void BlueMagicCameraConnection::begin(String name) {
    if (_init) return;

    _name = name;
    setState(CAMERA_DISCONNECTED);

    if (!PREF_INCLUDED) {
        _pref = new Preferences();
    }

    _pref->begin(_name.c_str(), false);
    setAuthentication(_pref->getBool("authenticated", false));
    String addr = _pref->getString("cameraAddress", "");
    if (addr.length() > 0) {
        setCameraAddress(BLEAddress(addr.c_str()));
    }
    _pref->end();

    _device.init(_name.c_str());
    _device.setPower(ESP_PWR_LVL_P9);
    _device.setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    _device.setSecurityCallbacks(new MySecurity());

    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_IN);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    _init = true;
}

bool BlueMagicCameraConnection::scan(bool active, int duration) {
    if (getAuthentication() && getCameraAddress() != nullptr) {
        return false;
    }

    _bleScan = _device.getScan();
    _bleScan->clearResults();
    _bleScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    _bleScan->setActiveScan(active);
    return _bleScan->start(duration);
}

int BlueMagicCameraConnection::connected() {
    return _connected;
}

bool BlueMagicCameraConnection::available() {
    return connected() && (_cameraControl != nullptr);
}

bool BlueMagicCameraConnection::connectToServer(BLEAddress address) {
    _client = BLEDevice::createClient();
    if (!_client) {
        Serial.println("Error: Failed to create BLE client.");
        return false;
    }

    setState(CAMERA_CONNECTING);
    if (!_client->connect(address)) {
        Serial.println("Error: Failed to connect to server.");
        return false;
    }

    BLERemoteService* pRemoteService = _client->getService(BmdCameraService);
    if (!pRemoteService) {
        Serial.println("Error: Failed to find service UUID.");
        _client->disconnect();
        return false;
    }

    _deviceName = pRemoteService->getCharacteristic(DeviceName);
    if (_deviceName) {
        _deviceName->writeValue(_name.c_str(), _name.length());
    }

    _outgoingCameraControl = pRemoteService->getCharacteristic(OutgoingCameraControl);
    _incomingCameraControl = pRemoteService->getCharacteristic(IncomingCameraControl);
    _timecode = pRemoteService->getCharacteristic(Timecode);
    _cameraStatus = pRemoteService->getCharacteristic(CameraStatus);

    if (!_incomingCameraControl || !_timecode || !_cameraStatus) {
        Serial.println("Error: Failed to find required characteristics.");
        _client->disconnect();
        return false;
    }

    _incomingCameraControl->registerForNotify(controlNotify, false);
    _timecode->registerForNotify(timeCodeNotify);
    _cameraStatus->registerForNotify(cameraStatusNotify);

    setState(CAMERA_CONNECTED);
    setController();
    return true;
}

BlueMagicCameraController* BlueMagicCameraConnection::connect(uint8_t index) {
    if (_cameraControl) return _cameraControl;

    bool ok = false;
    if (getAuthentication() && getCameraAddress() != nullptr) {
        ok = connectToServer(*getCameraAddress());
    } else {
        if (scan(false, 5)) {
            BLEScanResults* scanResults = _bleScan->getResults();
            if (scanResults && scanResults->getCount() > 0) {
                BLEAddress address = scanResults->getDevice(scanResults->getCount() - 1).getAddress();
                ok = connectToServer(address);
                if (ok) {
                    setCameraAddress(address);
                }
            } else {
                Serial.println("No devices found during scan.");
            }
        } else {
            Serial.println("Scan failed.");
        }
    }

    if (ok) {
        setAuthentication(true);
        _pref->begin(_name.c_str(), false);
        _pref->putString("cameraAddress", getCameraAddress()->toString().c_str());
        _pref->putBool("authenticated", getAuthentication());
        _pref->end();
        return _cameraControl;
    }

    return nullptr;
}

void BlueMagicCameraConnection::disconnect() {
    if (_client && _client->isConnected()) {
        _client->disconnect();
    }
    if (_cameraControl) {
        delete _cameraControl;
        _cameraControl = nullptr;
    }
    setState(CAMERA_DISCONNECTED);
}

void BlueMagicCameraConnection::clearPairing() {
    disconnect();
    if (getCameraAddress() && getCameraAddress()->getNative()) {
        esp_ble_remove_bond_device(getCameraAddress()->getNative());
    }

    int dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t* dev_list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (dev_list) {
        esp_ble_get_bond_device_list(&dev_num, dev_list);
        for (int i = 0; i < dev_num; i++) {
            esp_ble_remove_bond_device(dev_list[i].bd_addr);
        }
        free(dev_list);
    }
    setAuthentication(false);
    _pref->begin(_name.c_str(), false);
    _pref->putString("cameraAddress", "");
    _pref->putBool("authenticated", getAuthentication());
    _pref->end();
}

void BlueMagicCameraConnection::setController() {
    _cameraControl = new BlueMagicCameraController(_outgoingCameraControl);
}

// ... (setState, setAuthentication, getAuthentication, setCameraAddress, getCameraAddress remain the same) ...
