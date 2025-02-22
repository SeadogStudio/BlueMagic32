#include "BlueMagicCameraConnection.h"
#include "MyAdvertisedDeviceCallbacks.h"
#include "MySecurity.h"

static BLEUUID ServiceId("00001800-0000-1000-8000-00805f9b34fb");
static BLEUUID BmdCameraService("291D567A-6D75-11E6-8B77-86F30CA893D3");

static void cameraStatusNotify(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    BlueMagicState *blu = BlueMagicState::getInstance();
    blu->statusNotify(true, pData);
    blu->setCameraStatus(pData[0]);
}

static void timeCodeNotify(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    BlueMagicState *blu = BlueMagicState::getInstance();
    blu->timecodeNotify(true, pData);
    uint8_t H, M, S, f;
    H = (pData[11] / 16 * 10) + (pData[11] % 16);
    M = (pData[10] / 16 * 10) + (pData[10] % 16);
    S = (pData[9] / 16 * 10) + (pData[9] % 16);
    f = (pData[8] / 16 * 10) + (pData[8] % 16);
    blu->setTimecode(H, M, S, f);
}

static void controlNotify(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    BlueMagicState *blu = BlueMagicState::getInstance();
    blu->settingsNotify(true, pData);
    bool changed = false;
    // ... (rest of the controlNotify function remains the same)
    blu->setChanged(changed);
}

BlueMagicCameraConnection::BlueMagicCameraConnection() : _pref(new Preferences()), _cameraAddress(BLEAddress::nullAddress) {}

BlueMagicCameraConnection::~BlueMagicCameraConnection() {
    delete _pref;
    delete _client;
    delete _cameraControl;
    _device.deinit(true);
}

void BlueMagicCameraConnection::begin(const String& name) {
    if (_init) {
        return;
    }

    _name = name;
    setState(CAMERA_DISCONNECTED);

    _pref->begin(_name.c_str(), false);

    setAuthentication(_pref->getBool("authenticated", false));
    String addr = _pref->getString("cameraAddress", "");
    if (addr.length() > 0) {
        _cameraAddress = BLEAddress(addr.c_str());
    }

    _pref->end();

    _device.init(_name.c_str());
    _device.setPower(ESP_PWR_LVL_P9);
    _device.setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    _device.setSecurityCallbacks(new MySecurity());

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_IN);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    _init = true;
}

void BlueMagicCameraConnection::begin(const String& name, Preferences& pref) {
    _pref = &pref;
    begin(name);
}

bool BlueMagicCameraConnection::scan(bool active, int duration) {
    if (getAuthentication() && !_cameraAddress.equals(BLEAddress::nullAddress)) {
        return false;
    } else {
        _bleScan = _device.getScan();
        _bleScan->clearResults();
        _bleScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(this));
        _bleScan->setActiveScan(active);
        _bleScan->start(duration);
    }
    return true;
}

int BlueMagicCameraConnection::connected() const {
    return _connected;
}

bool BlueMagicCameraConnection::available() const {
    return connected() && (_cameraControl != nullptr);
}

bool BlueMagicCameraConnection::connectToServer(const BLEAddress& address) {
    _client = _device.createClient();
    setState(CAMERA_CONNECTING);
    _client->connect(address);

    BLERemoteService *pRemoteService = _client->getService(BmdCameraService);
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(BmdCameraService.toString().c_str());
        return false;
    }

    BLEUUID DeviceName("FFAC0C52-C9FB-41A0-B063-CC76282EB89C");
    _deviceName = pRemoteService->getCharacteristic(DeviceName);
    if (_deviceName != nullptr) {
        _deviceName->writeValue(_name.c_str(), _name.length());
    }

    BLEUUID OutgoingCameraControl("5DD3465F-1AEE-4299-8493-D2ECA2F8E1BB");
    _outgoingCameraControl = pRemoteService->getCharacteristic(OutgoingCameraControl);
    if (_outgoingCameraControl == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(OutgoingCameraControl.toString().c_str());
        return false;
    }

    BLEUUID IncomingCameraControl("B864E140-76A0-416A-BF30-5876504537D9");
    _incomingCameraControl = pRemoteService->getCharacteristic(IncomingCameraControl);
    if (_incomingCameraControl == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(IncomingCameraControl.toString().c_str());
        return false;
    }
    _incomingCameraControl->registerForNotify(controlNotify, false);

    BLEUUID Timecode("6D8F2110-86F1-41BF-9AFB-451D87E976C8");
    _timecode =
