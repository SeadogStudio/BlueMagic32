// BlueMagic32.h
#ifndef BLUE_MAGIC_32_H
#define BLUE_MAGIC_32_H

#include "Arduino.h"
#include "BLEDevice.h"
#include "BLEScan.h"
#include "BLEClient.h"
#include "BLEUtils.h"
#include "BLEAdvertisedDevice.h"
#include "BLECharacteristic.h"

class BlueMagic32 {
public:
    BlueMagic32(const char* targetDeviceName, const char* serviceUUID, const char* characteristicUUID);
    ~BlueMagic32();

    bool begin();
    bool connectToCamera();
    void disconnectCamera();
    bool sendCommand(const char* command);
    bool isCameraConnected();

    void setCallbacks(void (*connectedCallback)(), void (*disconnectedCallback)(), void (*commandSentCallback)(bool success));

private:
    class ClientCallback : public BLEClientCallbacks {
    public:
        ClientCallback(BlueMagic32* parent);
        void onConnect(BLEClient* pClient) override;
        void onDisconnect(BLEClient* pClient) override;
    private:
        BlueMagic32* parent_;
    };

    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    public:
        AdvertisedDeviceCallbacks(BlueMagic32* parent);
        void onResult(BLEAdvertisedDevice advertisedDevice) override;
    private:
        BlueMagic32* parent_;
    };

    ClientCallback* clientCallbacks_;
    AdvertisedDeviceCallbacks* advertisedDeviceCallbacks_;

    BLEScan* bleScan_;
    BLEClient* bleClient_;
    BLECharacteristic* bleCharacteristic_;

    String targetDeviceName_;
    BLEUUID serviceUUID_;
    BLEUUID characteristicUUID_;
    bool isConnected_;

    void (*connectedCallback_)();
    void (*disconnectedCallback_)();
    void (*commandSentCallback_)(bool success);
};

#endif // BLUE_MAGIC_32_H

// BlueMagic32.cpp
#include "BlueMagic32.h"

BlueMagic32::BlueMagic32(const char* targetDeviceName, const char* serviceUUID, const char* characteristicUUID)
    : targetDeviceName_(targetDeviceName), serviceUUID_(serviceUUID), characteristicUUID_(characteristicUUID), isConnected_(false),
      clientCallbacks_(nullptr), advertisedDeviceCallbacks_(nullptr), bleScan_(nullptr), bleClient_(nullptr), bleCharacteristic_(nullptr),
      connectedCallback_(nullptr), disconnectedCallback_(nullptr), commandSentCallback_(nullptr) {}

BlueMagic32::~BlueMagic32() {
    disconnectCamera();
    delete clientCallbacks_;
    delete advertisedDeviceCallbacks_;
    if (bleScan_) delete bleScan_;
    BLEDevice::deinit(false);
}

bool BlueMagic32::begin() {
    BLEDevice::init("");
    bleScan_ = BLEDevice::getScan();
    advertisedDeviceCallbacks_ = new AdvertisedDeviceCallbacks(this);
    bleScan_->setAdvertisedDeviceCallbacks(advertisedDeviceCallbacks_);
    bleScan_->setActiveScan(true);
    return true;
}

bool BlueMagic32::connectToCamera() {
    bleScan_->start(0, nullptr, false);
    return true;
}

void BlueMagic32::disconnectCamera() {
    if (bleClient_ && bleClient_->isConnected()) {
        bleClient_->disconnect();
    }
    isConnected_ = false;
    bleScan_->stop();
}

bool BlueMagic32::sendCommand(const char* command) {
    if (bleCharacteristic_ && bleCharacteristic_->canWrite()) {
        if (bleCharacteristic_->writeValue((uint8_t*)command, strlen(command), false)) { // write without response, for speed.
            if(commandSentCallback_) commandSentCallback_(true); // success callback
            return true;
        } else {
            if(commandSentCallback_) commandSentCallback_(false); // fail callback
            return false;
        }

    }
    if(commandSentCallback_) commandSentCallback_(false); // fail callback
    return false;
}

bool BlueMagic32::isCameraConnected() {
    return isConnected_;
}

void BlueMagic32::setCallbacks(void (*connectedCallback)(), void (*disconnectedCallback)(), void (*commandSentCallback)(bool success)) {
    connectedCallback_ = connectedCallback;
    disconnectedCallback_ = disconnectedCallback;
    commandSentCallback_ = commandSentCallback;
}

// ClientCallback implementation
BlueMagic32::ClientCallback::ClientCallback(BlueMagic32* parent) : parent_(parent) {}

void BlueMagic32::ClientCallback::onConnect(BLEClient* pClient) {
    parent_->isConnected_ = true;
    if (parent_->connectedCallback_) {
        parent_->connectedCallback_();
    }
}

void BlueMagic32::ClientCallback::onDisconnect(BLEClient* pClient) {
    parent_->isConnected_ = false;
    if (parent_->disconnectedCallback_) {
        parent_->disconnectedCallback_();
    }
}

// AdvertisedDeviceCallbacks implementation
BlueMagic32::AdvertisedDeviceCallbacks::AdvertisedDeviceCallbacks(BlueMagic32* parent) : parent_(parent) {}

void BlueMagic32::AdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName().find(parent_->targetDeviceName_) != std::string::npos) {
        parent_->bleScan_->stop();
        parent_->bleClient_ = BLEDevice::createClient();
        parent_->clientCallbacks_ = new BlueMagic32::ClientCallback(parent_);
        parent_->bleClient_->setClientCallbacks(parent_->clientCallbacks_);

        if (parent_->bleClient_->connect(&advertisedDevice)) {
            BLERemoteService* remoteService = parent_->bleClient_->getService(parent_->serviceUUID_);
            if (remoteService) {
                parent_->bleCharacteristic_ = remoteService->getCharacteristic(parent_->characteristicUUID_);
                if (!parent_->bleCharacteristic_ || !parent_->bleCharacteristic_->canWrite()) {
                    parent_->bleClient_->disconnect();
                    parent_->isConnected_ = false;
                }
            } else {
                parent_->bleClient_->disconnect();
                parent_->isConnected_ = false;
            }
        } else {
            parent_->isConnected_ = false;
        }
    }
}

// Example Basic_Remote_Trigger.ino
#include "BlueMagic32.h"

BlueMagic32 cameraControl("FFAC0C52-C9FB-41A0-B063-CC76282EB89C", "00001800-0000-1000-8000-00805f9b34fb", "291D567A-6D75-11E6-8B77-86F30CA893D3");

void setup() {
    Serial.begin(115200);
    cameraControl.setCallbacks(cameraConnected, cameraDisconnected, commandSent);
    cameraControl.begin();
    cameraControl.connectToCamera();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        cameraControl.sendCommand(command.c_str());
    }
    delay(10);
}

void cameraConnected() {
    Serial.println("Camera Connected");
}

void cameraDisconnected() {
    Serial.println("Camera Disconnected");
}

void commandSent(bool success) {
    if (success) {
        Serial.println("Command sent successfully");
    } else {
        Serial.println("Command failed to send");
    }
}
