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
    bool sendShutterPress();
    bool isCameraConnected();

    void setCallbacks(void (*connectedCallback)(), void (*disconnectedCallback)(), void (*shutterCallback)());

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
    void (*shutterCallback_)();
};

#endif // BLUE_MAGIC_32_H

// BlueMagic32.cpp
#include "BlueMagic32.h"

BlueMagic32::BlueMagic32(const char* targetDeviceName, const char* serviceUUID, const char* characteristicUUID)
    : targetDeviceName_(targetDeviceName), serviceUUID_(serviceUUID), characteristicUUID_(characteristicUUID), isConnected_(false),
      clientCallbacks_(nullptr), advertisedDeviceCallbacks_(nullptr), bleScan_(nullptr), bleClient_(nullptr), bleCharacteristic_(nullptr),
      connectedCallback_(nullptr), disconnectedCallback_(nullptr), shutterCallback_(nullptr) {}

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

bool BlueMagic32::sendShutterPress() {
    if (bleCharacteristic_ && bleCharacteristic_->canWrite()) {
        bleCharacteristic_->writeValue("1");
        return true;
    }
    return false;
}

bool BlueMagic32::isCameraConnected() {
    return isConnected_;
}

void BlueMagic32::setCallbacks(void (*connectedCallback)(), void (*disconnectedCallback)(), void (*shutterCallback)()) {
    connectedCallback_ = connectedCallback;
    disconnectedCallback_ = disconnectedCallback;
    shutterCallback_ = shutterCallback;
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

BlueMagic32 cameraControl("YourCameraName", "YourServiceUUID", "YourCharacteristicUUID");

void setup() {
    Serial.begin(115200);
    cameraControl.setCallbacks(cameraConnected, cameraDisconnected, shutterPressed);
    cameraControl.begin();
    cameraControl.connectToCamera();
}

void loop() {
    if (Serial.available() && Serial.read() == 's') {
        cameraControl.sendShutterPress();
    }
    delay(10);
}

void cameraConnected() {
    Serial.println("Camera Connected");
}

void cameraDisconnected() {
    Serial.println("Camera Disconnected");
}

void shutterPressed() {
    Serial.println("Shutter Pressed");
}
