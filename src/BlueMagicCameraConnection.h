// BlueMagicCameraConnection.cpp

#include "BlueMagicCameraConnection.h"

BlueMagicCameraConnection::BlueMagicCameraConnection() : connected(false), pClient(nullptr), pCharacteristic(nullptr), pBLEScan(nullptr), advertisedDeviceCallbacks(nullptr) {}

void BlueMagicCameraConnection::begin(const char* deviceName, void (*cameraConnectedCallback)(), void (*cameraDisconnectedCallback)(), void (*shutterPressedCallback)()) {
  targetDeviceName = deviceName;
  this->cameraConnectedCallback = cameraConnectedCallback;
  this->cameraDisconnectedCallback = cameraDisconnectedCallback;
  this->shutterPressedCallback = shutterPressedCallback;

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  advertisedDeviceCallbacks = new MyAdvertisedDeviceCallbacks(this); // Pass 'this'
  pBLEScan->setAdvertisedDeviceCallbacks(advertisedDeviceCallbacks);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, nullptr, false); // Scan forever
}

void BlueMagicCameraConnection::startScan() {
  if (pBLEScan) {
    pBLEScan->start(0, nullptr, false); // Scan forever
  }
}

void BlueMagicCameraConnection::stopScan() {
  if (pBLEScan) {
    pBLEScan->stop();
  }
}

void BlueMagicCameraConnection::connectToServer(BLEAdvertisedDevice* advertisedDevice) {
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(&clientCallbacks);

  if (pClient->connect(advertisedDevice)) {
    Serial.println("Connected to server");

    BLERemoteService* pRemoteService = pClient->getService(BLEUUID("YOUR_SERVICE_UUID")); // Replace with your service UUID

    if (pRemoteService) {
      pCharacteristic = pRemoteService->getCharacteristic(BLEUUID("YOUR_CHARACTERISTIC_UUID")); // Replace with your characteristic UUID

      if (pCharacteristic) {
        if (pCharacteristic->canWrite()) {
          connected = true;
          if (cameraConnectedCallback) {
            cameraConnectedCallback();
          }
        } else {
          Serial.println("Characteristic cannot write");
          pClient->disconnect();
        }
      } else {
        Serial.println("Failed to find characteristic");
        pClient->disconnect();
      }
    } else {
      Serial.println("Failed to find service");
      pClient->disconnect();
    }
  } else {
    Serial.println("Failed to connect to server");
  }
}

void BlueMagicCameraConnection::disconnectFromServer() {
  if (pClient) {
    pClient->disconnect();
  }
}

void BlueMagicCameraConnection::sendShutterPress() {
  if (pCharacteristic) {
    pCharacteristic->writeValue("1"); // Or any value that triggers the shutter
  }
}

bool BlueMagicCameraConnection::isConnected() {
  return connected;
}

// MyAdvertisedDeviceCallbacks.h

#ifndef MyAdvertisedDeviceCallbacks_h
#define MyAdvertisedDeviceCallbacks_h

#include "BLEAdvertisedDeviceCallbacks.h"
#include "BlueMagicCameraConnection.h"

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
  MyAdvertisedDeviceCallbacks(BlueMagicCameraConnection* cameraConnection);
  void onResult(BLEAdvertisedDevice advertisedDevice) override;

private:
  BlueMagicCameraConnection* cameraConnection;
};

#endif

// MyAdvertisedDeviceCallbacks.cpp

#include "MyAdvertisedDeviceCallbacks.h"

MyAdvertisedDeviceCallbacks::MyAdvertisedDeviceCallbacks(BlueMagicCameraConnection* cameraConnection) : cameraConnection(cameraConnection) {}

void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
  if (advertisedDevice.getName().find(cameraConnection->targetDeviceName) != std::string::npos) {
    cameraConnection->stopScan();
    cameraConnection->connectToServer(&advertisedDevice);
  }
}

// BlueMagic32.cpp

#include "BlueMagic32.h"

BlueMagic32::BlueMagic32() : cameraConnected(false) {}

void BlueMagic32::begin(const char* deviceName) {
  cameraConnection.begin(deviceName, [this]() {
    cameraConnected = true;
    if (cameraConnectedCallback) {
      cameraConnectedCallback();
    }

  }, [this]() {
    cameraConnected = false;
    if (cameraDisconnectedCallback) {
      cameraDisconnectedCallback();
    }

  }, [this]() {
    if (shutterPressedCallback) {
      shutterPressedCallback();
    }
  });
}

void BlueMagic32::startCameraConnection(void (*cameraConnectedCallback)(), void (*cameraDisconnectedCallback)(), void (*shutterPressedCallback)()) {
  this->cameraConnectedCallback = cameraConnectedCallback;
  this->cameraDisconnectedCallback = cameraDisconnectedCallback;
  this->shutterPressedCallback = shutterPressedCallback;
  cameraConnection.startScan();
}

void BlueMagic32::stopCameraConnection() {
  cameraConnection.stopScan();
  cameraConnection.disconnectFromServer();
}

void BlueMagic32::sendShutterPress() {
  cameraConnection.sendShutterPress();
}

bool BlueMagic32::isCameraConnected() {
  return cameraConnected;
}

// Basic_Remote_Trigger.ino example
#include "BlueMagic32.h"

BlueMagic32 remote;

void setup() {
  Serial.begin(115200);
  remote.begin("YourTargetDeviceName"); // Replace with your camera's name
  remote.startCameraConnection(cameraConnected, cameraDisconnected, shutterPressed);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      remote.sendShutterPress();
    }
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
