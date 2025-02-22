// BlueMagicCameraConnection.h

#ifndef BlueMagicCameraConnection_h
#define BlueMagicCameraConnection_h

#include "Arduino.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "MyAdvertisedDeviceCallbacks.h"
#include "BLEClient.h"
#include "BLECharacteristic.h"
#include "BLEDescriptor.h"

class BlueMagicCameraConnection {
public:
  BlueMagicCameraConnection();
  void begin(const char* deviceName, void (*cameraConnectedCallback)(), void (*cameraDisconnectedCallback)(), void (*shutterPressedCallback)());
  void startScan();
  void stopScan();
  void connectToServer(BLEAdvertisedDevice* advertisedDevice);
  void disconnectFromServer();
  void sendShutterPress();
  bool isConnected();

private:
  class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pClient) override;
    void onDisconnect(BLEClient* pClient) override;
  };

  class MyCharacteristicCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override;
  };

  MyAdvertisedDeviceCallbacks* advertisedDeviceCallbacks;
  MyClientCallback clientCallbacks;
  MyCharacteristicCallback characteristicCallbacks;

  BLEScan* pBLEScan;
  BLEClient* pClient;
  BLECharacteristic* pCharacteristic;

  String targetDeviceName;
  bool connected;

  void (*cameraConnectedCallback)();
  void (*cameraDisconnectedCallback)();
  void (*shutterPressedCallback)();
};

#endif
