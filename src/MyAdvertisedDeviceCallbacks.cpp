#include "MyAdvertisedDeviceCallbacks.h"
#include <Arduino.h>

MyAdvertisedDeviceCallbacks::MyAdvertisedDeviceCallbacks(BlueMagicCameraConnection* connection) : _connection(connection) {}

void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Advertised Device: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(BLEUUID(ServiceId))) {
        advertisedDevice.getScan()->stop();
        _connection->connectToServer(advertisedDevice.getAddress());
    }
}
