#ifndef MYADVERTISEDEVICE_H
#define MYADVERTISEDEVICE_H

#include <BLEAdvertisedDevice.h>
// #include <BLEAdvertisedDeviceCallbacks.h>
#include "BlueMagicCameraConnection.h" // Needed because it takes a pointer to the BlueMagicCameraConnection class

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
    MyAdvertisedDeviceCallbacks(BlueMagicCameraConnection* connection);
    void onResult(BLEAdvertisedDevice advertisedDevice);

private:
    BlueMagicCameraConnection* _connection;

};

#endif // MYADVERTISEDEVICE_H
