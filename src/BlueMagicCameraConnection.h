#ifndef BlueMagicCameraConnection_h
#define BlueMagicCameraConnection_h

#include "Arduino.h"
#include <BLEDevice.h>
#include <Preferences.h>
#include "BlueMagicState.h"
#include "BlueMagicCameraController.h"
#include "MyAdvertisedDeviceCallbacks.h"
#include "MySecurity.h"

enum CONNECTION_STATE {
    CAMERA_CONNECTED = 1,
    CAMERA_DISCONNECTED = 2,
    CAMERA_CONNECTING = 3
};

class BlueMagicCameraConnection {
public:
    BlueMagicCameraConnection();
    ~BlueMagicCameraConnection();

    void begin(const String& name);
    void begin(const String& name, Preferences& pref);

    bool scan(bool active, int duration);

    BlueMagicCameraController* connect();
    BlueMagicCameraController* connect(uint8_t index);

    void disconnect();
    void clearPairing();

    bool available() const;
    int connected() const;
    bool getAuthentication() const;
    BLEAddress getCameraAddress() const;

private:
    String _name;
    Preferences* _pref;
    bool _init = false;
    bool _authenticated;
    int _connected;

    BLEDevice _device;
    BLEClient* _client;
    BLEAddress _cameraAddress;

    BLERemoteCharacteristic* _outgoingCameraControl;
    BLERemoteCharacteristic* _incomingCameraControl;
    BLERemoteCharacteristic* _timecode;
    BLERemoteCharacteristic* _cameraStatus;
    BLERemoteCharacteristic* _deviceName;

    BLEScan* _bleScan;
    BlueMagicCameraController* _cameraControl;

    bool connectToServer(const BLEAddress& address);
    void setController();
    void setState(CONNECTION_STATE state);
    void setAuthentication(bool authenticated);
};

#endif // BlueMagicCameraConnection_h
