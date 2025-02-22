// BlueMagic32.h

#ifndef BlueMagic32_h
#define BlueMagic32_h

#include "Arduino.h"
#include "BlueMagicCameraConnection.h"

class BlueMagic32 {
public:
  BlueMagic32();
  void begin(const char* deviceName);
  void startCameraConnection(void (*cameraConnectedCallback)(), void (*cameraDisconnectedCallback)(), void (*shutterPressedCallback)());
  void stopCameraConnection();
  void sendShutterPress();
  bool isCameraConnected();

private:
  BlueMagicCameraConnection cameraConnection;
  bool cameraConnected;
};

#endif
