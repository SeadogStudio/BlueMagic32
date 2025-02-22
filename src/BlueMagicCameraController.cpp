#include "BlueMagicCameraController.h"

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

BlueMagicCameraController::BlueMagicCameraController(BLERemoteCharacteristic* outgoingCameraControl)
    : _cameraControl(outgoingCameraControl) {}

BlueMagicCameraController::~BlueMagicCameraController() {}

bool BlueMagicCameraController::changed() {
    return _state->changed();
}

bool BlueMagicCameraController::settingsChanged() {
    return _state->settingsChanged();
}

uint8_t* BlueMagicCameraController::settingsData() {
    return _state->settingsData();
}

bool BlueMagicCameraController::timecodeChanged() {
    return _state->timecodeChanged();
}

uint8_t* BlueMagicCameraController::timecodeData() {
    return _state->timecodeData();
}

bool BlueMagicCameraController::statusChanged() {
    return _state->statusChanged();
}

uint8_t* BlueMagicCameraController::statusData() {
    return _state->statusData();
}

uint32_t BlueMagicCameraController::mapFloat(float value) {
    return static_cast<uint32_t>(mapf(value, 0, 1.0, 0, 2047.0));
}

void BlueMagicCameraController::setCamera(uint8_t index) {
    _cameraIndex = index;
}

bool BlueMagicCameraController::custom(uint8_t* data, size_t len) {
    if (!_cameraControl) return false;
    return _cameraControl->writeValue(data, len, true);
}

uint8_t BlueMagicCameraController::getCameraStatus() {
    return _state->getCameraStatus();
}

int8_t BlueMagicCameraController::getTransportMode() {
    return _state->getTransportMode();
}

bool BlueMagicCameraController::record(bool record) {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 5, 0, 0, 10, 1, 1, 0, 0, 0, 0, 0 };
    data[8] = record ? 2 : 0;
    return _cameraControl->writeValue(data, 12, true);
}

bool BlueMagicCameraController::toggleRecording() {
    return record(!isRecording());
}

bool BlueMagicCameraController::isRecording() {
    return getTransportMode() == 2;
}

bool BlueMagicCameraController::play(bool play) {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 5, 0, 0, 10, 1, 1, 0, 0, 0, 0, 0 };
    data[8] = play ? 1 : 0;
    return _cameraControl->writeValue(data, 12, true);
}

bool BlueMagicCameraController::isPlaying() {
    return getTransportMode() == 1;
}

bool BlueMagicCameraController::preview(bool preview) {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 5, 0, 0, 10, 1, 1, 0, 0, 0, 0, 0 };
    data[8] = preview ? 0 : 3; // 3 means stop preview.
    return _cameraControl->writeValue(data, 12, true);
}

bool BlueMagicCameraController::isPreviewing() {
    return getTransportMode() == 0;
}

bool BlueMagicCameraController::ois(bool enabled) {
    if (!_cameraControl) return false;
    uint8_t data[10] = { 255, 6, 0, 0, 0, 6, 0, 0, 0 };
    data[8] = enabled ? 1 : 0;
    return _cameraControl->writeValue(data, 10, true);
}

bool BlueMagicCameraController::getOis() {
    // Implement logic to get OIS status from camera state
    return true; // Placeholder
}

bool BlueMagicCameraController::codec(CODEC_TYPE c, CODEC_QUALITY q) {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 5, 0, 0, 10, 0, 1, 0, 0, 0, 0, 0 };
    data[8] = static_cast<uint8_t>(c);
    data[9] = static_cast<uint8_t>(q);
    return _cameraControl->writeValue(data, 12, true);
}

int8_t BlueMagicCameraController::getCodecType() {
    return _state->getCodec();
}

int8_t BlueMagicCameraController::getCodecQuality() {
    return _state->getQuality();
}

bool BlueMagicCameraController::focus(float focus) {
    if (!_cameraControl || focus < 0 || focus > 1) return false;
    uint16_t val = mapFloat(focus);
    uint8_t xlow = val & 0xff;
    uint8_t xhigh = (val >> 8);
    uint8_t data[10] = { 255, 6, 0, 0, 0, 0, 128, 0, xlow, xhigh };
    return _cameraControl->writeValue(data, 10, true);
}

bool BlueMagicCameraController::instantAutoFocus() {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 4, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 };
    return _cameraControl->writeValue(data, 12, true);
}

float BlueMagicCameraController::getFocus() {
    return _state->getFocus();
}

bool BlueMagicCameraController::zoom(float zoom) {
    if (!_cameraControl || zoom < 0 || zoom > 1) return false;
    uint16_t val = mapFloat(zoom);
    uint8_t xlow = val & 0xff;
    uint8_t xhigh = (val >> 8);
    uint8_t data[10] = { 255, 6, 0, 0, 0, 8, 128, 0, xlow, xhigh };
    return _cameraControl->writeValue(data, 10, true);
}

float BlueMagicCameraController::getZoom() {
    return _state->getZoom();
}

bool BlueMagicCameraController::aperture(float value) {
    if (!_cameraControl || value < 0 || value > 1) return false;
    uint16_t val = mapFloat(value);
    uint8_t xlow = val & 0xff;
    uint8_t xhigh = (val >> 8);
    uint8_t data[10] = { 255, 6, 0, 0, 0, 3, 128, 0, xlow, xhigh };
    return _cameraControl->writeValue(data, 10, true);
}

bool BlueMagicCameraController::autoAperture() {
    if (!_cameraControl) return false;
    uint8_t data[12] = { 255, 4, 0, 0, 0, 5, 1, 0, 0, 0, 0, 0 };
    return _cameraControl->writeValue(data, 12, true);
}

float BlueMagicCameraController::getAperture() {
    return _state->getAperture();
}

bool BlueMagicCameraController::iso(int32_t iso) {
    if (!_cameraControl || iso < 100 || iso > 25600) return false; // Example ISO range
    uint8_t xlow = iso & 0xff;
    uint8_t xhigh = (iso >> 8);
    uint8_t data[12] = { 255, 8, 0, 0, 1, 14, 3, 0, xlow, xhigh, 0,
