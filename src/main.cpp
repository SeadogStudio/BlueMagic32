#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include <NimBLEUtils.h>
#include "PS2Controller.h"
#include "esp_ps2dev.h"
#include "Adafruit_TinyUSB.h"
#include "analog_reader.h"
#include "rumble.h"
#include "profiles/analog_axis_profile.h" // Include the default profile

#ifdef USE_GPIO_EXPANDER
#include "esp32_gpio_expander.h"
#endif

// Configuration Constants (Adjust these for your hardware setup)
#define PS2_DATA_PIN    13  // GPIO pin connected to PS2 Data
#define PS2_COMMAND_PIN 15  // GPIO pin connected to PS2 Command
#define PS2_CLOCK_PIN   14  // GPIO pin connected to PS2 Clock
#define PS2_ATTENTION_PIN 12  // GPIO pin connected to PS2 Attention

#define LED_PIN         2   // GPIO pin for status LED (optional)
#define MODE_BUTTON_PIN 0   // GPIO pin for mode button (optional, typically BOOT button)

// HID Report Descriptor (from profiles/analog_axis_profile.h)
static const uint8_t hid_report_descriptor[] = {
  TUD_HID_REPORT_DESC_GAMEPAD() // Use the pre-defined gamepad descriptor (important)
};

// Gamepad Report Structure (must match HID report descriptor)
typedef struct {
    uint8_t buttons[2];   // 16 buttons (2 bytes)
    uint8_t hat;          // Hat switch (D-pad)
    uint8_t x;            // X-axis (Left Analog Stick)
    uint8_t y;            // Y-axis (Left Analog Stick)
    uint8_t z;            // Z-axis (Right Analog Stick - Horizontal)
    uint8_t rz;           // Rz-axis (Right Analog Stick - Vertical)
    uint8_t rx;           // Rx-axis (Unused, or pressure-sensitive buttons)
    uint8_t ry;           // Ry-axis (Unused, or pressure-sensitive buttons)
} gamepad_report_t;


// Global Variables
static NimBLEHIDDevice* hid;
static NimBLEServer* pServer;
static PS2Controller ps2(PS2_DATA_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_CLOCK_PIN);  // PS2 controller interface.
static bool deviceConnected = false;
static gamepad_report_t report;

//Forward Declarations
void processPS2Data(const PS2Data &ps2data, gamepad_report_t &report);

// Analog Reader (for pressure-sensitive buttons, if used)
AnalogReader analogReader;

// Rumble handling
RumbleHandler rumbleHandler;

// Function to cycle through modes (Bluetooth, USB, etc.) -  Implementation depends on your desired behavior.
void gamepad_cycle_modes() {
    // Add your mode switching logic here.  This is highly dependent on your project's needs.
    // Examples:
    // - Toggle between Bluetooth and USB modes.
    // - Change HID profiles.
    // - Enter a configuration mode.
    // This might involve:
    //   - Disconnecting/reconnecting Bluetooth.
    //   - Re-initializing USB.
    //   - Changing global variables to affect behavior.

    // Example (simple toggle between two states - you'll need to adapt this):
    static bool mode = false;
    mode = !mode;
    Serial.print("Mode changed to: ");
    Serial.println(mode ? "Mode 1" : "Mode 0"); // Replace with descriptive names.
}

// BLE Server Callbacks
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
        digitalWrite(LED_PIN, HIGH);  // Turn on LED on connection (optional)
    };

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc, int reason) {
        deviceConnected = false;
        Serial.print("Client disconnected (reason = ");
        Serial.print(reason); // Added the reason code
        Serial.println(")");
        digitalWrite(LED_PIN, LOW);   // Turn off LED on disconnection (optional)
        pServer->startAdvertising(); // Start advertising again
    }
};

// HID Callbacks (for rumble)
class HIDCallbacks : public NimBLEHIDCallbacks {
    void onSetReport(uint8_t report_id, NimBLEHIDReportType type, uint8_t* buffer, size_t len) {
        if (type == OUTPUT_REPORT) {
            rumbleHandler.handleRumble(buffer, len, &ps2);
        }
    }
};


void processPS2Data(const PS2Data &ps2data, gamepad_report_t &report) {
    // Reset report data
    memset(&report, 0, sizeof(report));

    // --- Button Mapping (Digital Buttons) ---
    report.buttons[0] = 0;
    report.buttons[1] = 0;

    // PS2 Button -> HID Button Mapping
    if (!ps2data.button.triangle)  report.buttons[0] |= (1 << 0); // Button 1
    if (!ps2data.button.circle)    report.buttons[0] |= (1 << 1); // Button 2
    if (!ps2data.button.cross)     report.buttons[0] |= (1 << 2); // Button 3
    if (!ps2data.button.square)    report.buttons[0] |= (1 << 3); // Button 4
    if (!ps2data.button.l1)        report.buttons[0] |= (1 << 4); // Button 5
    if (!ps2data.button.r1)        report.buttons[0] |= (1 << 5); // Button 6
    if (!ps2data.button.l2)        report.buttons[0] |= (1 << 6); // Button 7
    if (!ps2data.button.r2)        report.buttons[0] |= (1 << 7); // Button 8
    if (!ps2data.button.select)    report.buttons[1] |= (1 << 0); // Button 9
    if (!ps2data.button.start)     report.buttons[1] |= (1 << 1); // Button 10
    if (!ps2data.button.l3)        report.buttons[1] |= (1 << 2); // Button 11
    if (!ps2data.button.r3)        report.buttons[1] |= (1 << 3); // Button 12


    // --- D-Pad (Hat Switch) Mapping ---
    if (!ps2data.button.up)        report.hat = 0; // Up
    else if (!ps2data.button.right) report.hat = 2; // Right
    else if (!ps2data.button.down)  report.hat = 4; // Down
    else if (!ps2data.button.left)  report.hat = 6; // Left
    else                           report.hat = 8; // Neutral (all released)

    // Handle diagonals
    if (!ps2data.button.up && !ps2data.button.right)     report.hat = 1; // Up-Right
    else if (!ps2data.button.right && !ps2data.button.down)  report.hat = 3; // Right-Down
    else if (!ps2data.button.down && !ps2data.button.left)   report.hat = 5; // Down-Left
    else if (!ps2data.button.left && !ps2data.button.up)      report.hat = 7; // Left-Up


    // --- Analog Stick Mapping ---
    // Normalize analog stick values to 0-255 range (center at 128)
    report.x  = ps2data.analog.left_x;   // Left Stick X
    report.y  = ps2data.analog.left_y;   // Left Stick Y
    report.z = ps2data.analog.right_x;  // Right Stick X
    report.rz = ps2data.analog.right_y;  // Right Stick Y


    // --- Pressure-Sensitive Buttons (Optional, Example with L1/R1) ---
    // This maps pressure values to unused axes (Rx/Ry). You'll need to adjust this
    // based on how you want to handle pressure data.
    report.rx = analogReader.getCalibratedAnalogValue(ps2data.pressure.l1);  // Example: L1 pressure
    report.ry = analogReader.getCalibratedAnalogValue(ps2data.pressure.r1);  // Example: R1 pressure
    // ... map other pressure-sensitive buttons similarly ...


    // --- Debug Output (Optional) ---
    // Serial.printf("LX: %3u LY: %3u RX: %3u RY: %3u\n", report.x, report.y, report.z, report.rz);
    // Serial.printf("Buttons: %02X %02X  Hat: %u\n", report.buttons[0], report.buttons[1], report.hat);
}


void setup() {
    Serial.begin(115200);
    Serial.println("Starting BlueMagic32");

    // Initialize LED pin (optional)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize mode button pin (optional)
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

     // Initialize PS2 controller
    if (!ps2.init()) {
        Serial.println("PS2 controller initialization failed!");
        while (1) {
            digitalWrite(LED_PIN, HIGH); // Indicate error with LED
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
            } // Halt
    }
    Serial.println("PS2 Controller Initialized");

     // Initialize analog reader
    analogReader.init();
    analogReader.calibrate(ps2); // Calibrate using values from the PS2 controller

    // Initialize Rumble handler
    rumbleHandler.init();

    // Initialize Bluetooth
    NimBLEDevice::init("PS2 Gamepad"); // Set Bluetooth device name
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    hid = new NimBLEHIDDevice(pServer);
    hid->setReportDescriptor((uint8_t*)hid_report_descriptor, sizeof(hid_report_descriptor));
    hid->setCallbacks(new HIDCallbacks()); // Set callbacks for rumble
    hid->startServices();


    NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);  // Set appearance to gamepad
    pAdvertising->addServiceUUID(hid->getHIDService()->getUUID());  // Advertise HID service
    pAdvertising->setScanResponse(false);
    pAdvertising->start();
    Serial.println("Bluetooth advertising started");
}

void loop() {
    // Check for mode button press
    if (digitalRead(MODE_BUTTON_PIN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(MODE_BUTTON_PIN) == LOW) {
            gamepad_cycle_modes();
            while (digitalRead(MODE_BUTTON_PIN) == LOW); // Wait for button release
        }
    }

    // Read PS2 controller data
    PS2Data ps2data = ps2.read();  // Read *all* data from the PS2 controller (buttons, analogs, pressure)
    processPS2Data(ps2data, report);


    // Send HID report (if connected)
    if (deviceConnected) {
        // Debug: Print the raw report data being sent.  VERY useful for troubleshooting.
        Serial.print("Sending report: ");
        for (int i = 0; i < sizeof(gamepad_report_t); i++) {
            Serial.printf("%02X ", ((uint8_t*)&report)[i]);
        }
        Serial.println();

        hid->setReport(0x01, &report, sizeof(report)); //report type 1 (input)
        hid->notify(); // Explicitly notify the central (important!)
    }

    delay(20); // Adjust delay as needed.  This controls the report rate.
}
