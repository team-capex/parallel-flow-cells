#include <Arduino.h>
#include "leds.h"
#include "loadcells.h"
#include "ble.h"

#include <string>
#include <stdio.h>

// --- Pins ---
static const uint8_t PIN_BUTTON_TARE = 42;   // active LOW with internal pull-up

// --- State ---
static const uint16_t BTN_DEBOUNCE_MS = 30;
static const uint16_t BTN_TARE_TRIGGER_MS = 60;

// --- add near top of main.cpp, after other consts ---
static float CAL_KNOWN_LOAD_KG  = 200.0f;       // EDIT to your calibration mass
static const uint32_t BTN_LONG_MS       = 5000;      // 5 seconds to enter calibration
static bool     btnDown = false;
static uint32_t btnDownAt = 0;

float masses[NUM_CHANNELS] = {0};
float mass = 0;
String action;
#define MSG_BUF_SIZE 256

#define SERVICE_UUID        "fc193d64-a716-4734-8002-abe9205b4dc8"
#define CHARACTERISTIC_UUID "e0535719-06e3-46e6-a89a-4dfeb0c4f01a"
#define DEVICE_NAME         "LoadCellBoard"

// --- LED config (moved implementation to led.*) ---
#define NUMPIXELS 2
#define LEDPIN 41
static const uint8_t OUTER[2] = {0,1};
static const uint8_t CENTER_IDX = 0; //not used
LedAnimations LEDS;

// ------ Prototypes ------
static void handleButton();
void buildMassMessage(char* buffer, size_t bufferSize, const float masses[], size_t n);
// Route arg reads: prefer BLE message args when present, else Serial.
static inline String readArg(char delim = ',') {
  if (ble_message_received()) return ble_read_arg(delim);
  return Serial.readStringUntil(delim);
}
static inline void respond(const String& s) { ble_respond(s); }

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Load Cell Board V1");

  // LED init
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6);

  // Button
  pinMode(PIN_BUTTON_TARE, INPUT_PULLUP);

  Serial.println("Setting up load cells..");

  // Load cells
  loadcellsInit();
  loadcellsTareAll();                 // initial tare on boot

  // BLE
  ble_begin(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);

  Serial.println("Available functions:");
  Serial.println("readCell(int cell)");
  Serial.println("tareCell(int cell)");
  Serial.println("tareAll()");
  Serial.println("calibrateCell(int cell)");
  Serial.println("statusCheck()");
  Serial.println("changeCalWeight(float weight)");

  Serial.println("# Ready");
  LEDS.pulse(HAPPY);
}

void loop() {
  delay(200);

  // Read either BLE message or Serial command
  if (ble_message_received() || Serial.available() > 0) {
    LEDS.pulse(TRANSMIT);

    if (!ble_message_received()) {
      action = Serial.readStringUntil('(');
    } else {
      action = ble_get_action();
    }

    if (action == "readCell") {
      int cell = readArg(')').toInt();

      bool ok = loadcellRead(cell, mass);
      if (!ok) {
        LEDS.pulse(ERROR, 2);
        respond("Failed to read load cell");
      }
      else {
        respond(String(mass, 3));
      }
    }

    else if (action == "tareCell") {
      int cell = readArg(')').toInt();

      loadcellTare(cell);
      LEDS.pulse(HAPPY, 1, 20);

      respond("# Done");
    }

    else if (action == "calibrateCell") {
      int cell = readArg(')').toInt();
      
      bool ok = loadcellCalibrate(cell, CAL_KNOWN_LOAD_KG);
      if (!ok) {
        LEDS.pulse(ERROR, 2);
        respond("Failed to calibrate load cell");
      }
      else {
        LEDS.pulse(HAPPY, 1, 20);
        respond("# Done");
      }
    }

    else if (action == "tareAll") {
      (void)readArg(')');
      loadcellsTareAll();
      respond("# Done");
    }

    else if (action == "statusCheck") {
      (void)readArg(')');
      respond("# Ready");
      LEDS.pulse(BLUETOOTH);
    }

    else if (action == "changeCalWeight") {
      CAL_KNOWN_LOAD_KG = readArg(')').toFloat();
      respond("# Done");
    }

    else {
      (void)readArg(')');
      Serial.println("Unknown command: " + action);
      LEDS.pulse(ERROR, 2);
    }
  }
}