#include <Arduino.h>
#include "leds.h"
#include "loadcells.h"

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

// --- LED config (moved implementation to led.*) ---
#define NUMPIXELS 2
#define LEDPIN 41
static const uint8_t OUTER[2] = {0,1};
static const uint8_t CENTER_IDX = 0; //not used
LedAnimations LEDS;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Load Cell Board V1");
  Serial.println("Preparing load cells..");

  // Load cells first (takes a while)
  loadcellsInit(); 

  // LED init
  pinMode(LEDPIN, OUTPUT);
  LEDS.begin(LEDPIN, NUMPIXELS, CENTER_IDX, OUTER, 6);

  // Button
  pinMode(PIN_BUTTON_TARE, INPUT_PULLUP);               

  Serial.println("Available functions:");
  Serial.println("readCell(int cell)");
  Serial.println("tareCell(int cell)");
  Serial.println("tareAll()");
  Serial.println("calibrateCell(int cell)");
  Serial.println("statusCheck()");
  Serial.println("changeCalWeight(float weight)");

  Serial.println("# Controller available");
  LEDS.pulse(HAPPY);
}

void loop() {
  delay(200);

  // Read either BLE message or Serial command
  if (Serial.available() > 0) {
    LEDS.pulse(TRANSMIT);

    action = Serial.readStringUntil('(');

    if (action == "readCell") {
      int cell = Serial.readStringUntil(')').toInt();

      bool ok = loadcellRead(cell, mass);
      if (!ok) {
        LEDS.pulse(ERROR, 2);
        Serial.println("Failed to read load cell");
      }
      else {
        Serial.println(String(mass, 3));
      }
    }

    else if (action == "tareCell") {
      int cell = Serial.readStringUntil(')').toInt();

      loadcellTare(cell);
      LEDS.pulse(HAPPY, 1, 20);

      Serial.println("# Done");
    }

    else if (action == "calibrateCell") {
      int cell = Serial.readStringUntil(')').toInt();
      
      bool ok = loadcellCalibrate(cell, CAL_KNOWN_LOAD_KG);
      if (!ok) {
        LEDS.pulse(ERROR, 2);
        Serial.println("Failed to calibrate load cell");
      }
      else {
        LEDS.pulse(HAPPY, 1, 20);
        Serial.println("# Done");
      }
    }

    else if (action == "tareAll") {
      (void)Serial.readStringUntil(')');
      loadcellsTareAll();
      Serial.println("# Done");
    }

    else if (action == "statusCheck") {
      (void)Serial.readStringUntil(')');
      Serial.println("# Ready");
      LEDS.pulse(BLUETOOTH);
    }

    else if (action == "changeCalWeight") {
      CAL_KNOWN_LOAD_KG = Serial.readStringUntil(')').toFloat();
      Serial.println("# Done");
    }

    else {
      (void)Serial.readStringUntil(')');
      Serial.println("Unknown command: " + action);
      LEDS.pulse(ERROR, 2);
    }
  }
}