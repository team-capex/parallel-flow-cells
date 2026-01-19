#include "loadcells.h"
#include "leds.h"
#include "storage.h"
#include <HX711.h>

static HX711 scales[NUM_CHANNELS];

uint8_t LC_IDS[NUM_CHANNELS] = { 1, 2, 3, 4 }; // edit as needed

const uint8_t LC_DOUT[NUM_CHANNELS] = { 14, 16, 39, 18 };
const uint8_t LC_SCK [NUM_CHANNELS] = { 15, 17, 40, 21 };

// --- EDIT THESE for your real calibration ---
float LC_SCALE [NUM_CHANNELS] = { 1.00f, 1.00f, 1.00f, 1.00f };
long  LC_OFFSET[NUM_CHANNELS] = { 0, 0, 0, 0 };
// -------------------------------------------


void loadcellsInit() {
  // 1) Wire up each HX711 with your pin map
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    scales[i].begin(LC_DOUT[i], LC_SCK[i]);
    delay(50); // allow HX711 to power up/stabilize

    // Apply compiled-in calibration as a baseline
    scales[i].set_scale(LC_SCALE[i]);    // scale = counts per unit (e.g., counts per kg)
    scales[i].set_offset(LC_OFFSET[i]);  // raw offset
  }

  // 2) Override with any persisted calibration from NVS
  if (storageLoadCalib(LC_SCALE, LC_OFFSET, NUM_CHANNELS)) {
    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
      scales[i].set_scale(LC_SCALE[i]);
      scales[i].set_offset(LC_OFFSET[i]);
    }
  }

  // 3) Discard a few initial readings to let everything settle
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    if (scales[i].is_ready()) {
      (void)scales[i].read();       // throwaway
      (void)scales[i].read_average(20);
    }
  }
}


void loadcellsTareAll() {
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    // Take a few readings to stabilize, then tare
    scales[i].tare(10); // averages 10 reads
    LC_OFFSET[i] = scales[i].get_offset();
  }

  // Keep current scales, but persist new offsets
  storageSaveCalib(LC_SCALE, LC_OFFSET, NUM_CHANNELS);

  Serial.println("Tared all channels");
}

void loadcellTare(int cell) {
  if (cell < 1 || cell > NUM_CHANNELS) {
    Serial.println("Invalid cell number!");
    return;
  } 

  delay(1000);
  
  scales[cell-1].tare(20); // averages 10 reads
  LC_OFFSET[cell-1] = scales[cell-1].get_offset();

  // Keep current scales, but persist new offsets
  storageSaveCalib(LC_SCALE, LC_OFFSET, NUM_CHANNELS);

  Serial.println("Tared channel " + String(cell));
}

bool loadcellsReadAll(float outMass[NUM_CHANNELS]) {
  bool ok = true;
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    if (scales[i].is_ready()) {
      // Average a few samples for stability but keep it fast
      outMass[i] = scales[i].get_units(5); // result already in units via set_scale()
    } else {
      ok = false;
      outMass[i] = NAN;
    }
  }
  return ok;
}

bool loadcellRead(int cell, float &outMass) {
  bool ok = false;

  if (cell < 1 || cell > NUM_CHANNELS) {
    Serial.println("Invalid cell number!");
    return ok;
  } 

  if (scales[cell-1].is_ready()) {
    // Average a few samples for stability but keep it fast
    outMass = scales[cell-1].get_units(5); // result already in units via set_scale()
    ok = true;
  } else {
    outMass = NAN;
  }
  
  return ok;
}

bool loadcellCalibrate(int cell, float knownMassKg) {
  bool ok = false;

  if (cell < 1 || cell > NUM_CHANNELS) {
    Serial.println("Invalid cell number!");
    return ok;
  } 

  delay(1000);

  if (scales[cell-1].is_ready()) {
    ok = true;
  } else {
    return ok;
  }

  long counts = scales[cell-1].get_value(20);
  if (counts == 0) counts = 1;              // avoid div0
  float scale = (float)counts / knownMassKg; // counts per kg (units = kg)

  LC_SCALE[cell-1]  = scale;
  scales[cell-1].set_scale(scale);

  // Persist to NVS
  storageSaveCalib(LC_SCALE, LC_OFFSET, NUM_CHANNELS);

  Serial.println("Calibrated channel " + String(cell) + " with " + String(knownMassKg, 3) + "kg");
  return ok;
}

bool loadcellsCalibrateAll(float knownMassKg, uint8_t btn_pin, LedAnimations &leds, uint16_t settleMsPerStep) {
  if (knownMassKg <= 0.0f) return false;

  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    // Indicate which channel we're calibrating (i+1 blue blinks)
    Serial.println("Calibrating channel " + String(i+1));
    leds.pulse(CALIBRATING, i+1, 20);
    delay(1000);

    // Wait for user confirmation to start 
    Serial.println("Press MODE to calibrate NO LOAD");
    do {leds.run(NEUTRAL);} while (digitalRead(btn_pin) == HIGH);

    // Step 1: NO LOAD
    leds.pulse(WARNING);   
    delay(settleMsPerStep);

    long offset = scales[i].read_average(20); // raw average counts
    scales[i].set_offset(offset);

    // Step 2: APPLY KNOWN LOAD
    Serial.println("Press MODE to calibrate LOAD (" + String(knownMassKg) + "Kg)");
    do {leds.run(NEUTRAL);} while (digitalRead(btn_pin) == HIGH);
    leds.pulse(BLUETOOTH);    
    delay(settleMsPerStep);

    // get_value subtracts offset internally -> raw counts due to knownMass
    long counts = scales[i].get_value(10);
    if (counts == 0) counts = 1;              // avoid div0
    float scale = (float)counts / knownMassKg; // counts per kg (units = kg)

    LC_OFFSET[i] = offset;
    LC_SCALE[i]  = scale;
    scales[i].set_scale(scale);

    // Persist to NVS after each
    storageSaveCalib(LC_SCALE, LC_OFFSET, NUM_CHANNELS);

    Serial.println("Calibration saved for channel " + String(i+1));

    // Confirm: green blink
    leds.pulse(HAPPY, 20);
    delay(1000);
  }

  return true;
}
