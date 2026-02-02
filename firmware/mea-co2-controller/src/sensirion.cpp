#include "sensirion.h"

bool SFC6000D::begin(TwoWire& wire, uint8_t i2c_addr) {
  _addr = i2c_addr;
  _dev.begin(wire, _addr);
  _begun = true;
  _measuring = false;
  return true;
}

/* ---------- Robust start / recovery ---------- */

bool SFC6000D::robustStartCO2(bool primeValve) {
  if (!_begun) return false;

  // 1) Stop anything that might be running (best effort)
  (void)_dev.stopContinuousMeasurement();
  delay(50);

  // 2) Clear any forced valve state
  (void)_dev.resetForceCloseValve();
  (void)_dev.resetForceOpenValve();
  delay(50);

  // 3) Start CO2 measurement
  int16_t err = _dev.startCo2ContinuousMeasurement();
  if (!ok(err)) return logIfError(err, F("startCo2ContinuousMeasurement"));

  _measuring = true;

  // 4) Allow internal pipeline to fill
  delay(200);

  // 5) Reset measurement buffer pointer (important after state changes)
  (void)_dev.resetPointerToMeasurementBuffer();
  delay(10);

  // 6) Optional valve priming (empirically important)
  if (primeValve) {
    (void)_dev.forceOpenValve();
    delay(500);                       // short pulse, not a mode
    (void)_dev.resetForceOpenValve();
    delay(50);
  }

  return true;
}

bool SFC6000D::recoverFromReadError() {
  Serial.println(F("[SFC] read error -> attempting recovery"));
  _measuring = false;
  return robustStartCO2(false);       // no priming during recovery
}

/* ---------- Public API ---------- */

bool SFC6000D::mfcOn() {
  if (!_begun) return false;

  // Prime valve on initial startup (matches observed good behavior)
  return robustStartCO2(true);
}

bool SFC6000D::mfcOff() {
  if (!_begun) return false;

  // Command zero flow (slm)
  int16_t err = _dev.updateSetpoint(0.0f);
  if (!ok(err)) return logIfError(err, F("updateSetpoint(0)"));

  (void)_dev.resetPointerToMeasurementBuffer();
  delay(10);

  (void)_dev.stopContinuousMeasurement();
  _measuring = false;
  return true;
}

bool SFC6000D::mfcSetFlowSccm(float sccm) {
  if (!_begun) return false;
  if (sccm < 0.0f) sccm = 0.0f;

  const float slm = sccm / 1000.0f;
  int16_t err = _dev.updateSetpoint(slm);
  if (!ok(err)) return logIfError(err, F("updateSetpoint"));

  (void)_dev.resetPointerToMeasurementBuffer();
  return true;
}

bool SFC6000D::mfcReadFlowSccm(float& out_sccm,
                              uint16_t avg_samples,
                              uint16_t inter_sample_ms) {
  if (!_begun || !_measuring) return false;
  if (avg_samples == 0) avg_samples = 1;

  float sum_slm = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < avg_samples; ++i) {
    float slm = NAN;
    int16_t err = _dev.readFlow(slm);

    if (ok(err)) {
      sum_slm += slm;
      ++good;
    } else {
      logIfError(err, F("readFlow"));
      if (!recoverFromReadError()) break;
    }

    if (inter_sample_ms) delay(inter_sample_ms);
  }

  if (good == 0) return false;

  out_sccm = (sum_slm / good) * 1000.0f;
  return true;
}

bool SFC6000D::readTemperatureC(float& out_degC) {
  if (!_begun) return false;
  int16_t err = _dev.readTemperature(out_degC);
  return logIfError(err, F("readTemperature"));
}

bool SFC6000D::mfcForceCloseValve() {
  if (!_begun) return false;

  if (!_measuring) {
    if (!robustStartCO2(false)) return false;
  }

  int16_t err = _dev.forceCloseValve();
  return logIfError(err, F("forceCloseValve"));
}

bool SFC6000D::mfcResetForceCloseValve() {
  if (!_begun) return false;
  int16_t err = _dev.resetForceCloseValve();
  return logIfError(err, F("resetForceCloseValve"));
}

/* ---------- Logging ---------- */

bool SFC6000D::logIfError(int16_t err, const __FlashStringHelper* where) {
  if (ok(err)) return true;
  Serial.print(F("[SFC] "));
  Serial.print(where);
  Serial.print(F(" err="));
  Serial.println(err);
  return false;
}
