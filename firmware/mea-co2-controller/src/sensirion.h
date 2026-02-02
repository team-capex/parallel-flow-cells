#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSFX6XXX.h>

// Default I2C address
#ifndef SFC6XXX_I2C_ADDR
#define SFC6XXX_I2C_ADDR 0x24
#endif

class SFC6000D {
public:
  bool begin(TwoWire& wire = Wire, uint8_t i2c_addr = SFC6XXX_I2C_ADDR);

  // Controller control
  bool mfcOn();                       // robust CO2 start
  bool mfcOff();                      // setpoint=0, stop measurement
  bool mfcSetFlowSccm(float sccm);

  // Read flow (sccm), averaged
  bool mfcReadFlowSccm(float& out_sccm,
                       uint16_t avg_samples = 10,
                       uint16_t inter_sample_ms = 2);

  // Optional diagnostics
  bool readTemperatureC(float& out_degC);

  // Valve override
  bool mfcForceCloseValve();
  bool mfcResetForceCloseValve();

  uint8_t address() const { return _addr; }

private:
  SensirionI2cSfx6xxx _dev;
  uint8_t _addr = SFC6XXX_I2C_ADDR;
  bool _begun = false;
  bool _measuring = false;

  // Helpers
  static inline bool ok(int16_t err) { return err == 0; }
  bool logIfError(int16_t err, const __FlashStringHelper* where);

  // Robust internals
  bool robustStartCO2(bool primeValve);
  bool recoverFromReadError();
};
