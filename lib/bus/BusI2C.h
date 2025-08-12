#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Bus.h"
#include "Config.h"

class BusI2C : public Bus {
public:
    explicit BusI2C(unsigned clock_hz = Config::I2C_CLOCK_HZ) : clock_hz_(clock_hz) {}
    bool begin() override;
    bool writeRegister(uint8_t dev7, uint8_t reg, uint8_t val) override;
    bool readRegisters(uint8_t dev7, uint8_t reg, uint8_t* buf, size_t len) override;

    // Convenience for ST drivers
    bool write(uint8_t dev7, uint8_t reg, const uint8_t* data, size_t len);
    bool read(uint8_t dev7, uint8_t reg, uint8_t* data, size_t len);
private:
    unsigned clock_hz_;
};
