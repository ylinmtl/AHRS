#pragma once
#include <Arduino.h>

class Bus {
public:
    virtual ~Bus() = default;
    virtual bool begin() = 0;
    virtual bool writeRegister(uint8_t dev7, uint8_t reg, uint8_t val) = 0;
    virtual bool readRegisters(uint8_t dev7, uint8_t reg, uint8_t* buf, size_t len) = 0;
};
