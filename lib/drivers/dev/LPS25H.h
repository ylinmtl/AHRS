#pragma once
#include <Arduino.h>
#include "BusI2C.h"
#include "Config.h"

class LPS25H {
public:
    explicit LPS25H(BusI2C& bus, uint8_t addr = Config::Addr::LPS25H)
    : bus_(bus), addr_(addr) {}

    bool begin();
    bool read(float& pressure_Pa, float& temp_C);
    float getRateHz() const { return 25.0f; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t startReg, uint8_t* buf, size_t len);

    BusI2C& bus_;
    uint8_t addr_;
};
