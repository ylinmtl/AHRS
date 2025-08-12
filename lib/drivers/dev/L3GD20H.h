#pragma once
#include <Arduino.h>
#include "BusI2C.h"
#include "Config.h"

class L3GD20H {
public:
    explicit L3GD20H(BusI2C& bus, uint8_t addr = Config::Addr::L3GD20H)
    : bus_(bus), addr_(addr) {}

    bool begin();
    bool read(float& gx_dps, float& gy_dps, float& gz_dps);
    float getSampleRateHz() const { return 800.0f; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t startReg, uint8_t* buf, size_t len);
    BusI2C& bus_;
    uint8_t addr_;
    static constexpr float kSensitivityDPSPerLSB = 0.07f; // 70 mdps/LSB @ 2000 dps
};
