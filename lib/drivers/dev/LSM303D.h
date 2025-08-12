#pragma once
#include <Arduino.h>
#include "BusI2C.h"
#include "Config.h"

class LSM303D {
public:
    explicit LSM303D(BusI2C& bus, uint8_t addr = Config::Addr::LSM303D)
    : bus_(bus), addr_(addr) {}

    bool begin();
    bool readAccel(float& ax_g, float& ay_g, float& az_g);
    bool readMag(float& mx_uT, float& my_uT, float& mz_uT);

    float getAccelRateHz() const { return 800.0f; }
    float getMagRateHz()   const { return 25.0f;  }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t startReg, uint8_t* buf, size_t len);

    BusI2C& bus_;
    uint8_t addr_;

    static constexpr float kAccel_g_per_LSB = 0.000732f; // 0.732 mg/LSB @ ±16 g
    static constexpr float kMag_uT_per_LSB  = 0.016f;    // 0.160 mgauss/LSB = 0.016 µT/LSB @ ±4 gauss
};
