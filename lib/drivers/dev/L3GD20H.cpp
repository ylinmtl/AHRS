#include "L3GD20H.h"

static constexpr uint8_t WHO_AM_I = 0x0F; // expect 0xD7
static constexpr uint8_t CTRL1    = 0x20;
static constexpr uint8_t CTRL4    = 0x23;
static constexpr uint8_t OUT_X_L  = 0x28; // auto-increment

bool L3GD20H::begin() {
    uint8_t id=0;
    if (!readRegs(WHO_AM_I, &id, 1)) return false;
    if (id != 0xD7) return false;
    // CTRL1: DR=11 (800Hz), BW=11, PD=1, Zen/Yen/Xen=1 => 0xFF
    if (!writeReg(CTRL1, 0xFF)) return false;
    // CTRL4: BDU=1 (bit7), FS=10 (±2000 dps) => 0x90
    if (!writeReg(CTRL4, 0x90)) return false;
    return true;
}

bool L3GD20H::read(float& gx_dps, float& gy_dps, float& gz_dps) {
    uint8_t raw[6];
    if (!readRegs(OUT_X_L | 0x80, raw, 6)) return false;
    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    gx_dps = x * kSensitivityDPSPerLSB;
    gy_dps = y * kSensitivityDPSPerLSB;
    gz_dps = z * kSensitivityDPSPerLSB;
    return true;
}

bool L3GD20H::writeReg(uint8_t reg, uint8_t val) {
    return bus_.write(addr_, reg, &val, 1);
}
bool L3GD20H::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    return bus_.read(addr_, startReg, buf, len);
}
