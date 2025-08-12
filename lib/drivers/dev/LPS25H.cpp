#include "LPS25H.h"

static constexpr uint8_t WHO_AM_I     = 0x0F; // expect 0xBD
static constexpr uint8_t CTRL_REG1    = 0x20;
static constexpr uint8_t PRESS_OUT_XL = 0x28; // 24-bit P
static constexpr uint8_t TEMP_OUT_L   = 0x2B; // 16-bit T

bool LPS25H::begin() {
    uint8_t id=0;
    if (!readRegs(WHO_AM_I, &id, 1)) return false;
    if (id != 0xBD) return false;
    // CTRL_REG1: PD=1 (bit7), ODR=25Hz (100<<4), BDU=1 (bit2) -> 0xC4
    if (!writeReg(CTRL_REG1, 0xC4)) return false;
    return true;
}

bool LPS25H::read(float& pressure_Pa, float& temp_C) {
    uint8_t buf[5];
    if (!readRegs(PRESS_OUT_XL | 0x80, buf, 5)) return false;
    int32_t p24 = (int32_t)((int32_t)buf[2] << 16 | (int32_t)buf[1] << 8 | buf[0]);
    if (p24 & 0x00800000) p24 |= 0xFF000000; // sign extend if needed
    int16_t t16 = (int16_t)((int16_t)buf[4] << 8 | buf[3]);
    float p_hPa = (float)p24 / 4096.0f;
    pressure_Pa = p_hPa * 100.0f;
    temp_C = 42.5f + ((float)t16 / 480.0f);
    return true;
}

bool LPS25H::writeReg(uint8_t reg, uint8_t val) {
    return bus_.write(addr_, reg, &val, 1);
}
bool LPS25H::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    return bus_.read(addr_, startReg, buf, len);
}
