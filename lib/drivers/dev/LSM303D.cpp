#include "LSM303D.h"

static constexpr uint8_t WHO_AM_I   = 0x0F; // expect 0x49
static constexpr uint8_t CTRL0      = 0x1F;
static constexpr uint8_t CTRL1      = 0x20;
static constexpr uint8_t CTRL2      = 0x21;
static constexpr uint8_t CTRL5      = 0x24;
static constexpr uint8_t CTRL6      = 0x25;
static constexpr uint8_t CTRL7      = 0x26;
static constexpr uint8_t OUT_X_L_A  = 0x28; // accel
static constexpr uint8_t OUT_X_L_M  = 0x08; // mag

bool LSM303D::begin() {
    uint8_t id=0;
    if (!readRegs(WHO_AM_I, &id, 1)) return false;
    if (id != 0x49) return false;

    // CTRL1: AODR=800Hz (1001), BDU=1, enable XYZ => 0x9F
    if (!writeReg(CTRL1, 0x9F)) return false;

    // CTRL2: AFS=±16g (100), ABW=362Hz (10) => 0xA0
    if (!writeReg(CTRL2, 0xA0)) return false;

    // CTRL5: TEMP_EN=1<<7, M_RES=11<<5, M_ODR=25Hz (011<<2)
    const uint8_t ctrl5 = (1u<<7) | (3u<<5) | (0b011<<2);
    if (!writeReg(CTRL5, ctrl5)) return false;

    // CTRL6: MFS=±4 gauss (01<<5) => 0x20
    if (!writeReg(CTRL6, 0x20)) return false;

    // CTRL7: magnetometer continuous mode (MD=00)
    if (!writeReg(CTRL7, 0x00)) return false;

    return true;
}

bool LSM303D::readAccel(float& ax_g, float& ay_g, float& az_g) {
    uint8_t raw[6];
    if (!readRegs(OUT_X_L_A | 0x80, raw, 6)) return false;
    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    ax_g = x * kAccel_g_per_LSB;
    ay_g = y * kAccel_g_per_LSB;
    az_g = z * kAccel_g_per_LSB;
    return true;
}

bool LSM303D::readMag(float& mx_uT, float& my_uT, float& mz_uT) {
    uint8_t raw[6];
    if (!readRegs(OUT_X_L_M | 0x80, raw, 6)) return false;
    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    mx_uT = x * kMag_uT_per_LSB;
    my_uT = y * kMag_uT_per_LSB;
    mz_uT = z * kMag_uT_per_LSB;
    return true;
}

bool LSM303D::writeReg(uint8_t reg, uint8_t val) {
    return bus_.write(addr_, reg, &val, 1);
}
bool LSM303D::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    return bus_.read(addr_, startReg, buf, len);
}
