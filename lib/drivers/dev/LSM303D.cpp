#include "LSM303D.h"                                   // Match header

// ---- Low-level helpers ------------------------------------------------------

bool LSM303D::writeReg(uint8_t reg, uint8_t val) {
    // Write one register at the resolved address (or primary if begin hasn't run yet).
    return bus_.writeRegister(addrActive_ ? addrActive_ : addrPrimary_, reg, val);
}

bool LSM303D::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    // Use auto-increment (0x80) to read consecutive registers in a single transaction.
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, (uint8_t)(startReg | 0x80), buf, len);
}

bool LSM303D::readByte(uint8_t reg, uint8_t& val) {
    // Read a single byte into 'val'.
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, reg, &val, 1);
}

// Probe primary first, then alternate; accept only the correct WHO_AM_I.
bool LSM303D::probeAndSelectAddress() {
    uint8_t who = 0;                                    // Buffer for WHO_AM_I
    addrActive_ = 0;                                    // Clear active until known

    // Try primary
    addrActive_ = addrPrimary_;                         // Tentatively set primary
    if (readByte(REG_WHO_AM_I, who) && who == kWhoAmI) {
        return true;                                    // Primary confirmed
    }

    // Try alternate
    addrActive_ = addrAlt_;                             // Switch to alternate
    if (readByte(REG_WHO_AM_I, who) && who == kWhoAmI) {
        return true;                                    // Alternate confirmed
    }

    // Neither responded with the expected ID
    addrActive_ = 0;                                    // Mark invalid
    return false;                                       // Probe failed
}

// ---- Public API -------------------------------------------------------------

bool LSM303D::begin() {
    // Determine which address is populated by probing WHO_AM_I.
    if (!probeAndSelectAddress()) {                     // Fail out if no valid device is found
        return false;
    }

    // -------- Accelerometer configuration --------
    // CTRL1 (0x20): Accel ODR = 800 Hz (AODR=1001 -> 0x90), enable XYZ axes (X/Y/Z=1 -> 0x07) => 0x97
    if (!writeReg(REG_CTRL1, 0x97)) return false;      // High ODR, all axes on

    // CTRL2 (0x21): Accel full-scale ±16 g (AFS=11 -> 0x30). Leave filter bits at default => 0x30
    if (!writeReg(REG_CTRL2, 0x30)) return false;      // Set FS to ±16 g

    // -------- Magnetometer configuration --------
    // CTRL5 (0x24): Enable temp sensor (if desired), set mag resolution/high res and ODR.
    //  A common robust config used in the field is 0x64 (M_RES=11 high, M_ODR ~6.25–25 Hz, BDU=1).
    if (!writeReg(REG_CTRL5, 0x64)) return false;      // High resolution, BDU set, mag ODR typical

    // CTRL6 (0x25): Mag full-scale ±4 gauss (MFS=00 -> 0x00 or 0x20 depending on variant). Use 0x20 for ±4 gauss.
    if (!writeReg(REG_CTRL6, 0x20)) return false;      // Set mag FS to ±4 gauss

    // CTRL7 (0x26): Continuous-conversion mode for magnetometer (MD=00), high-pass off
    if (!writeReg(REG_CTRL7, 0x00)) return false;      // Continuous mode for mag

    return true;                                        // Initialization successful
}

bool LSM303D::readAccel(float& ax_g, float& ay_g, float& az_g) {
    uint8_t raw[6];                                     // Buffer for 3 axes (L,H)*3
    if (!readRegs(REG_OUT_X_L_A, raw, sizeof(raw))) {   // Burst read accel outputs
        return false;                                   // Abort on I²C error
    }

    // The LSM303D accel output is 16-bit two's complement (left-justified in some modes).
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);      // Pack X as signed 16-bit
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);      // Pack Y
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);      // Pack Z

    // Convert to g using effective sensitivity at ±16 g (approx 0.000732 g/LSB).
    ax_g = x * kAccel_g_per_LSB;                        // Scale X to g
    ay_g = y * kAccel_g_per_LSB;                        // Scale Y to g
    az_g = z * kAccel_g_per_LSB;                        // Scale Z to g

    return true;                                        // Success
}

bool LSM303D::readMag(float& mx_uT, float& my_uT, float& mz_uT) {
    uint8_t raw[6];                                     // Buffer for 3 axes (L,H)*3
    if (!readRegs(REG_OUT_X_L_M, raw, sizeof(raw))) {   // Burst read magnetometer outputs
        return false;                                   // Abort on I²C error
    }

    // Pack two's complement values (note: LSM303D axis order is X,Y,Z).
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);      // Pack X
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);      // Pack Y
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);      // Pack Z

    // Convert to microtesla using sensitivity 0.016 µT/LSB at ±4 gauss.
    mx_uT = x * kMag_uT_per_LSB;                        // Scale X to µT
    my_uT = y * kMag_uT_per_LSB;                        // Scale Y to µT
    mz_uT = z * kMag_uT_per_LSB;                        // Scale Z to µT

    return true;                                        // Success
}
