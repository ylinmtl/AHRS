#include "L3GD20H.h"                               // Include our own declarations for consistency

// ---- Low-level helpers ------------------------------------------------------

bool L3GD20H::writeReg(uint8_t reg, uint8_t val) {
    // Delegate a single register write to the shared Bus instance.
    return bus_.writeRegister(addrActive_ ? addrActive_ : addrPrimary_, reg, val);
}

bool L3GD20H::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    // Use auto-increment by OR'ing the subaddress with 0x80 (per ST convention).
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, (uint8_t)(startReg | 0x80), buf, len);
}

bool L3GD20H::readByte(uint8_t reg, uint8_t& val) {
    // Read a single byte from the device into 'val'.
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, reg, &val, 1);
}

// Try the primary address first; if WHO_AM_I is wrong (or read fails), try the alternate.
bool L3GD20H::probeAndSelectAddress() {
    uint8_t who = 0;                                  // Buffer for WHO_AM_I value
    addrActive_ = 0;                                  // Clear active address until proven

    // Try primary address
    addrActive_ = addrPrimary_;                       // Tentatively mark primary as active
    if (readByte(REG_WHO_AM_I, who) &&                // If read succeeds...
        (who == kWhoAmI_L3GD20H || who == kWhoAmI_L3GD20)) { // ...and ID matches accepted values
        return true;                                  // Primary address confirmed
    }

    // Try alternate address
    addrActive_ = addrAlt_;                           // Switch tentative active address to alternate
    if (readByte(REG_WHO_AM_I, who) &&                // Attempt read at alt address
        (who == kWhoAmI_L3GD20H || who == kWhoAmI_L3GD20)) { // Accept D7 (20H) or D4 (20)
        return true;                                  // Alternate confirmed
    }

    // Neither address responded properly
    addrActive_ = 0;                                  // Mark as not found
    return false;                                     // Probe failed
}

// ---- Public API -------------------------------------------------------------

bool L3GD20H::begin() {
    // Resolve which I²C address is present by probing WHO_AM_I.
    if (!probeAndSelectAddress()) {                   // If neither address works...
        return false;                                 // ...fail init
    }

    // CTRL1: ODR=800 Hz (DR=11), BW=min (00), PD=1 (normal), Zen/Yen/Xen=1 -> 0xCF
    if (!writeReg(REG_CTRL1, 0xCF)) return false;    // Configure data rate and enable axes

    // CTRL4: BDU=1 (bit7), FS=±2000 dps (FS1:FS0=10) -> 0xA0
    if (!writeReg(REG_CTRL4, 0xA0)) return false;    // Latch data until both bytes read, set full-scale

    // CTRL2/CTRL5 left at defaults for simplicity (HPF/FIFO not used in this minimal path)

    return true;                                      // Initialization successful
}

bool L3GD20H::read(float& gx_dps, float& gy_dps, float& gz_dps) {
    uint8_t raw[6];                                   // Buffer for 3 axes, little endian L,H per axis
    if (!readRegs(REG_OUT_X_L, raw, sizeof(raw))) {   // Burst read X/Y/Z low..high with auto-inc
        return false;                                 // Abort on I²C error
    }

    // Pack little-endian pairs into signed 16-bit raw values
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);    // X = X_H << 8 | X_L
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);    // Y = Y_H << 8 | Y_L
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);    // Z = Z_H << 8 | Z_L

    // Convert raw counts to dps using the ±2000 dps sensitivity (0.07 dps/LSB)
    gx_dps = x * kSensitivityDPSPerLSB;               // Scale X to dps
    gy_dps = y * kSensitivityDPSPerLSB;               // Scale Y to dps
    gz_dps = z * kSensitivityDPSPerLSB;               // Scale Z to dps

    return true;                                      // Report success to caller
}
