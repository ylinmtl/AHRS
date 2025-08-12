#include "LPS25H.h"                                   // Include matching header

// ---- Low-level helpers ------------------------------------------------------

bool LPS25H::writeReg(uint8_t reg, uint8_t val) {
    // Write a single register via the shared Bus.
    return bus_.writeRegister(addrActive_ ? addrActive_ : addrPrimary_, reg, val);
}

bool LPS25H::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
    // Use auto-increment (0x80) for multi-byte reads (per ST convention).
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, (uint8_t)(startReg | 0x80), buf, len);
}

bool LPS25H::readByte(uint8_t reg, uint8_t& val) {
    // Read a single byte into 'val'.
    return bus_.readRegisters(addrActive_ ? addrActive_ : addrPrimary_, reg, &val, 1);
}

// Probe primary, then alternate; validate WHO_AM_I.
bool LPS25H::probeAndSelectAddress() {
    uint8_t who = 0;                                  // WHO_AM_I value holder
    addrActive_ = 0;                                  // Clear active until we know

    // Try primary address first
    addrActive_ = addrPrimary_;                       // Tentatively set to primary
    if (readByte(REG_WHO_AM_I, who) && who == kWhoAmI) {
        return true;                                  // Primary responded with expected ID
    }

    // Try alternate address
    addrActive_ = addrAlt_;                           // Switch to alternate
    if (readByte(REG_WHO_AM_I, who) && who == kWhoAmI) {
        return true;                                  // Alternate responded
    }

    // Neither address worked
    addrActive_ = 0;                                  // Mark invalid
    return false;                                     // Probe failed
}

// ---- Public API -------------------------------------------------------------

bool LPS25H::begin() {
    // Determine which address is present by checking WHO_AM_I.
    if (!probeAndSelectAddress()) {                   // Abort if no device responds properly
        return false;
    }

    // CTRL1: PD=1 (power on), ODR=25 Hz (bits 6:4 = 011), BDU=1 (bit2) -> 0xB4
    if (!writeReg(REG_CTRL1, 0xB4)) return false;    // Enable device, set ODR, and latch data until full read

    // (Optional) Additional configuration (resolution, FIFO) can be set here if needed.

    return true;                                      // Initialization done
}

bool LPS25H::read(float& pressure_Pa, float& temp_C) {
    uint8_t buf[5];                                   // Buffer: P[0..2], T[3..4]
    if (!readRegs(REG_PRESS_OUT_XL, buf, sizeof(buf))) {
        return false;                                 // Abort on I²C error
    }

    // Pressure is 24-bit unsigned, little endian: XL, L, H
    uint32_t p_raw = (uint32_t)buf[0]                 // XL
                   | ((uint32_t)buf[1] << 8)          // L
                   | ((uint32_t)buf[2] << 16);        // H

    // Temperature is 16-bit signed, little endian: L, H
    int16_t t_raw = (int16_t)((buf[4] << 8) | buf[3]); // Pack T as signed value

    // Convert to physical units per datasheet:
    //  Pressure in hPa = p_raw / 4096; 1 hPa = 100 Pa  => Pa = (p_raw / 4096) * 100
    pressure_Pa = (p_raw / 4096.0f) * 100.0f;         // Output pressure in pascal

    //  Temperature in °C = 42.5 + t_raw / 480
    temp_C = 42.5f + (t_raw / 480.0f);                // Output temperature in Celsius

    return true;                                      // Completed successfully
}
