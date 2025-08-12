#pragma once                                          // Guard against multiple inclusion
#include <Arduino.h>                                  // Arduino core for types
#include "Bus.h"                                      // Abstract Bus used for I²C access

/**
 * @file LPS25H.h
 * @brief Driver for ST LPS25H barometer + temperature.
 *
 *  - Outputs pressure in **Pa** and temperature in **°C**.
 *  - Configures 25 Hz ODR with BDU on.
 *  - Tries primary I²C address then falls back to alternate if needed.
 *  - Centralizes all register addresses here for clarity.
 */
class LPS25H {
public:
    // ===== I²C addresses (7-bit) =====
    static constexpr uint8_t kAddrPrimary   = 0x5D;   // SA0 high (typical on many breakout boards)
    static constexpr uint8_t kAddrAlternate = 0x5C;   // SA0 low

    // ===== WHO_AM_I value =====
    static constexpr uint8_t kWhoAmI = 0xBD;          // Expected WHO_AM_I response for LPS25H

    // ===== Register map (subset used) =====
    static constexpr uint8_t REG_REF_P_XL   = 0x08;   // Reference pressure (unused)
    static constexpr uint8_t REG_WHO_AM_I   = 0x0F;   // Device ID
    static constexpr uint8_t REG_CTRL1      = 0x20;   // PD, ODR, BDU, DIFF_EN
    static constexpr uint8_t REG_CTRL2      = 0x21;   // Reset, FIFO, I2C/SPI (unused)
    static constexpr uint8_t REG_RES_CONF   = 0x10;   // Resolution config (unused here)
    static constexpr uint8_t REG_STATUS     = 0x27;   // Status (unused)
    static constexpr uint8_t REG_PRESS_OUT_XL = 0x28; // Pressure out low byte (auto-inc)
    static constexpr uint8_t REG_PRESS_OUT_L  = 0x29; // Pressure out mid byte
    static constexpr uint8_t REG_PRESS_OUT_H  = 0x2A; // Pressure out high byte
    static constexpr uint8_t REG_TEMP_OUT_L   = 0x2B; // Temperature out low byte
    static constexpr uint8_t REG_TEMP_OUT_H   = 0x2C; // Temperature out high byte

    /**
     * @brief Construct the driver with an abstract Bus and optional addresses.
     * @param bus          Reference to shared Bus (e.g., BusI2C).
     * @param addrPrimary  Preferred 7-bit I²C address.
     * @param addrAlt      Alternate 7-bit I²C address.
     */
    explicit LPS25H(Bus& bus,
                    uint8_t addrPrimary = kAddrPrimary,
                    uint8_t addrAlt     = kAddrAlternate)
    : bus_(bus)                 // Store bus reference
    , addrPrimary_(addrPrimary) // Save primary address
    , addrAlt_(addrAlt)         // Save alternate address
    , addrActive_(0)            // Active address to be resolved in begin()
    {}

    /**
     * @brief Initialize hardware (WHO_AM_I, power on, 25 Hz ODR, BDU on).
     * @return true if a valid device was found and configured.
     */
    bool begin();

    /**
     * @brief Read pressure and temperature.
     * @param pressure_Pa  Output pressure in **pascal**.
     * @param temp_C       Output temperature in **degrees Celsius**.
     * @return true on success.
     */
    bool read(float& pressure_Pa, float& temp_C);

    /**
     * @brief Nominal data rate (Hz).
     */
    float getRateHz() const { return 25.0f; }

    /**
     * @brief Active I²C address used after successful begin().
     */
    uint8_t activeAddress() const { return addrActive_; }

private:
    // ===== Helpers =====
    bool   writeReg(uint8_t reg, uint8_t val);        // Write one register
    bool   readRegs(uint8_t startReg, uint8_t* buf, size_t len); // Burst read with auto-inc
    bool   readByte(uint8_t reg, uint8_t& val);       // Read one register
    bool   probeAndSelectAddress();                   // Primary then alternate WHO_AM_I

    // ===== State =====
    Bus&    bus_;             // Shared bus
    uint8_t addrPrimary_;     // Preferred I²C address
    uint8_t addrAlt_;         // Alternate I²C address
    uint8_t addrActive_;      // Resolved address after probing
};
