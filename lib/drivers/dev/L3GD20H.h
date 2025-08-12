#pragma once                                          // Ensure this header is included only once per TU
#include <Arduino.h>                                  // Arduino types and timing helpers
#include "Bus.h"                                      // Abstract bus interface used by all drivers

/**
 * @file L3GD20H.h
 * @brief Driver for ST L3GD20H 3-axis gyroscope.
 *
 *  - Provides ±2000 dps full-scale by default (matches project requirement).
 *  - Configures 800 Hz ODR and enables XYZ axes.
 *  - Returns **degrees per second** (dps) in read().
 *  - Tries primary address first, then falls back to the alternate address.
 *  - All register addresses are centralized here.
 */
class L3GD20H {
public:
    // ===== I²C addresses (7-bit) =====
    static constexpr uint8_t kAddrPrimary   = 0x6B;   // Typical when SA0 is pulled HIGH
    static constexpr uint8_t kAddrAlternate = 0x6A;   // Alternate when SA0 is LOW

    // ===== WHO_AM_I values this driver accepts =====
    static constexpr uint8_t kWhoAmI_L3GD20H = 0xD7;  // L3GD20H WHO_AM_I response
    static constexpr uint8_t kWhoAmI_L3GD20  = 0xD4;  // Older L3GD20 (accepted as compatible)

    // ===== Register map (subset used) =====
    static constexpr uint8_t REG_WHO_AM_I  = 0x0F;    // Device ID register
    static constexpr uint8_t REG_CTRL1     = 0x20;    // ODR / bandwidth / power / axis enable
    static constexpr uint8_t REG_CTRL2     = 0x21;    // High-pass filter config (unused here)
    static constexpr uint8_t REG_CTRL3     = 0x22;    // Interrupts (unused here)
    static constexpr uint8_t REG_CTRL4     = 0x23;    // BDU, scale, endianness
    static constexpr uint8_t REG_CTRL5     = 0x24;    // FIFO/HPF controls (unused here)
    static constexpr uint8_t REG_STATUS    = 0x27;    // Status bits (unused in this minimal driver)
    static constexpr uint8_t REG_OUT_X_L   = 0x28;    // X low  byte (auto-inc capable)
    static constexpr uint8_t REG_OUT_X_H   = 0x29;    // X high byte
    static constexpr uint8_t REG_OUT_Y_L   = 0x2A;    // Y low  byte
    static constexpr uint8_t REG_OUT_Y_H   = 0x2B;    // Y high byte
    static constexpr uint8_t REG_OUT_Z_L   = 0x2C;    // Z low  byte
    static constexpr uint8_t REG_OUT_Z_H   = 0x2D;    // Z high byte

    // ===== Scale and rate constants =====
    static constexpr float   kSensitivityDPSPerLSB = 0.07f; // 70 mdps/LSB at ±2000 dps
    static constexpr float   kSampleRateHz         = 800.0f; // Configured ODR

    /**
     * @brief Construct the driver with an abstract bus and optional preferred addresses.
     * @param bus          Reference to a concrete Bus (e.g., BusI2C) shared with other drivers.
     * @param addrPrimary  Preferred 7-bit I²C address to try first.
     * @param addrAlt      Alternate 7-bit I²C address to try if primary probe fails.
     */
    explicit L3GD20H(Bus& bus,
                     uint8_t addrPrimary = kAddrPrimary,
                     uint8_t addrAlt     = kAddrAlternate)
    : bus_(bus)                 // Store bus reference for later use
    , addrPrimary_(addrPrimary) // Save preferred address
    , addrAlt_(addrAlt)         // Save alternate address
    , addrActive_(0)            // Active address will be decided in begin()
    {}

    /**
     * @brief Initialize the device: probe WHO_AM_I, configure ODR/scale/BDU, enable axes.
     * @return true if initialization succeeded on either primary or alternate address.
     */
    bool begin();

    /**
     * @brief Read angular rate in degrees per second from XYZ axes.
     * @param gx_dps X axis angular rate (output).
     * @param gy_dps Y axis angular rate (output).
     * @param gz_dps Z axis angular rate (output).
     * @return true on successful I²C transaction and data conversion.
     */
    bool read(float& gx_dps, float& gy_dps, float& gz_dps);

    /**
     * @brief Return the configured output data rate in Hz (used by SensorHub pacing).
     */
    float getSampleRateHz() const { return kSampleRateHz; }

    /**
     * @brief Return the detected 7-bit I²C address in use after begin().
     */
    uint8_t activeAddress() const { return addrActive_; }

private:
    // ===== Small helpers to keep cpp readable =====
    bool   writeReg(uint8_t reg, uint8_t val);        // Write one register
    bool   readRegs(uint8_t startReg, uint8_t* buf, size_t len); // Read len bytes starting at startReg (auto-inc)
    bool   readByte(uint8_t reg, uint8_t& val);       // Read a single register
    bool   probeAndSelectAddress();                   // Try primary, then alternate, validate WHO_AM_I

    // ===== State =====
    Bus&    bus_;             // Shared bus (I²C in your current build)
    uint8_t addrPrimary_;     // Preferred address to try first
    uint8_t addrAlt_;         // Fallback address
    uint8_t addrActive_;      // The address that responded and passed WHO_AM_I check
};
