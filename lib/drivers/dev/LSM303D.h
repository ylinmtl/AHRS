#pragma once                                          // Prevent multiple inclusion
#include <Arduino.h>                                  // Arduino core
#include "Bus.h"                                      // Abstract Bus (I²C in your build)

/**
 * @file LSM303D.h
 * @brief Driver for ST LSM303D (accelerometer + magnetometer).
 *
 *  - Accel configured to ±16 g, ODR target 800 Hz (axes enabled).
 *  - Mag configured to ±4 gauss, continuous mode (typical ~25 Hz ODR).
 *  - Returns **g** (accel) and **µT** (mag) in read functions.
 *  - Tries primary address first, then alternate.
 *  - All register addresses centralized here.
 */
class LSM303D {
public:
    // ===== I²C addresses (7-bit) =====
    static constexpr uint8_t kAddrPrimary   = 0x1D;   // SA0 high (common)
    static constexpr uint8_t kAddrAlternate = 0x1E;   // SA0 low

    // ===== WHO_AM_I value =====
    static constexpr uint8_t kWhoAmI = 0x49;          // LSM303D device ID

    // ===== Register map (subset used) =====
    // Accel control
    static constexpr uint8_t REG_WHO_AM_I   = 0x0F;   // Device ID register
    static constexpr uint8_t REG_CTRL1      = 0x20;   // Accel ODR, axis enables
    static constexpr uint8_t REG_CTRL2      = 0x21;   // Accel full-scale, filter
    static constexpr uint8_t REG_CTRL5      = 0x24;   // Temp enable, mag res/ODR, BDU
    static constexpr uint8_t REG_CTRL6      = 0x25;   // Mag full-scale
    static constexpr uint8_t REG_CTRL7      = 0x26;   // Mag power mode (continuous)
    // Accel output (auto-inc capable)
    static constexpr uint8_t REG_OUT_X_L_A  = 0x28;   // Accel X LSB
    static constexpr uint8_t REG_OUT_X_H_A  = 0x29;   // Accel X MSB
    static constexpr uint8_t REG_OUT_Y_L_A  = 0x2A;   // Accel Y LSB
    static constexpr uint8_t REG_OUT_Y_H_A  = 0x2B;   // Accel Y MSB
    static constexpr uint8_t REG_OUT_Z_L_A  = 0x2C;   // Accel Z LSB
    static constexpr uint8_t REG_OUT_Z_H_A  = 0x2D;   // Accel Z MSB
    // Mag output (auto-inc capable)
    static constexpr uint8_t REG_OUT_X_L_M  = 0x08;   // Mag X LSB
    static constexpr uint8_t REG_OUT_X_H_M  = 0x09;   // Mag X MSB
    static constexpr uint8_t REG_OUT_Y_L_M  = 0x0A;   // Mag Y LSB
    static constexpr uint8_t REG_OUT_Y_H_M  = 0x0B;   // Mag Y MSB
    static constexpr uint8_t REG_OUT_Z_L_M  = 0x0C;   // Mag Z LSB
    static constexpr uint8_t REG_OUT_Z_H_M  = 0x0D;   // Mag Z MSB

    // ===== Sensitivity constants =====
    static constexpr float kAccel_g_per_LSB = 0.000732f; // ≈0.732 mg/LSB @ ±16 g (left-justified → effective)
    static constexpr float kMag_uT_per_LSB  = 0.016f;    // 0.016 µT/LSB @ ±4 gauss

    /**
     * @brief Construct the driver with abstract bus and optional addresses.
     * @param bus          Reference to shared Bus.
     * @param addrPrimary  Preferred I²C address to try first.
     * @param addrAlt      Alternate I²C address to try next.
     */
    explicit LSM303D(Bus& bus,
                     uint8_t addrPrimary = kAddrPrimary,
                     uint8_t addrAlt     = kAddrAlternate)
    : bus_(bus)                 // Save bus reference
    , addrPrimary_(addrPrimary) // Save primary address
    , addrAlt_(addrAlt)         // Save alternate address
    , addrActive_(0)            // Active address decided in begin()
    {}

    /**
     * @brief Initialize accelerometer and magnetometer.
     * @return true if device is detected and configured.
     */
    bool begin();

    /**
     * @brief Read linear acceleration in g.
     * @param ax_g Accel X (g)
     * @param ay_g Accel Y (g)
     * @param az_g Accel Z (g)
     * @return true on success.
     */
    bool readAccel(float& ax_g, float& ay_g, float& az_g);

    /**
     * @brief Read magnetic field in microtesla.
     * @param mx_uT Mag X (µT)
     * @param my_uT Mag Y (µT)
     * @param mz_uT Mag Z (µT)
     * @return true on success.
     */
    bool readMag(float& mx_uT, float& my_uT, float& mz_uT);

    /**
     * @brief Nominal accel and mag data rates (for info/logging).
     */
    float getAccelRateHz() const { return 800.0f; }   // Configured in CTRL1
    float getMagRateHz()   const { return 25.0f;  }   // Typical mag ODR

    /**
     * @brief Return the active I²C address used after begin().
     */
    uint8_t activeAddress() const { return addrActive_; }

private:
    // ===== Helpers =====
    bool   writeReg(uint8_t reg, uint8_t val);        // Write a register
    bool   readRegs(uint8_t startReg, uint8_t* buf, size_t len); // Burst read with auto-inc
    bool   readByte(uint8_t reg, uint8_t& val);       // Read a single byte
    bool   probeAndSelectAddress();                   // WHO_AM_I on primary then alternate

    // ===== State =====
    Bus&    bus_;             // Shared bus
    uint8_t addrPrimary_;     // Preferred address
    uint8_t addrAlt_;         // Alternate address
    uint8_t addrActive_;      // Resolved active address
};
