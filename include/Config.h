#pragma once
#include <Arduino.h>

/**
 * @file Config.h
 * @brief Central build-time configuration for the AHRS sensor layer.
 *
 * Keep this file minimal. Devices probe their own I2C addresses (primary → alternate),
 * so we do NOT carry per-device address constants here anymore.
 */
namespace Config {

    /**
     * @brief I2C clock frequency in Hz for the shared bus.
     * Teensy/Arduino Wire typically supports 100 kHz, 400 kHz, 1 MHz (board-dependent).
     */
    static constexpr unsigned I2C_CLOCK_HZ = 400000U;

    /**
     * @brief Enable/disable drivers at build time.
     * At runtime, each enabled driver is probed (WHO_AM_I). Only present devices are used.
     */
    namespace Enable {
        static constexpr bool MPU9150 = 1;  // InvenSense IMU + internal AK8975 mag
        static constexpr bool L3GD20H = 0;  // ST gyro
        static constexpr bool LSM303D = 0;  // ST accel + mag
        static constexpr bool LPS25H  = 0;  // ST baro + temp
    }

    /**
     * @brief Source preferences when multiple devices can provide the same quantity.
     * Runtime selection also considers actual device presence.
     */
    namespace Select {
        static constexpr bool PreferExternalGyro  = false; // true -> choose L3GD20H over MPU9150 gyro when both are present
        static constexpr bool PreferExternalAccel = false; // true -> choose LSM303D over MPU9150 accel when both are present
        static constexpr bool PreferExternalMag   = false; // true -> choose LSM303D over MPU9150 mag when both are present
        // Temperature preference is implicit: prefer baro ambient (LPS25H) when present,
        // otherwise fall back to IMU die temp (MPU9150).
    }
}
