#pragma once
#include <stdint.h>

// Global config & build-time sensor selection.
namespace Config {

// I2C speed
static constexpr unsigned I2C_CLOCK_HZ = 400000; // 400 kHz

// Enable flags
namespace Enable {
    static constexpr bool MPU9150 = 1; // set true to use MPU9150 combo IMU

    // ST discrete sensors
    static constexpr bool L3GD20H = 0;  // Gyro
    static constexpr bool LSM303D = 0;  // Accel + Mag
    static constexpr bool LPS25H  = 0;  // Baro + Temp
}

// I2C addresses
namespace Addr {
    // InvenSense
    static constexpr uint8_t MPU9150 = 0x68; // AD0=0
    static constexpr uint8_t AK8975  = 0x0C; // behind MPU master

    // ST
    static constexpr uint8_t L3GD20H = 0x6B; // SA0=1
    static constexpr uint8_t LSM303D = 0x1D; // SA0=1
    static constexpr uint8_t LPS25H  = 0x5D; // SA0=1
}

// Source preferences when multiple are enabled
namespace Select {
    static constexpr bool PreferExternalGyro  = true;
    static constexpr bool PreferExternalAccel = true;
    static constexpr bool PreferExternalMag   = true;
}

} // namespace Config
