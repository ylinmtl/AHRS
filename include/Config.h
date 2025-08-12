#pragma once

// Global configuration for platform, bus, and sensors.
// Keep this small; extend as you add boards/sensors.

namespace Config {
    // Target sample rate for printing raw values (Hz)
    static constexpr unsigned SAMPLE_RATE_HZ = 800;

    // I2C bus parameters
    static constexpr unsigned I2C_CLOCK_HZ = 400000; // 400 kHz default

    // Sensor selection – for now, only MPU9150 is populated
    enum class ImuType : uint8_t { MPU9150 };
    static constexpr ImuType IMU = ImuType::MPU9150;

    // I2C addresses
    namespace Addr {
        static constexpr uint8_t MPU9150 = 0x68; // AD0=0
        static constexpr uint8_t AK8975  = 0x0C; // Mag behind bypass
    }
}
