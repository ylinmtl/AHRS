#pragma once
#include <Arduino.h>
#include <memory>
#include "Bus.h"            // Bus abstraction (I2C in current builds)
#include "Config.h"

// Device drivers (each uses Bus& and probes its own address)
#include "dev/MPU9150.h"
#include "dev/L3GD20H.h"
#include "dev/LSM303D.h"
#include "dev/LPS25H.h"

/**
 * @struct SensorScaled
 * @brief One unified, scaled sample across all sensors.
 *
 * Units:
 *  - Gyro: degrees/second (dps)
 *  - Accel: g
 *  - Mag: microtesla (uT)
 *  - Pressure: pascal (Pa)
 *  - Temperature: degrees Celsius (°C)
 */
struct SensorScaled {
    float ax_g=0, ay_g=0, az_g=0;
    float gx_dps=0, gy_dps=0, gz_dps=0;
    float mx_uT=0, my_uT=0, mz_uT=0;
    float pressure_Pa = NAN;
    float temperature_C = NAN;
};

/**
 * @class SensorHub
 * @brief Orchestrates device drivers and exposes a single read() of scaled units.
 *
 * Responsibilities:
 *  - Construct, probe, and initialize enabled drivers
 *  - Choose a source for each quantity (gyro/accel/mag/baro/temp)
 *  - Mirror the chosen gyro's sample rate as the hub rate (for fusion pacing)
 *  - Read each selected source exactly once per loop
 */
class SensorHub {
public:
    /**
     * @brief Construct with a shared bus (I2C or other Bus implementation).
     * @param bus Shared Bus used by all device drivers.
     */
    explicit SensorHub(Bus& bus);

    /**
     * @brief Probe and configure enabled devices, then select sources.
     * @return true if a valid gyro source was found and a hub rate was established.
     */
    bool begin();

    /**
     * @brief Read a complete scaled sample from the selected sources.
     * @param out Destination struct for scaled values.
     * @return true if all selected sources were read successfully.
     */
    bool read(SensorScaled& out);

    /**
     * @brief Hub sample rate in Hz (mirrors selected gyro).
     */
    float getSampleRateHz() const { return sampleRateHz_; }

private:
    // ----- bus and owned drivers -----
    Bus& bus_;
    std::unique_ptr<MPU9150> mpu9150_;
    std::unique_ptr<L3GD20H> l3gd20h_;
    std::unique_ptr<LSM303D> lsm303d_;
    std::unique_ptr<LPS25H>  lps25h_;

    // ----- selection enums -----
    enum class GyroSrc { None, MPU9150, L3GD20H };
    enum class AccelSrc{ None, MPU9150, LSM303D };
    enum class MagSrc  { None, MPU9150, LSM303D };
    enum class BaroSrc { None, LPS25H };
    enum class TempSrc { None, LPS25H, MPU9150 }; // Prefer ambient (baro), else IMU die

    GyroSrc  gyroSrc_  = GyroSrc::None;
    AccelSrc accelSrc_ = AccelSrc::None;
    MagSrc   magSrc_   = MagSrc::None;
    BaroSrc  baroSrc_  = BaroSrc::None;
    TempSrc  tempSrc_  = TempSrc::None;

    float sampleRateHz_ = 800.0f; // Default fallback if no gyro advertises a rate

    // ----- source selection helpers -----
    void chooseGyroSource();
    void chooseAccelSource();
    void chooseMagSource();
    void chooseBaroSource();
    void chooseTempSource();
};
