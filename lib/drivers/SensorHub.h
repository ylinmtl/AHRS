#pragma once
#include <Arduino.h>
#include <memory>
#include <math.h>
#include "BusI2C.h"
#include "Config.h"
#include "dev/MPU9150.h"
#include "dev/L3GD20H.h"
#include "dev/LSM303D.h"
#include "dev/LPS25H.h"

/**
 * @struct SensorScaled
 * @brief One unified, scaled sensor sample.
 *
 * Units:
 *  - Gyro: degrees/second
 *  - Accel: g
 *  - Mag: microtesla
 *  - Pressure: pascal
 *  - Temperature: degrees Celsius
 */
struct SensorScaled {
    float ax_g=0, ay_g=0, az_g=0;
    float gx_dps=0, gy_dps=0, gz_dps=0;
    float mx_uT=0, my_uT=0, mz_uT=0;
    float pressure_Pa=NAN;
    float temperature_C=NAN;
};

/**
 * @class SensorHub
 * @brief Orchestrates device drivers and exposes a single read() with scaled units.
 *
 * - Probes enabled drivers during begin()
 * - Picks a source for each quantity (gyro/accel/mag/baro/temp)
 * - Exposes a unified sample rate (mirrors chosen gyro rate)
 */
class SensorHub {
public:
    /**
     * @brief Construct with a bus reference.
     * @param bus I²C bus instance shared among drivers.
     */
    explicit SensorHub(BusI2C& bus);

    /**
     * @brief Probe and initialize all enabled sensors.
     * @return true if a gyro source is active and a sample rate was established.
     */
    bool begin();

    /**
     * @brief Read one sensor sample in scaled units.
     * @param out Destination sample struct.
     * @return true if all selected sources were read successfully.
     */
    bool read(SensorScaled& out);

    /**
     * @brief Hub sample rate in Hz (mirrors selected gyro source).
     */
    float getSampleRateHz() const { return sampleRateHz_; }

private:
    BusI2C& bus_;
    std::unique_ptr<MPU9150> mpu9150_;
    std::unique_ptr<L3GD20H> l3gd20h_;
    std::unique_ptr<LSM303D> lsm303d_;
    std::unique_ptr<LPS25H>  lps25h_;

    /// Selected sources per quantity.
    enum class GyroSrc { None, MPU9150, L3GD20H };
    enum class AccelSrc{ None, MPU9150, LSM303D };
    enum class MagSrc  { None, MPU9150, LSM303D };
    enum class BaroSrc { None, LPS25H };
    enum class TempSrc { None, LPS25H, MPU9150 }; // <— NEW: prefer LPS25H ambient; fallback to IMU die

    GyroSrc  gyroSrc_  = GyroSrc::None;
    AccelSrc accelSrc_ = AccelSrc::None;
    MagSrc   magSrc_   = MagSrc::None;
    BaroSrc  baroSrc_  = BaroSrc::None;
    TempSrc  tempSrc_  = TempSrc::None;

    float sampleRateHz_ = 800.0f; // default

    /** @brief Decide which gyro to use given availability and Config::Select. */
    void chooseGyroSource();
    /** @brief Decide which accelerometer to use given availability and Config::Select. */
    void chooseAccelSource();
    /** @brief Decide which magnetometer to use given availability and Config::Select. */
    void chooseMagSource();
    /** @brief Decide which barometer to use (if any). */
    void chooseBaroSource();
    /** @brief Decide temperature source (prefer baro, else IMU die). */
    void chooseTempSource();
};
