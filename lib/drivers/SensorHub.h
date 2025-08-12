#pragma once
#include <Arduino.h>
#include <memory>
#include "BusI2C.h"
#include "Config.h"
#include "dev/MPU9150.h"
#include "dev/L3GD20H.h"
#include "dev/LSM303D.h"
#include "dev/LPS25H.h"

struct SensorScaled {
    float ax_g=0, ay_g=0, az_g=0;
    float gx_dps=0, gy_dps=0, gz_dps=0;
    float mx_uT=0, my_uT=0, mz_uT=0;
    float pressure_Pa=0;
    float temperature_C=0;
};

class SensorHub {
public:
    explicit SensorHub(BusI2C& bus);
    bool begin();
    bool read(SensorScaled& out);
    float getSampleRateHz() const { return sampleRateHz_; }

private:
    BusI2C& bus_;
    std::unique_ptr<MPU9150> mpu9150_;
    std::unique_ptr<L3GD20H> l3gd20h_;
    std::unique_ptr<LSM303D> lsm303d_;
    std::unique_ptr<LPS25H>  lps25h_;

    enum class GyroSrc { None, MPU9150, L3GD20H };
    enum class AccelSrc{ None, MPU9150, LSM303D };
    enum class MagSrc  { None, MPU9150, LSM303D };
    enum class BaroSrc { None, LPS25H };

    GyroSrc  gyroSrc_  = GyroSrc::None;
    AccelSrc accelSrc_ = AccelSrc::None;
    MagSrc   magSrc_   = MagSrc::None;
    BaroSrc  baroSrc_  = BaroSrc::None;

    float sampleRateHz_ = 0.0f;

    void chooseGyroSource();
    void chooseAccelSource();
    void chooseMagSource();
    void chooseBaroSource();
};
