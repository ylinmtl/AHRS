#include "SensorHub.h"

/**
 * @brief Store the bus reference. Device instances are created in begin() so
 *        they can honor Config::Enable and we fail fast if probing fails.
 */
SensorHub::SensorHub(Bus& bus) : bus_(bus) {}

/**
 * @brief Instantiate/probe enabled devices, select sources, and establish hub rate.
 */
bool SensorHub::begin() {
    // Construct enabled drivers (probing happens in each begin()).
    if (Config::Enable::MPU9150) mpu9150_ = std::make_unique<MPU9150>(bus_, 0x68, 0x0C); // MPU @0x68, AK8975 @0x0C
    if (Config::Enable::L3GD20H) l3gd20h_ = std::make_unique<L3GD20H>(bus_);
    if (Config::Enable::LSM303D) lsm303d_ = std::make_unique<LSM303D>(bus_);
    if (Config::Enable::LPS25H)  lps25h_  = std::make_unique<LPS25H >(bus_);

    // Probe/init each device; ignore absent hardware (driver begin() returns false).
    if (mpu9150_ && !mpu9150_->begin()) mpu9150_.reset();
    if (l3gd20h_ && !l3gd20h_->begin()) l3gd20h_.reset();
    if (lsm303d_ && !lsm303d_->begin()) lsm303d_.reset();
    if (lps25h_  && !lps25h_ ->begin()) lps25h_ .reset();

    // Choose a source for each quantity based on presence + preferences.
    chooseGyroSource();
    chooseAccelSource();
    chooseMagSource();
    chooseBaroSource();
    chooseTempSource();

    // Mirror selected gyro's rate for hub pacing.
    if (gyroSrc_ == GyroSrc::MPU9150 && mpu9150_)      sampleRateHz_ = mpu9150_->sampleRateHz();
    else if (gyroSrc_ == GyroSrc::L3GD20H && l3gd20h_) sampleRateHz_ = l3gd20h_->getSampleRateHz();
    else                                               sampleRateHz_ = 800.0f;

    // Require a valid gyro source for meaningful operation.
    return (gyroSrc_ != GyroSrc::None);
}

/**
 * @brief Read the selected sources and fan out into a single scaled struct.
 *        Ensures MPU9150::read() is called at most once per loop even if it
 *        supplies multiple quantities (gyro, accel, mag, temp).
 */
bool SensorHub::read(SensorScaled& out) {
    bool ok = true;              // Accumulates overall success
    bool mpuRead = false;        // Tracks whether we already called mpu9150_->read()

    // Initialize "optional" channels to NAN so missing sensors are explicit.
    out.pressure_Pa   = NAN;
    out.temperature_C = NAN;

    // ----- Any dependence on MPU9150? Call read() once then scatter results. -----
    const bool needMPU =
        (gyroSrc_ == GyroSrc::MPU9150) ||
        (accelSrc_ == AccelSrc::MPU9150) ||
        (magSrc_  == MagSrc::MPU9150)  ||
        (tempSrc_ == TempSrc::MPU9150);

    if (needMPU) {
        if (mpu9150_) {
            ok &= mpu9150_->read();
            mpuRead = true;

            if (gyroSrc_ == GyroSrc::MPU9150) {
                out.gx_dps = mpu9150_->gx_dps();
                out.gy_dps = mpu9150_->gy_dps();
                out.gz_dps = mpu9150_->gz_dps();
            }
            if (accelSrc_ == AccelSrc::MPU9150) {
                out.ax_g = mpu9150_->ax_g();
                out.ay_g = mpu9150_->ay_g();
                out.az_g = mpu9150_->az_g();
            }
            if (magSrc_ == MagSrc::MPU9150) {
                out.mx_uT = mpu9150_->mx_uT();
                out.my_uT = mpu9150_->my_uT();
                out.mz_uT = mpu9150_->mz_uT();
            }
            if (tempSrc_ == TempSrc::MPU9150) {
                out.temperature_C = mpu9150_->temp_C(); // Die temperature (fallback if no baro)
            }
        } else {
            ok = false; // Selected MPU9150 but it's not available
        }
    }

    // ----- External gyro (L3GD20H) -----
    if (gyroSrc_ == GyroSrc::L3GD20H) {
        if (l3gd20h_) {
            float x,y,z;
            ok &= l3gd20h_->read(x,y,z);
            out.gx_dps = x; out.gy_dps = y; out.gz_dps = z;
        } else {
            ok = false;
        }
    }

    // ----- External accel/mag (LSM303D) -----
    if (accelSrc_ == AccelSrc::LSM303D) {
        if (lsm303d_) {
            float x,y,z;
            ok &= lsm303d_->readAccel(x,y,z);
            out.ax_g = x; out.ay_g = y; out.az_g = z;
        } else {
            ok = false;
        }
    }
    if (magSrc_ == MagSrc::LSM303D) {
        if (lsm303d_) {
            float x,y,z;
            ok &= lsm303d_->readMag(x,y,z);
            out.mx_uT = x; out.my_uT = y; out.mz_uT = z;
        } else {
            ok = false;
        }
    }

    // ----- Barometer + ambient temperature (LPS25H) -----
    if (baroSrc_ == BaroSrc::LPS25H || tempSrc_ == TempSrc::LPS25H) {
        if (lps25h_) {
            float p, t;
            ok &= lps25h_->read(p, t);
            if (baroSrc_ == BaroSrc::LPS25H) out.pressure_Pa = p;
            if (tempSrc_ == TempSrc::LPS25H) out.temperature_C = t;
        } else {
            ok = false;
        }
    }

    return ok;
}

// ----- source selection -------------------------------------------------------

void SensorHub::chooseGyroSource() {
    if (Config::Select::PreferExternalGyro && l3gd20h_)       { gyroSrc_ = GyroSrc::L3GD20H; return; }
    if (mpu9150_)                                             { gyroSrc_ = GyroSrc::MPU9150; return; }
    if (l3gd20h_)                                             { gyroSrc_ = GyroSrc::L3GD20H; return; }
    gyroSrc_ = GyroSrc::None;
}

void SensorHub::chooseAccelSource() {
    if (Config::Select::PreferExternalAccel && lsm303d_)      { accelSrc_ = AccelSrc::LSM303D; return; }
    if (mpu9150_)                                             { accelSrc_ = AccelSrc::MPU9150; return; }
    if (lsm303d_)                                             { accelSrc_ = AccelSrc::LSM303D; return; }
    accelSrc_ = AccelSrc::None;
}

void SensorHub::chooseMagSource() {
    if (Config::Select::PreferExternalMag && lsm303d_)        { magSrc_ = MagSrc::LSM303D; return; }
    if (mpu9150_)                                             { magSrc_ = MagSrc::MPU9150; return; }
    if (lsm303d_)                                             { magSrc_ = MagSrc::LSM303D; return; }
    magSrc_ = MagSrc::None;
}

void SensorHub::chooseBaroSource() {
    if (lps25h_)                                              { baroSrc_ = BaroSrc::LPS25H; return; }
    baroSrc_ = BaroSrc::None;
}

void SensorHub::chooseTempSource() {
    if (lps25h_)                                              { tempSrc_ = TempSrc::LPS25H; return; } // Ambient preferred
    if (mpu9150_)                                             { tempSrc_ = TempSrc::MPU9150; return; } // Fallback: IMU die
    tempSrc_ = TempSrc::None;
}
