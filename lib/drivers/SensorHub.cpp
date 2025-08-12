#include "SensorHub.h"

/**
 * @brief Store the shared bus and defer device construction to begin().
 */
SensorHub::SensorHub(BusI2C& bus) : bus_(bus) {}

/**
 * @brief Probe and initialize enabled devices, then choose data sources.
 */
bool SensorHub::begin() {
    // Instantiate per Config (kept modular for future SPI/mocks)
    if (Config::Enable::MPU9150) mpu9150_.reset(new MPU9150(bus_, Config::Addr::MPU9150, Config::Addr::AK8975));
    if (Config::Enable::L3GD20H) l3gd20h_.reset(new L3GD20H(bus_, Config::Addr::L3GD20H));
    if (Config::Enable::LSM303D) lsm303d_.reset(new LSM303D(bus_, Config::Addr::LSM303D));
    if (Config::Enable::LPS25H)  lps25h_.reset (new LPS25H (bus_, Config::Addr::LPS25H));

    bool any = false;
    if (mpu9150_) any |= mpu9150_->begin();
    if (l3gd20h_) any |= l3gd20h_->begin();
    if (lsm303d_) any |= lsm303d_->begin();
    if (lps25h_)  any |= lps25h_->begin();

    // Choose sources
    chooseGyroSource();
    chooseAccelSource();
    chooseMagSource();
    chooseBaroSource();
    chooseTempSource();

    // Establish hub sample rate from the selected gyro
    if (gyroSrc_ == GyroSrc::MPU9150 && mpu9150_) {
        sampleRateHz_ = mpu9150_->sampleRateHz();
    } else if (gyroSrc_ == GyroSrc::L3GD20H && l3gd20h_) {
        sampleRateHz_ = l3gd20h_->getSampleRateHz();
    } else {
        sampleRateHz_ = 800.0f; // sane default
    }

    // Require a valid gyro source
    return (gyroSrc_ != GyroSrc::None);
}

/**
 * @brief Read selected sources once per loop and fan out to the scaled struct.
 *
 * Notes:
 * - We call MPU9150::read() at most once per loop even if multiple quantities
 *   (gyro/accel/mag/temp) come from it.
 * - If LPS25H is present, its temperature is treated as ambient and preferred.
 *   Otherwise we fall back to the IMU die temperature from MPU9150.
 */
bool SensorHub::read(SensorScaled& out) {
    bool ok = true;
    bool mpuRead = false;

    // Zero/keep prior values where appropriate. Pressure/Temp default to NaN to make "not available"
    // explicit. Change to 0 if you prefer zeros.
    out.pressure_Pa   = NAN;
    out.temperature_C = NAN;

    // ---- Gyro & Accel (and IMU temp) from MPU9150 if selected ----
    if ((gyroSrc_ == GyroSrc::MPU9150) || (accelSrc_ == AccelSrc::MPU9150) || (magSrc_ == MagSrc::MPU9150) || (tempSrc_ == TempSrc::MPU9150)) {
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
                out.temperature_C = mpu9150_->temp_C(); // <— FALLBACK temp when no baro
            }
        } else {
            ok = false;
        }
    }

    // ---- Gyro from L3GD20H ----
    if (gyroSrc_ == GyroSrc::L3GD20H) {
        if (l3gd20h_) {
            float gx, gy, gz;
            ok &= l3gd20h_->read(gx, gy, gz);
            out.gx_dps = gx; out.gy_dps = gy; out.gz_dps = gz;
        } else {
            ok = false;
        }
    }

    // ---- Accel + Mag from LSM303D (read only if selected) ----
    if (accelSrc_ == AccelSrc::LSM303D) {
        if (lsm303d_) {
            float ax, ay, az;
            ok &= lsm303d_->readAccel(ax, ay, az);
            out.ax_g = ax; out.ay_g = ay; out.az_g = az;
        } else {
            ok = false;
        }
    }
    if (magSrc_ == MagSrc::LSM303D) {
        if (lsm303d_) {
            float mx, my, mz;
            ok &= lsm303d_->readMag(mx, my, mz);
            out.mx_uT = mx; out.my_uT = my; out.mz_uT = mz;
        } else {
            ok = false;
        }
    }

    // ---- Baro + Temp from LPS25H (Temp preferred when available) ----
    if (baroSrc_ == BaroSrc::LPS25H || tempSrc_ == TempSrc::LPS25H) {
        if (lps25h_) {
            float p, tc;
            ok &= lps25h_->read(p, tc);
            if (baroSrc_ == BaroSrc::LPS25H) out.pressure_Pa = p;
            if (tempSrc_ == TempSrc::LPS25H) out.temperature_C = tc; // <— AMBIENT temp
        } else {
            ok = false;
        }
    }

    return ok;
}

/** @brief Choose gyro source per availability and preference. */
void SensorHub::chooseGyroSource() {
    // Prefer external gyro if requested and available
    if (Config::Select::PreferExternalGyro && l3gd20h_) {
        gyroSrc_ = GyroSrc::L3GD20H;
    } else if (mpu9150_) {
        gyroSrc_ = GyroSrc::MPU9150;
    } else if (l3gd20h_) {
        gyroSrc_ = GyroSrc::L3GD20H;
    } else {
        gyroSrc_ = GyroSrc::None;
    }
}

/** @brief Choose accelerometer source per availability and preference. */
void SensorHub::chooseAccelSource() {
    if (Config::Select::PreferExternalAccel && lsm303d_) {
        accelSrc_ = AccelSrc::LSM303D;
    } else if (mpu9150_) {
        accelSrc_ = AccelSrc::MPU9150;
    } else if (lsm303d_) {
        accelSrc_ = AccelSrc::LSM303D;
    } else {
        accelSrc_ = AccelSrc::None;
    }
}

/** @brief Choose magnetometer source per availability and preference. */
void SensorHub::chooseMagSource() {
    if (Config::Select::PreferExternalMag && lsm303d_) {
        magSrc_ = MagSrc::LSM303D;
    } else if (mpu9150_) {
        magSrc_ = MagSrc::MPU9150;
    } else if (lsm303d_) {
        magSrc_ = MagSrc::LSM303D;
    } else {
        magSrc_ = MagSrc::None;
    }
}

/** @brief Choose barometer source (only LPS25H currently). */
void SensorHub::chooseBaroSource() {
    if (lps25h_) baroSrc_ = BaroSrc::LPS25H;
    else         baroSrc_ = BaroSrc::None;
}

/**
 * @brief Choose temperature source: prefer ambient from baro if present, else IMU die temp.
 *
 * Rationale:
 * - LPS25H provides ambient; best for environment/logging.
 * - MPU9150 die temp is available even without a baro; useful fallback.
 */
void SensorHub::chooseTempSource() {
    if (lps25h_)      tempSrc_ = TempSrc::LPS25H;
    else if (mpu9150_) tempSrc_ = TempSrc::MPU9150;
    else               tempSrc_ = TempSrc::None;
}
