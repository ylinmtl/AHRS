#include "SensorHub.h"

SensorHub::SensorHub(BusI2C& bus) : bus_(bus) {
    if (Config::Enable::MPU9150) mpu9150_.reset(new MPU9150(bus_, Config::Addr::MPU9150, Config::Addr::AK8975));
    if (Config::Enable::L3GD20H) l3gd20h_.reset(new L3GD20H(bus_, Config::Addr::L3GD20H));
    if (Config::Enable::LSM303D) lsm303d_.reset(new LSM303D(bus_, Config::Addr::LSM303D));
    if (Config::Enable::LPS25H)  lps25h_.reset (new LPS25H (bus_, Config::Addr::LPS25H ));
}

bool SensorHub::begin() {
    bool anyGyro=false;
    if (mpu9150_) { if (mpu9150_->begin()) anyGyro = true; }
    if (l3gd20h_) { if (l3gd20h_->begin()) anyGyro = true; }
    if (lsm303d_) { (void)lsm303d_->begin(); }
    if (lps25h_)  { (void)lps25h_->begin(); }

    chooseGyroSource();
    chooseAccelSource();
    chooseMagSource();
    chooseBaroSource();

    switch (gyroSrc_) {
        case GyroSrc::MPU9150: sampleRateHz_ = mpu9150_ ? mpu9150_->sampleRateHz() : 0.0f; break;
        case GyroSrc::L3GD20H: sampleRateHz_ = l3gd20h_ ? l3gd20h_->getSampleRateHz() : 0.0f; break;
        default: sampleRateHz_ = 0.0f; break;
    }
    return anyGyro && sampleRateHz_ > 0.0f;
}

bool SensorHub::read(SensorScaled& out) {
    // Gyro (primary)
    switch (gyroSrc_) {
        case GyroSrc::MPU9150:
            if (!mpu9150_->read()) return false;
            out.gx_dps = mpu9150_->gx_dps();
            out.gy_dps = mpu9150_->gy_dps();
            out.gz_dps = mpu9150_->gz_dps();
            break;
        case GyroSrc::L3GD20H:
            if (!l3gd20h_->read(out.gx_dps, out.gy_dps, out.gz_dps)) return false;
            break;
        default:
            return false;
    }

    // Accel
    switch (accelSrc_) {
        case AccelSrc::MPU9150:
            out.ax_g = mpu9150_->ax_g();
            out.ay_g = mpu9150_->ay_g();
            out.az_g = mpu9150_->az_g();
            break;
        case AccelSrc::LSM303D:
            (void)lsm303d_->readAccel(out.ax_g, out.ay_g, out.az_g);
            break;
        default: break;
    }

    // Mag
    switch (magSrc_) {
        case MagSrc::MPU9150:
            out.mx_uT = mpu9150_->mx_uT();
            out.my_uT = mpu9150_->my_uT();
            out.mz_uT = mpu9150_->mz_uT();
            break;
        case MagSrc::LSM303D:
            (void)lsm303d_->readMag(out.mx_uT, out.my_uT, out.mz_uT);
            break;
        default: break;
    }

    // Baro/Temp
    switch (baroSrc_) {
        case BaroSrc::LPS25H:
            (void)lps25h_->read(out.pressure_Pa, out.temperature_C);
            break;
        default: break;
    }

    return true;
}

void SensorHub::chooseGyroSource() {
    const bool haveMPU = (mpu9150_ != nullptr);
    const bool haveL3G = (l3gd20h_ != nullptr);
    if (haveMPU && haveL3G) {
        gyroSrc_ = (Config::Select::PreferExternalGyro ? GyroSrc::L3GD20H : GyroSrc::MPU9150);
    } else if (haveMPU) {
        gyroSrc_ = GyroSrc::MPU9150;
    } else if (haveL3G) {
        gyroSrc_ = GyroSrc::L3GD20H;
    } else {
        gyroSrc_ = GyroSrc::None;
    }
}

void SensorHub::chooseAccelSource() {
    const bool haveMPU = (mpu9150_ != nullptr);
    const bool haveLSM = (lsm303d_ != nullptr);
    if (haveMPU && haveLSM) {
        accelSrc_ = (Config::Select::PreferExternalAccel ? AccelSrc::LSM303D : AccelSrc::MPU9150);
    } else if (haveMPU) {
        accelSrc_ = AccelSrc::MPU9150;
    } else if (haveLSM) {
        accelSrc_ = AccelSrc::LSM303D;
    } else {
        accelSrc_ = AccelSrc::None;
    }
}

void SensorHub::chooseMagSource() {
    const bool haveMPU = (mpu9150_ != nullptr);
    const bool haveLSM = (lsm303d_ != nullptr);
    if (haveMPU && haveLSM) {
        magSrc_ = (Config::Select::PreferExternalMag ? MagSrc::LSM303D : MagSrc::MPU9150);
    } else if (haveMPU) {
        magSrc_ = MagSrc::MPU9150;
    } else if (haveLSM) {
        magSrc_ = MagSrc::LSM303D;
    } else {
        magSrc_ = MagSrc::None;
    }
}

void SensorHub::chooseBaroSource() {
    baroSrc_ = (lps25h_ ? BaroSrc::LPS25H : BaroSrc::None);
}
