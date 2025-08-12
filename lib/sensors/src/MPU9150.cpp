#include "MPU9150.h"

// ---- Bus helper ----
bool MPU9150::readBlock(uint8_t dev, uint8_t reg, uint8_t* buf, size_t len) {
    return _bus.readRegisters(dev, reg, buf, len);
}

/** Enable internal I2C master ~400 kHz, small slave delays; bypass OFF. */
bool MPU9150::enableI2CMaster() {
    if (!_bus.writeRegister(_mpu, REG_INT_PIN_CFG, 0x00)) return false; // bypass off

    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, 0x00)) return false;
    delay(2);
    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, USER_I2C_MST_RST)) return false;
    delay(2);
    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, USER_I2C_MST_EN)) return false;
    delay(2);

    if (!_bus.writeRegister(_mpu, REG_I2C_MST_CTRL, MST_CLK_400K)) return false;
    _bus.writeRegister(_mpu, REG_I2C_MST_DELAY, 0x03); // tiny inter-slave delays
    return true;
}

/** Probe AK8975 WIA (0x48) via SLV0 into EXT_SENS_DATA_00. */
bool MPU9150::probeMagWIA() {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_ADDR, (uint8_t)(0x80 | _mag))) return false; // read
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_REG,  AK_REG_WIA)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_CTRL, 0x81)) return false; // EN, LEN=1
    delay(8);
    uint8_t id = 0;
    if (!readBlock(_mpu, REG_EXT_SENS_DATA_00, &id, 1)) return false;
    return (id == 0x48);
}

/** Stream 8 bytes from ST1..ST2 into EXT_SENS_DATA_00..07. */
bool MPU9150::setupMagStream() {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_ADDR, (uint8_t)(0x80 | _mag))) return false; // read
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_REG,  AK_REG_ST1)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_CTRL, 0x88)) return false; // EN, LEN=8
    return true;
}

/** Blocking SLV4 write (poll DONE, check NACK). */
bool MPU9150::slv4WriteBlocking(uint8_t dev, uint8_t reg, uint8_t data, uint32_t timeout_us) {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_ADDR, dev)) return false; // write op (bit7=0)
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_REG,  reg)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_DO,   data)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_CTRL, 0x80)) return false; // EN

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < timeout_us) {
        uint8_t st=0;
        if (!readBlock(_mpu, REG_I2C_MST_STATUS, &st, 1)) return false;
        if (st & 0x40) {              // SLV4_DONE
            if (st & 0x10) return false; // NACK
            return true;
        }
    }
    return false; // timeout
}

/** Blocking SLV4 read (ADDR with read bit, poll DONE, read DI). */
bool MPU9150::slv4ReadBlocking(uint8_t dev, uint8_t reg, uint8_t& data, uint32_t timeout_us) {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_ADDR, (uint8_t)(0x80 | dev))) return false; // read op
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_REG,  reg)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_CTRL, 0x80)) return false; // EN

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < timeout_us) {
        uint8_t st=0;
        if (!readBlock(_mpu, REG_I2C_MST_STATUS, &st, 1)) return false;
        if (st & 0x40) {              // SLV4_DONE
            if (st & 0x10) return false; // NACK
            break;
        }
    }
    // Read the received byte
    if (!readBlock(_mpu, REG_I2C_SLV4_DI, &data, 1)) return false;
    return true;
}

/** Read AK8975 ASA (fuse ROM) and compute per-axis adjustment multipliers. */
bool MPU9150::readASA() {
    // Enter power-down then fuse ROM access mode
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_FUSEROM,   2000);

    uint8_t asax=128, asay=128, asaz=128; // default -> multiplier 1.0
    slv4ReadBlocking(_mag, AK_REG_ASAX, asax, 2000);
    slv4ReadBlocking(_mag, AK_REG_ASAY, asay, 2000);
    slv4ReadBlocking(_mag, AK_REG_ASAZ, asaz, 2000);

    // Leave fuse mode
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);

    // Compute multipliers per datasheet: Hadj = H * ( ((ASA-128)*0.5/128) + 1 )
    auto adj = [](uint8_t ASA)->float {
        return ( ( (float)ASA - 128.0f ) * 0.5f / 128.0f ) + 1.0f;
    };
    _asaAdjX = adj(asax);
    _asaAdjY = adj(asay);
    _asaAdjZ = adj(asaz);
    return true;
}

/** Trigger AK8975 single-shot every ~12 ms (PD -> SINGLE). */
void MPU9150::triggerMagIfDue(uint32_t now_us) {
    const uint32_t period_us = 12000; // ~83 Hz nominal, ~80 Hz at 800 Hz loop
    if (!_magPresent || (uint32_t)(now_us - _lastMagTrigUs) < period_us) return;
    _lastMagTrigUs = now_us;

    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_SINGLE,    2000);
}

/** Compute scaled accel (G), gyro (dps), temp (C) from latest raw. */
void MPU9150::computeScaledIMU() {
    _ax_g = (float)_ax / ACC_LSB_PER_G;
    _ay_g = (float)_ay / ACC_LSB_PER_G;
    _az_g = (float)_az / ACC_LSB_PER_G;

    _gx_dps = (float)_gx / GYRO_LSB_PER_DPS;
    _gy_dps = (float)_gy / GYRO_LSB_PER_DPS;
    _gz_dps = (float)_gz / GYRO_LSB_PER_DPS;

    _temp_C = ((float)_temp) / 340.0f + 36.53f;
}

/** Apply ASA then convert to microtesla. */
void MPU9150::computeScaledMag(int16_t mx, int16_t my, int16_t mz) {
    // ASA-adjusted counts
    const float hx = (float)mx * _asaAdjX;
    const float hy = (float)my * _asaAdjY;
    const float hz = (float)mz * _asaAdjZ;

    // Convert to microtesla (typical 0.3 uT/LSB per AK8975 datasheet).
    _mx_uT = hx * MAG_uT_PER_LSB;
    _my_uT = hy * MAG_uT_PER_LSB;
    _mz_uT = hz * MAG_uT_PER_LSB;
}

/** Initialize IMU + master + ASA + mag stream. */
bool MPU9150::begin() {
    // Wake & ranges (raw -> scaled in driver)
    if (!_bus.writeRegister(_mpu, REG_PWR_MGMT_1, 0x01)) return false;
    if (!_bus.writeRegister(_mpu, REG_GYRO_CONFIG,  (3 << 3))) return false; // ±2000 dps
    if (!_bus.writeRegister(_mpu, REG_ACCEL_CONFIG, (3 << 3))) return false; // ±16 g

    if (!enableI2CMaster()) return false;

    _magPresent = probeMagWIA();
    if (_magPresent) {
        readASA();              // compute _asaAdj*
        setupMagStream();       // stream ST1..ST2
    }

    _lastMagTrigUs = micros();
    return true; // IMU usable even if mag missing
}

/** Read IMU raw, compute scaled; update mag if DRDY, compute scaled. */
bool MPU9150::read() {
    // Accel(6), Temp(2), Gyro(6)
    uint8_t buf[14];
    if (!readBlock(_mpu, REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

    _ax   = toInt16(buf[0],  buf[1]);
    _ay   = toInt16(buf[2],  buf[3]);
    _az   = toInt16(buf[4],  buf[5]);
    _temp = toInt16(buf[6],  buf[7]);
    _gx   = toInt16(buf[8],  buf[9]);
    _gy   = toInt16(buf[10], buf[11]);
    _gz   = toInt16(buf[12], buf[13]);

    computeScaledIMU();

    // Magnetometer
    const uint32_t now = micros();
    triggerMagIfDue(now);

    if (_magPresent) {
        uint8_t m[8] = {0};
        if (readBlock(_mpu, REG_EXT_SENS_DATA_00, m, sizeof(m))) {
            if (m[0] & 0x01) { // ST1.DRDY
                _mx = toInt16(m[2], m[1]);
                _my = toInt16(m[4], m[3]);
                _mz = toInt16(m[6], m[5]);
                computeScaledMag(_mx, _my, _mz);
            }
        }
    } else {
        _mx = _my = _mz = 0;
        _mx_uT = _my_uT = _mz_uT = 0.0f;
    }

    return true;
}
