#include "MPU9150.h"

// ---- Bus helper: read a register block via Bus abstraction (MCU-agnostic) ----
bool MPU9150::readBlock(uint8_t dev, uint8_t reg, uint8_t* buf, size_t len) {
    return _bus.readRegisters(dev, reg, buf, len);
}

/* ===== configureSampleRateFromDriver ========================================
 * Decide the device ODR inside the driver, program DLPF + SMPLRT_DIV accordingly,
 * and store the actual resulting rate so the app/fusion can query sampleRateHz().
 *
 * Strategy:
 *  - Keep DLPF enabled (CONFIG.DLPF_CFG = 1 -> 184 Hz) so base rate = 1 kHz.
 *  - Compute SMPLRT_DIV = round(1000 / requested) - 1; clamp 0..255.
 *  - Record _gyroRateHzActual and _accelRateHzActual (accel follows gyro with DLPF on).
 */
bool MPU9150::configureSampleRateFromDriver() {
    uint16_t req = kRequestedGyroRateHz;
    if (req < 10)   req = 10;     // avoid pathological dividers
    if (req > 1000) req = 1000;   // accel max ODR with DLPF on

    // DLPF = 184 Hz -> base 1000 Hz
    if (!_bus.writeRegister(_mpu, REG_CONFIG, DLPF_CFG_184HZ)) return false;

    // SMPLRT_DIV to get closest to requested
    float div_f = (1000.0f / req) - 1.0f;
    int   div_i = (int)(div_f + 0.5f);
    if (div_i < 0)   div_i = 0;
    if (div_i > 255) div_i = 255;

    if (!_bus.writeRegister(_mpu, REG_SMPLRT_DIV, (uint8_t)div_i)) return false;

    _gyroRateHzActual  = (uint16_t)(1000 / (1 + div_i));
    _accelRateHzActual = _gyroRateHzActual;
    return true;
}

/* ===== enableI2CMaster =======================================================
 * Enable the MPU's internal I2C master @ ~400 kHz; keep bypass OFF.
 * This block owns the AK8975 link (no direct/bypass access).
 */
bool MPU9150::enableI2CMaster() {
    if (!_bus.writeRegister(_mpu, REG_INT_PIN_CFG, 0x00)) return false; // bypass off

    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, 0x00)) return false;
    delay(2);
    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, USER_I2C_MST_RST)) return false;
    delay(2);
    if (!_bus.writeRegister(_mpu, REG_USER_CTRL, USER_I2C_MST_EN))  return false;
    delay(2);

    if (!_bus.writeRegister(_mpu, REG_I2C_MST_CTRL, MST_CLK_400K))  return false;
    _bus.writeRegister(_mpu, REG_I2C_MST_DELAY, 0x03); // tiny inter-slave delays
    return true;
}

/* ===== probeMagWIA =========================================================== */
bool MPU9150::probeMagWIA() {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_ADDR, (uint8_t)(0x80 | _mag))) return false; // READ op
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_REG,  AK_REG_WIA))              return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_CTRL, 0x81))                    return false; // EN, LEN=1
    delay(8);
    uint8_t id=0;
    if (!readBlock(_mpu, REG_EXT_SENS_DATA_00, &id, 1)) return false;
    return (id == 0x48);
}

/* ===== setupMagStream ======================================================== */
bool MPU9150::setupMagStream() {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_ADDR, (uint8_t)(0x80 | _mag))) return false; // READ
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_REG,  AK_REG_ST1))              return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV0_CTRL, 0x88))                    return false; // EN, LEN=8
    return true;
}

/* ===== slv4WriteBlocking / slv4ReadBlocking ================================= */
bool MPU9150::slv4WriteBlocking(uint8_t dev, uint8_t reg, uint8_t data, uint32_t timeout_us) {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_ADDR, dev))  return false; // write
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_REG,  reg))  return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_DO,   data)) return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_CTRL, 0x80)) return false; // EN

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < timeout_us) {
        uint8_t st=0;
        if (!readBlock(_mpu, REG_I2C_MST_STATUS, &st, 1)) return false;
        if (st & 0x40) { if (st & 0x10) return false; return true; } // DONE / NACK
    }
    return false;
}
bool MPU9150::slv4ReadBlocking(uint8_t dev, uint8_t reg, uint8_t& data, uint32_t timeout_us) {
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_ADDR, (uint8_t)(0x80 | dev))) return false; // read
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_REG,  reg))                   return false;
    if (!_bus.writeRegister(_mpu, REG_I2C_SLV4_CTRL, 0x80))                  return false; // EN

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < timeout_us) {
        uint8_t st=0;
        if (!readBlock(_mpu, REG_I2C_MST_STATUS, &st, 1)) return false;
        if (st & 0x40) { if (st & 0x10) return false; break; } // DONE / NACK
    }
    if (!readBlock(_mpu, REG_I2C_SLV4_DI, &data, 1)) return false;
    return true;
}

/* ===== readASA =============================================================== */
bool MPU9150::readASA() {
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_FUSEROM,   2000);

    uint8_t asax=128, asay=128, asaz=128;
    slv4ReadBlocking(_mag, AK_REG_ASAX, asax, 2000);
    slv4ReadBlocking(_mag, AK_REG_ASAY, asay, 2000);
    slv4ReadBlocking(_mag, AK_REG_ASAZ, asaz, 2000);

    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);

    auto adj = [](uint8_t ASA)->float { return ((float)ASA - 128.0f) * 0.5f / 128.0f + 1.0f; };
    _asaAdjX = adj(asax);
    _asaAdjY = adj(asay);
    _asaAdjZ = adj(asaz);
    return true;
}

/* ===== triggerMagIfDue ======================================================= */
void MPU9150::triggerMagIfDue(uint32_t now_us) {
    const uint32_t period_us = 12000; // ~83 Hz nominal; ~80 Hz with 800/1000 Hz loop
    if (!_magPresent || (uint32_t)(now_us - _lastMagTrigUs) < period_us) return;
    _lastMagTrigUs = now_us;
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_POWERDOWN, 2000);
    slv4WriteBlocking(_mag, AK_REG_CNTL, AK_MODE_SINGLE,    2000);
}

/* ===== computeScaledIMU / computeScaledMag ================================== */
void MPU9150::computeScaledIMU() {
    _ax_g = (float)_ax / ACC_LSB_PER_G;
    _ay_g = (float)_ay / ACC_LSB_PER_G;
    _az_g = (float)_az / ACC_LSB_PER_G;

    _gx_dps = (float)_gx / GYRO_LSB_PER_DPS;
    _gy_dps = (float)_gy / GYRO_LSB_PER_DPS;
    _gz_dps = (float)_gz / GYRO_LSB_PER_DPS;

    _temp_C = ((float)_temp) / 340.0f + 36.53f;
}
void MPU9150::computeScaledMag(int16_t mx, int16_t my, int16_t mz) {
    const float hx = (float)mx * _asaAdjX;
    const float hy = (float)my * _asaAdjY;
    const float hz = (float)mz * _asaAdjZ;
    _mx_uT = hx * MAG_uT_PER_LSB;
    _my_uT = hy * MAG_uT_PER_LSB;
    _mz_uT = hz * MAG_uT_PER_LSB;
}

/* ===== begin =================================================================
 * - Wake device, set ranges
 * - Configure ODR (from driver’s kRequestedGyroRateHz)
 * - Bring up internal I2C master @ ~400 kHz
 * - Probe AK8975, read ASA, start SLV0 stream
 */
bool MPU9150::begin() {
    if (!_bus.writeRegister(_mpu, REG_PWR_MGMT_1, 0x01)) return false;       // wake, PLL
    if (!_bus.writeRegister(_mpu, REG_GYRO_CONFIG,  (3 << 3))) return false; // gyro ±2000 dps
    if (!_bus.writeRegister(_mpu, REG_ACCEL_CONFIG, (3 << 3))) return false; // accel ±16 g

    if (!configureSampleRateFromDriver()) return false;

    if (!enableI2CMaster()) return false;

    _magPresent = probeMagWIA();
    if (_magPresent) {
        readASA();
        setupMagStream();
    }

    _lastMagTrigUs = micros();
    return true;
}

/* ===== read ================================================================== */
bool MPU9150::read() {
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

    const uint32_t now = micros();
    triggerMagIfDue(now);

    if (_magPresent) {
        uint8_t m[8] = {0};
        if (readBlock(_mpu, REG_EXT_SENS_DATA_00, m, sizeof(m))) {
            if (m[0] & 0x01) {
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
