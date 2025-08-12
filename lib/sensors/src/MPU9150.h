#pragma once
#include <Arduino.h>
#include "SensorBase.h"
#include "Bus.h"

/**
 * MPU9150 driver (accel + gyro + temp + AK8975 mag via internal I2C master)
 *
 * Ownership / Responsibilities (important for modularity):
 * - THIS DRIVER decides the gyro/accel output data rate (ODR) and reports it via sampleRateHz().
 * - The application/fusion layer should schedule itself from sampleRateHz() to stay in lockstep.
 *
 * Quick reference (ranges & rates):
 * - Gyro FS (GYRO_CONFIG[4:3]): ±250/500/1000/2000 dps (LSB/dps: 131/65.5/32.8/16.4)
 * - Accel FS (ACCEL_CONFIG[4:3]): ±2/4/8/16 g (LSB/g: 16384/8192/4096/2048)
 * - DLPF (CONFIG.DLPF_CFG): 1..6 => base 1 kHz; 0 => base 8 kHz
 * - Fs (DLPF 1..6): Fs = 1000 / (1 + SMPLRT_DIV)
 * - This driver keeps DLPF enabled (default 184 Hz) for good AHRS behavior.
 *
 * Magnetometer:
 * - AK8975 single-shot, ~80–83 Hz effective via SLV0 stream + SLV4 PD->SINGLE trigger.
 * - Fuse ROM ASA used; output in microtesla (uT).
 */
class MPU9150 : public SensorBase {
public:
    MPU9150(Bus& bus, uint8_t mpuAddr, uint8_t magAddr)
        : _bus(bus), _mpu(mpuAddr), _mag(magAddr) {}

    /** Initialize IMU + internal master + mag stream; sets ODR from driver’s requested rate. */
    bool begin() override;

    /** Acquire one IMU sample; non-blocking mag trigger/read; update scaled caches. */
    bool read() override;

    /** The configured accel/gyro ODR (Hz) that this driver actually applied. */
    uint16_t sampleRateHz() const override { return _gyroRateHzActual; }

    // ------------ Raw accessors ------------
    int16_t rawAx() const override { return _ax; }
    int16_t rawAy() const override { return _ay; }
    int16_t rawAz() const override { return _az; }
    int16_t rawGx() const override { return _gx; }
    int16_t rawGy() const override { return _gy; }
    int16_t rawGz() const override { return _gz; }
    int16_t rawMx() const override { return _mx; }
    int16_t rawMy() const override { return _my; }
    int16_t rawMz() const override { return _mz; }
    int16_t rawTemp() const override { return _temp; }

    // ------------ Scaled accessors ----------
    float ax_g()   const override { return _ax_g; }
    float ay_g()   const override { return _ay_g; }
    float az_g()   const override { return _az_g; }
    float gx_dps() const override { return _gx_dps; }
    float gy_dps() const override { return _gy_dps; }
    float gz_dps() const override { return _gz_dps; }
    float mx_uT()  const override { return _mx_uT; }
    float my_uT()  const override { return _my_uT; }
    float mz_uT()  const override { return _mz_uT; }
    float temp_C() const override { return _temp_C; }

private:
    // -------- Bus/addresses --------
    Bus&    _bus;
    uint8_t _mpu;
    uint8_t _mag;

    // -------- Driver-owned rate selection --------
    // Change kRequestedGyroRateHz here to select the device’s gyro/accel ODR.
    // Options (exact with DLPF on): 1000, 500, 333, 250, 200, 100, 50, 40, 20, 10...
    static constexpr uint16_t kRequestedGyroRateHz = 1000;

    // We record the actual applied rate (after divider rounding/clamp).
    uint16_t _gyroRateHzActual  = 1000;
    uint16_t _accelRateHzActual = 1000;

    // -------- Raw caches --------
    int16_t _ax=0,_ay=0,_az=0;
    int16_t _gx=0,_gy=0,_gz=0;
    int16_t _mx=0,_my=0,_mz=0;
    int16_t _temp=0;

    // -------- Scaled caches --------
    float _ax_g=0,_ay_g=0,_az_g=0;
    float _gx_dps=0,_gy_dps=0,_gz_dps=0;
    float _mx_uT=0,_my_uT=0,_mz_uT=0;
    float _temp_C=0;

    // -------- AK8975 per-axis sensitivity from fuse ROM --------
    float _asaAdjX = 1.0f, _asaAdjY = 1.0f, _asaAdjZ = 1.0f;

    // -------- Timing --------
    uint32_t _lastMagTrigUs = 0;
    bool     _magPresent = false;

    // -------- Registers / constants --------
    static constexpr uint8_t REG_SMPLRT_DIV        = 0x19;
    static constexpr uint8_t REG_CONFIG            = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG       = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG      = 0x1C;

    static constexpr uint8_t REG_I2C_MST_CTRL      = 0x24;
    static constexpr uint8_t REG_I2C_SLV0_ADDR     = 0x25;
    static constexpr uint8_t REG_I2C_SLV0_REG      = 0x26;
    static constexpr uint8_t REG_I2C_SLV0_CTRL     = 0x27;
    static constexpr uint8_t REG_I2C_SLV4_ADDR     = 0x31;
    static constexpr uint8_t REG_I2C_SLV4_REG      = 0x32;
    static constexpr uint8_t REG_I2C_SLV4_DO       = 0x33;
    static constexpr uint8_t REG_I2C_SLV4_CTRL     = 0x34;
    static constexpr uint8_t REG_I2C_SLV4_DI       = 0x35;
    static constexpr uint8_t REG_I2C_MST_STATUS    = 0x36;

    static constexpr uint8_t REG_INT_PIN_CFG       = 0x37;
    static constexpr uint8_t REG_EXT_SENS_DATA_00  = 0x49;
    static constexpr uint8_t REG_I2C_MST_DELAY     = 0x67;
    static constexpr uint8_t REG_ACCEL_XOUT_H      = 0x3B;
    static constexpr uint8_t REG_USER_CTRL         = 0x6A;
    static constexpr uint8_t REG_PWR_MGMT_1        = 0x6B;

    static constexpr uint8_t USER_I2C_MST_EN       = 0x20;
    static constexpr uint8_t USER_I2C_MST_RST      = 0x02;

    static constexpr uint8_t MST_CLK_400K          = 0x0D;   // internal master ≈ 400 kHz
    static constexpr uint8_t DLPF_CFG_184HZ        = 0x01;   // base rate 1 kHz with DLPF on

    // AK8975 regs/modes
    static constexpr uint8_t AK_REG_WIA   = 0x00; // expect 0x48
    static constexpr uint8_t AK_REG_ST1   = 0x02;
    static constexpr uint8_t AK_REG_HXL   = 0x03;
    static constexpr uint8_t AK_REG_ST2   = 0x09;
    static constexpr uint8_t AK_REG_CNTL  = 0x0A;
    static constexpr uint8_t AK_REG_ASAX  = 0x10;
    static constexpr uint8_t AK_REG_ASAY  = 0x11;
    static constexpr uint8_t AK_REG_ASAZ  = 0x12;
    static constexpr uint8_t AK_MODE_POWERDOWN = 0x00;
    static constexpr uint8_t AK_MODE_SINGLE    = 0x01;
    static constexpr uint8_t AK_MODE_FUSEROM   = 0x0F;

    // Scale factors for current ranges (±16 g, ±2000 dps)
    static constexpr float ACC_LSB_PER_G    = 2048.0f;
    static constexpr float GYRO_LSB_PER_DPS = 16.4f;
    static constexpr float MAG_uT_PER_LSB   = 0.3f;

    // --- helpers ---
    bool readBlock(uint8_t dev, uint8_t reg, uint8_t* buf, size_t len);
    static int16_t toInt16(uint8_t msb, uint8_t lsb) { return (int16_t)((msb << 8) | lsb); }

    bool configureSampleRateFromDriver();    // Uses kRequestedGyroRateHz
    bool enableI2CMaster();
    bool probeMagWIA();
    bool setupMagStream();
    bool slv4WriteBlocking(uint8_t dev, uint8_t reg, uint8_t data, uint32_t timeout_us);
    bool slv4ReadBlocking (uint8_t dev, uint8_t reg, uint8_t& data, uint32_t timeout_us);
    bool readASA();
    void triggerMagIfDue(uint32_t now_us);

    void computeScaledIMU();
    void computeScaledMag(int16_t mx, int16_t my, int16_t mz);
};
