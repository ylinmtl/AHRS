#include <Arduino.h>
#include "Config.h"
#include "Bus.h"
#include "BusI2C.h"
#include "SensorsFactory.h"

static inline uint32_t micros32() { return micros(); }

BusI2C i2c(Config::I2C_CLOCK_HZ);
SensorBase* imu = nullptr;

void printHeaderOnce() {
    Serial.println(F("t_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx_uT,my_uT,mz_uT,temp_C"));
}

void setup() {
    Serial.begin(115200);
    delay(400);

    Serial.println(F("# AHRS-Lib: Raw+scaled sensor test (Teensy 4.1 + MPU9150)"));
    Serial.print(F("# I2C @ ")); Serial.print(Config::I2C_CLOCK_HZ); Serial.println(F(" Hz"));

    if (!i2c.begin()) { Serial.println(F("# ERROR: I2C begin failed")); while(1) delay(1000); }

    imu = SensorsFactory::createIMU(i2c);
    if (!imu)          { Serial.println(F("# ERROR: Sensor factory returned null")); while(1) delay(1000); }
    if (!imu->begin()) { Serial.println(F("# ERROR: IMU begin failed")); while(1) delay(1000); }

    Serial.print(F("# IMU sample rate (Hz): "));
    Serial.println(imu->sampleRateHz());   // <-- driver-owned ODR

    printHeaderOnce();
}

void loop() {
    // Period is derived from the driver's configured ODR so app/fusion stays in lockstep.
    static uint32_t period_us = 0;
    static bool init = false;
    if (!init && imu) {
        const uint16_t rate = imu->sampleRateHz();
        period_us = 1000000UL / (rate ? rate : 800); // fallback to 800 if zero
        init = true;
    }

    static uint32_t next_t = micros32();
    const uint32_t now = micros32();

    if ((int32_t)(now - next_t) >= 0) {
        next_t += period_us;

        if (imu->read()) {
            Serial.print(now);           Serial.print(',');
            Serial.print(imu->gx_dps()); Serial.print(',');
            Serial.print(imu->gy_dps()); Serial.print(',');
            Serial.print(imu->gz_dps()); Serial.print(',');
            Serial.print(imu->ax_g());   Serial.print(',');
            Serial.print(imu->ay_g());   Serial.print(',');
            Serial.print(imu->az_g());   Serial.print(',');
            Serial.print(imu->mx_uT());  Serial.print(',');
            Serial.print(imu->my_uT());  Serial.print(',');
            Serial.print(imu->mz_uT());  Serial.print(',');
            Serial.println(imu->temp_C());
        }
    }
}
