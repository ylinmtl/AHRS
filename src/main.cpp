#include <Arduino.h>
#include "Config.h"
#include "Bus.h"
#include "BusI2C.h"
#include "SensorsFactory.h"

/**
 * @brief Minimal sensor-test app:
 * - Initializes I2C bus and MPU9150 via factory
 * - Prints raw register values at Config::SAMPLE_RATE_HZ
 * 
 * Interaction:
 * - This program is the "companion test" for your driver layer.
 * - The same driver will be used by the future AHRS core without modification.
 */

// Use elapsedMicros if available (Teensy), else fallback to micros()
static inline uint32_t micros32() { return micros(); }

BusI2C i2c(Config::I2C_CLOCK_HZ);
SensorBase* imu = nullptr;

/** @brief Print CSV header once for downstream logging tools. */
void printHeaderOnce() {
    Serial.println(F("t_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx_uT,my_uT,mz_uT,temp_C"));
}

/** @brief Arduino setup: init Serial, bus, create IMU via factory, and start it. */
void setup() {
    Serial.begin(115200);
    // Give the USB/Serial a moment to connect (especially handy on Teensy)
    delay(400);

    Serial.println(F("# AHRS-Lib: Raw sensor test (Teensy 4.1 + MPU9150)"));
    Serial.print(F("# I2C @ "));
    Serial.print(Config::I2C_CLOCK_HZ);
    Serial.println(F(" Hz"));

    if (!i2c.begin()) {
        Serial.println(F("# ERROR: I2C begin failed"));
        while (1) { delay(1000); }
    }

    imu = SensorsFactory::createIMU(i2c);
    if (!imu) {
        Serial.println(F("# ERROR: Sensor factory returned null"));
        while (1) { delay(1000); }
    }

    if (!imu->begin()) {
        Serial.println(F("# ERROR: IMU begin failed"));
        while (1) { delay(1000); }
    }

    printHeaderOnce();
}

/** @brief Arduino loop: read sensor and print raw values at the configured rate. */
void loop() {
    static const uint32_t period_us = 1000000UL / Config::SAMPLE_RATE_HZ;
    static uint32_t next_t = micros();
    const uint32_t now = micros();

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