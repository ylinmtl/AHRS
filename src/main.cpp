#include <Arduino.h>
#include <memory>
#include "Config.h"
#include "BusI2C.h"
#include "SensorHub.h"
#include "SensorsFactory.h"

BusI2C bus(Config::I2C_CLOCK_HZ);
std::unique_ptr<SensorHub> hub;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("# AHRS-Lib: Modular sensor hub demo"));

    if (!bus.begin()) {
        Serial.println(F("# ERROR: I2C begin failed"));
        while (1) delay(1000);
    }

    hub = SensorsFactory::create(bus);
    if (!hub->begin()) {
        Serial.println(F("# ERROR: No gyro source available or begin failed."));
        while (1) delay(1000);
    }

    Serial.print(F("# Sample rate (Hz): "));
    Serial.println(hub->getSampleRateHz());
    Serial.println(F("t_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx_uT,my_uT,mz_uT,pressure_Pa,temp_C"));
}

void loop() {
    static uint32_t period_us = 0;
    if (!period_us) {
        float hz = hub->getSampleRateHz();
        if (hz < 1.0f) hz = 800.0f;
        period_us = (uint32_t)(1000000.0f / hz);
    }

    static uint32_t next_t = micros();
    const uint32_t now = micros();
    if ((int32_t)(now - next_t) >= 0) {
        next_t += period_us;

        SensorScaled s;
        if (hub->read(s)) {
            Serial.print(now); Serial.print(',');
            Serial.print(s.gx_dps); Serial.print(',');
            Serial.print(s.gy_dps); Serial.print(',');
            Serial.print(s.gz_dps); Serial.print(',');
            Serial.print(s.ax_g);   Serial.print(',');
            Serial.print(s.ay_g);   Serial.print(',');
            Serial.print(s.az_g);   Serial.print(',');
            Serial.print(s.mx_uT);  Serial.print(',');
            Serial.print(s.my_uT);  Serial.print(',');
            Serial.print(s.mz_uT);  Serial.print(',');
            Serial.print(s.pressure_Pa); Serial.print(',');
            Serial.println(s.temperature_C);
        }
    }
}
