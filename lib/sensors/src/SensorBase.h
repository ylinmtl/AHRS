#pragma once
#include <Arduino.h>
#include "Bus.h"

/**
 * @brief Common base class for all sensors.
 *
 * Adds both raw and scaled accessors so the fusion core can choose.
 * Scaled units:
 *  - Gyro: degrees per second (dps)
 *  - Accel: Gs
 *  - Mag: microtesla (uT)
 *  - Temp: deg C
 */
class SensorBase {
public:
    virtual ~SensorBase() = default;

    /** Initialize sensor (after Bus.begin()). */
    virtual bool begin() = 0;

    /** Read latest sample into internal cache. */
    virtual bool read() = 0;

    // ----- Raw (register counts) -----
    virtual int16_t rawAx() const = 0;
    virtual int16_t rawAy() const = 0;
    virtual int16_t rawAz() const = 0;

    virtual int16_t rawGx() const = 0;
    virtual int16_t rawGy() const = 0;
    virtual int16_t rawGz() const = 0;

    virtual int16_t rawMx() const = 0;
    virtual int16_t rawMy() const = 0;
    virtual int16_t rawMz() const = 0;

    virtual int16_t rawTemp() const = 0;

    // ----- Scaled -----
    virtual float ax_g()  const = 0;
    virtual float ay_g()  const = 0;
    virtual float az_g()  const = 0;

    virtual float gx_dps() const = 0;
    virtual float gy_dps() const = 0;
    virtual float gz_dps() const = 0;

    virtual float mx_uT() const = 0;
    virtual float my_uT() const = 0;
    virtual float mz_uT() const = 0;

    virtual float temp_C() const = 0;
};
