#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Bus.h"

/**
 * @brief I2C bus implementation (Arduino Wire-based).
 * 
 * Purpose:
 * - Provides a concrete Bus usable by all I2C sensors.
 * - Keeps Wire-specific setup and transactions out of sensor drivers.
 */
class BusI2C : public Bus {
public:
    explicit BusI2C(uint32_t clockHz) : _clockHz(clockHz) {}

    /** @brief Initialize Wire with the configured clock speed. */
    bool begin() override;

    /** @brief Write a single register over I2C. */
    bool writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value) override;

    /** @brief Read consecutive registers over I2C into a buffer. */
    bool readRegisters(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) override;

private:
    uint32_t _clockHz;
};
