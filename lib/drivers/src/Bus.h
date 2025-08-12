#pragma once
#include <Arduino.h>

/**
 * @brief Abstract bus interface. 
 * 
 * Purpose:
 * - Decouples sensor drivers from the underlying MCU and bus implementation.
 * - Allows swapping I2C/SPI (or mocked buses for tests) without changing drivers.
 */
class Bus {
public:
    virtual ~Bus() = default;

    /**
     * @brief Initialize the bus hardware.
     * 
     * Interaction:
     * - Called by higher-level code (e.g., factory) before any sensor begins.
     */
    virtual bool begin() = 0;

    /**
     * @brief Write a single register on a device.
     * @param devAddr     7-bit device address.
     * @param reg         Register address.
     * @param value       Single byte to write.
     */
    virtual bool writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value) = 0;

    /**
     * @brief Read multiple registers starting at @p reg.
     * @param devAddr     7-bit device address.
     * @param reg         First register to read.
     * @param buffer      Destination buffer.
     * @param length      Number of bytes to read.
     */
    virtual bool readRegisters(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) = 0;
};
