#pragma once
#include <Arduino.h>
#include "SensorBase.h"
#include "Bus.h"
#include "MPU9150.h"
#include "Config.h"

/**
 * @brief Factory for creating sensor instances based on configuration.
 * 
 * Purpose:
 * - Centralizes construction so the rest of the app never touches concrete drivers.
 * - Makes it trivial to add new sensor types later.
 */
class SensorsFactory {
public:
    /**
     * @brief Create the IMU instance specified in Config::IMU.
     * @param bus Initialized bus reference (e.g., BusI2C).
     * @return SensorBase* (ownership to caller); nullptr on failure.
     * 
     * Interaction:
     * - Call begin() on the returned sensor before use.
     */
    static SensorBase* createIMU(Bus& bus) {
        switch (Config::IMU) {
            case Config::ImuType::MPU9150:
                return new MPU9150(bus, Config::Addr::MPU9150, Config::Addr::AK8975);
            default:
                return nullptr;
        }
    }
};
