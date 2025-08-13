#include "SensorsFactory.h"
#include "SensorDetect.hpp"

/**
 * @brief Construct a SensorHub using the provided Bus.
 *        Caller owns the Bus lifetime (typically a global/static in main).
 *        Also runs a lightweight bus scan to report detected sensors (Betaflight-style).
 */
std::unique_ptr<SensorHub> SensorsFactory::create(Bus& bus) {
    // If the bus is I2C, run detection and print inventory (optional).
    // We rely on RTTI here to downcast; on Arduino/Teensy, this is fine when -fno-rtti isn't set.
    auto inv = SensorDetect::detectAll(bus);
    SensorDetect::printInventory(inv);
    return std::make_unique<SensorHub>(bus);
}
