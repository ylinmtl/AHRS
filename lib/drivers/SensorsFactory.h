#pragma once
#include <memory>
#include "Bus.h"
#include "SensorHub.h"

/**
 * @class SensorsFactory
 * @brief Minimal factory that wires a SensorHub to the provided Bus.
 *
 * Keep this layer so you can later inject mocks/SPI variants without touching app code.
 */
class SensorsFactory {
public:
    /**
     * @brief Create a SensorHub bound to the provided bus.
     * @param bus Shared Bus (I2C in current builds).
     * @return unique_ptr owning the SensorHub.
     */
    static std::unique_ptr<SensorHub> create(Bus& bus);
};
