#include "SensorsFactory.h"

/**
 * @brief Construct a SensorHub using the provided Bus.
 *        Caller owns the Bus lifetime (typically a global/static in main).
 */
std::unique_ptr<SensorHub> SensorsFactory::create(Bus& bus) {
    return std::make_unique<SensorHub>(bus);
}
