#include "SensorsFactory.h"
std::unique_ptr<SensorHub> SensorsFactory::create(BusI2C& bus) {
    return std::unique_ptr<SensorHub>(new SensorHub(bus));
}
