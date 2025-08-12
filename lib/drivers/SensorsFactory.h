#pragma once
#include <memory>
#include "SensorHub.h"
#include "BusI2C.h"

class SensorsFactory {
public:
    static std::unique_ptr<SensorHub> create(BusI2C& bus);
};
