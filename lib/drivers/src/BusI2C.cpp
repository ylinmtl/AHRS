#include "BusI2C.h"

bool BusI2C::begin() {
    // Note: Wire.begin() must be called before setClock on some cores.
    // Teensy supports setClock at runtime.
    Wire.begin();
    Wire.setClock(_clockHz);
    return true;
}

bool BusI2C::writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool BusI2C::readRegisters(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // repeated start
        return false;
    }
    size_t read = Wire.requestFrom((int)devAddr, (int)length, (int)true);
    if (read != length) return false;
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = Wire.read();
    }
    return true;
}
