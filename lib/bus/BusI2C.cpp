#include "BusI2C.h"

bool BusI2C::begin() {
    Wire.begin();
    Wire.setClock(clock_hz_);
    return true;
}
bool BusI2C::writeRegister(uint8_t dev7, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(dev7);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}
bool BusI2C::readRegisters(uint8_t dev7, uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(dev7);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t n = Wire.requestFrom((int)dev7, (int)len);
    if (n != len) return false;
    for (size_t i=0;i<len;++i) buf[i] = Wire.read();
    return true;
}
bool BusI2C::write(uint8_t dev7, uint8_t reg, const uint8_t* data, size_t len) {
    Wire.beginTransmission(dev7);
    Wire.write(reg);
    Wire.write(data, len);
    return Wire.endTransmission() == 0;
}
bool BusI2C::read(uint8_t dev7, uint8_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(dev7);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t n = Wire.requestFrom((int)dev7, (int)len);
    if (n != len) return false;
    for (size_t i=0;i<len;++i) data[i] = Wire.read();
    return true;
}
