
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include "Bus.h"

/**
 * @brief Minimal Betaflight-style detection registry for I2C devices.
 * It probes common WHO_AM_I registers at primary/alternate addresses and records what is present.
 * This does not replace your drivers. It simply discovers devices to help with flexible swaps.
 */
namespace SensorDetect {

struct FoundIMU   { bool present=false; uint8_t addr=0; uint8_t who=0; const char* name=""; };
struct FoundGyro  { bool present=false; uint8_t addr=0; uint8_t who=0; const char* name=""; };
struct FoundAccel { bool present=false; uint8_t addr=0; uint8_t who=0; const char* name=""; };
struct FoundMag   { bool present=false; uint8_t addr=0; uint8_t who=0; const char* name=""; };
struct FoundBaro  { bool present=false; uint8_t addr=0; uint8_t who=0; const char* name=""; };

struct Inventory {
    FoundIMU   imu;
    FoundGyro  gyro;
    FoundAccel accel;
    FoundMag   mag;
    FoundBaro  baro;
};

inline bool readWho(Bus& bus, uint8_t addr7, uint8_t whoReg, uint8_t& out) {
    return bus.readRegisters(addr7, whoReg, &out, 1);
}

// Probe helpers for specific parts
inline bool probeMPU9150(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    const uint8_t addrs[2] = {0x68, 0x69};
    for (auto a : addrs) {
        if (readWho(bus, a, 0x75, who) && (who == 0x68 || who == 0x69)) { addrOut = a; return true; }
    }
    return false;
}
inline bool probeL3GD20H(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    const uint8_t addrs[2] = {0x6B, 0x6A};
    for (auto a : addrs) {
        if (readWho(bus, a, 0x0F, who) && (who == 0xD7 || who == 0xD4)) { addrOut = a; return true; }
    }
    return false;
}
inline bool probeLSM303D(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    const uint8_t addrs[2] = {0x1D, 0x1E};
    for (auto a : addrs) {
        if (readWho(bus, a, 0x0F, who) && who == 0x49) { addrOut = a; return true; }
    }
    return false;
}
inline bool probeLPS25H(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    const uint8_t addrs[2] = {0x5D, 0x5C};
    for (auto a : addrs) {
        if (readWho(bus, a, 0x0F, who) && who == 0xBD) { addrOut = a; return true; }
    }
    return false;
}

inline Inventory detectAll(Bus& bus) {
    Inventory inv;
    uint8_t addr=0, who=0;
    if (probeMPU9150(bus, addr, who)) { inv.imu = {true, addr, who, "MPU9150"}; }
    if (probeL3GD20H(bus, addr, who)) { inv.gyro = {true, addr, who, "L3GD20H/L3GD20"}; }
    if (probeLSM303D(bus, addr, who)) { inv.accel = {true, addr, who, "LSM303D (accel+mag)"}; inv.mag.present = true; inv.mag.addr = addr; inv.mag.who = who; inv.mag.name = "LSM303D.mag"; }
    if (probeLPS25H(bus, addr, who)) { inv.baro = {true, addr, who, "LPS25H"}; }
    return inv;
}

inline void printInventory(const Inventory& inv) {
    Serial.println(F("# Detected inventory:"));
    if (inv.imu.present)   { Serial.print(F("#  IMU: "));   Serial.print(inv.imu.name);   Serial.print(F(" @0x")); Serial.println(inv.imu.addr, HEX); }
    if (inv.gyro.present)  { Serial.print(F("#  Gyro: "));  Serial.print(inv.gyro.name);  Serial.print(F(" @0x")); Serial.println(inv.gyro.addr, HEX); }
    if (inv.accel.present) { Serial.print(F("#  Accel: ")); Serial.print(inv.accel.name); Serial.print(F(" @0x")); Serial.println(inv.accel.addr, HEX); }
    if (inv.mag.present)   { Serial.print(F("#  Mag: "));   Serial.print(inv.mag.name);   Serial.print(F(" @0x")); Serial.println(inv.mag.addr, HEX); }
    if (inv.baro.present)  { Serial.print(F("#  Baro: "));  Serial.print(inv.baro.name);  Serial.print(F(" @0x")); Serial.println(inv.baro.addr, HEX); }
}

} // namespace SensorDetect
