#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "Bus.h"

/**
 * @brief Robust I2C sensor detection (WHO_AM_I probes).
 * Probes primary + alternate addresses, validates WHO values, and only
 * reports a device when both the transfer and ID match.
 */
namespace SensorDetect {

struct Found {
    bool        present = false;
    uint8_t     addr    = 0;
    uint8_t     who     = 0;
    const char* name    = "";
};

struct Inventory {
    Found imu;    // MPU6050/9150 (IMU)
    Found gyro;   // L3GD20 / L3GD20H
    Found accel;  // LSM303D (accel+mag)
    Found mag;    // LSM303D.mag (logical alias if accel present)
    Found baro;   // LPS25H
};

// ---- low-level helper ----
inline bool readWho(Bus& bus, uint8_t addr7, uint8_t reg, uint8_t& whoOut) {
    uint8_t w = 0;
    if (!bus.readRegisters(addr7, reg, &w, 1)) {
        whoOut = 0;
        return false;
    }
    whoOut = w;
    return true;
}

// ---- probes ----
inline bool probeMPU9150(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    // MPU-6050/9150 WHO_AM_I at 0x75 -> 0x68 (AD0=0) or 0x69 (AD0=1)
    const uint8_t addrs[2] = {0x68, 0x69};
    for (auto a : addrs) {
        uint8_t w = 0;
        if (readWho(bus, a, 0x75, w) && (w == 0x68 || w == 0x69)) { addrOut = a; who = w; return true; }
    }
    return false;
}

inline bool probeL3GD20H(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    // L3GD20H / L3GD20 WHO_AM_I at 0x0F -> 0xD7 (20H) or 0xD4 (20)
    const uint8_t addrs[2] = {0x6B, 0x6A};
    for (auto a : addrs) {
        uint8_t w = 0;
        if (readWho(bus, a, 0x0F, w) && (w == 0xD7 || w == 0xD4)) { addrOut = a; who = w; return true; }
    }
    return false;
}

inline bool probeLSM303D(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    // LSM303D WHO_AM_I at 0x0F -> 0x49
    const uint8_t addrs[2] = {0x1D, 0x1E};
    for (auto a : addrs) {
        uint8_t w = 0;
        if (readWho(bus, a, 0x0F, w) && w == 0x49) { addrOut = a; who = w; return true; }
    }
    return false;
}

inline bool probeLPS25H(Bus& bus, uint8_t& addrOut, uint8_t& who) {
    // LPS25H WHO_AM_I at 0x0F -> 0xBD
    const uint8_t addrs[2] = {0x5D, 0x5C};
    for (auto a : addrs) {
        uint8_t w = 0;
        if (readWho(bus, a, 0x0F, w) && w == 0xBD) { addrOut = a; who = w; return true; }
    }
    return false;
}

inline Inventory detectAll(Bus& bus) {
    Inventory inv;
    uint8_t addr=0, who=0;

    if (probeMPU9150(bus, addr, who)) { inv.imu   = {true, addr, who, "MPU9150"}; }
    if (probeL3GD20H(bus, addr, who)) { inv.gyro  = {true, addr, who, "L3GD20H/L3GD20"}; }
    if (probeLSM303D(bus, addr, who)) {
        inv.accel = {true, addr, who, "LSM303D (accel+mag)"};
        // For inventory, report mag as part of LSM303D as well (same chip / same addr).
        inv.mag   = {true, addr, who, "LSM303D.mag"};
    }
    if (probeLPS25H(bus, addr, who))  { inv.baro  = {true, addr, who, "LPS25H"}; }

    return inv;
}

inline void printInventory(const Inventory& inv) {
    Serial.println(F("# Detected inventory:"));
    if (inv.imu.present)   { Serial.print(F("#  IMU: "));   Serial.print(inv.imu.name);   Serial.print(F(" @0x")); Serial.println(inv.imu.addr,  HEX); }
    if (inv.gyro.present)  { Serial.print(F("#  Gyro: "));  Serial.print(inv.gyro.name);  Serial.print(F(" @0x")); Serial.println(inv.gyro.addr, HEX); }
    if (inv.accel.present) { Serial.print(F("#  Accel: ")); Serial.print(inv.accel.name); Serial.print(F(" @0x")); Serial.println(inv.accel.addr,HEX); }
    if (inv.mag.present)   { Serial.print(F("#  Mag: "));   Serial.print(inv.mag.name);   Serial.print(F(" @0x")); Serial.println(inv.mag.addr,  HEX); }
    if (inv.baro.present)  { Serial.print(F("#  Baro: "));  Serial.print(inv.baro.name);  Serial.print(F(" @0x")); Serial.println(inv.baro.addr, HEX); }
}

} // namespace SensorDetect