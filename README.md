# AHRS-Lib (Skeleton)

**Goal:** Modular sensor/bus framework for an AHRS stack that will later host a quaternion-based MEKF/ESKF core. This skeleton prints **raw** sensor values only.

## Design Highlights
- **Abstraction Layers**
  - `Bus` (I2C now; SPI later) decouples drivers from MCU specifics.
  - `SensorBase` defines a uniform sensor interface.
  - `SensorsFactory` centralizes sensor creation from config.
- **Portability**
  - Arduino/PlatformIO compliant.
  - Teensy 4.1 target now; Nano 33 BLE and STM32 F7/H7 later by reusing the same drivers.
- **Scalability**
  - Add new sensors/microcontrollers without touching the future fusion core.

## Current Status
- **Implemented:** I2C bus, MPU9150 driver (accel/gyro/temp + AK8975 mag via bypass), raw printing.
- **Not Implemented Yet:** AHRS, scaling to physical units, GPS/optical input, SPI.

## Build (Teensy 4.1)
1. `platformio run -e teensy41`
2. `platformio run -e teensy41 -t upload`
3. `platformio device monitor -b 115200`

## Output
CSV header:
```
t_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp_raw
```

All values are **raw register counts** (no scaling).

## Next Steps
- Add scaling helpers to convert raw -> dps, g, uT, °C.
- Introduce a **SensorHub** that aggregates multiple drivers (IMU, baro, GPS).
- Add SPI bus and alternate IMUs.
- Implement the MEKF/15-state ESKF using quaternions (no gimbal lock).
