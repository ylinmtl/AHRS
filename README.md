# AHRS-Lib (Modular Sensor Hub for Arduino/Teensy)

A lightweight, modular sensor layer for **AHRS** projects on Arduino/Teensy. It standardizes access to IMUs and discrete sensors (gyro, accel, mag, baro), outputs **scaled SI-like units**, and keeps the **fusion core independent** from sensor/microcontroller specifics via clean abstraction layers.

> **Design goals**
> - Swap sensors or MCUs without touching fusion or application code
> - Use **quaternions** in the fusion layer to avoid gimbal lock (sensor layer ready)
> - Run from **800 Hz up to 8 kHz** (default pacing follows selected gyro; current drivers 800–1000 Hz)
> - Consistent units: gyro **dps**, accel **g**, mag **µT**, baro **Pa**, temp **°C**
> - Output Euler **degrees** from the fusion layer (next stage)

---

## Project layout

```
AHRS/
├─ platformio.ini
├─ include/
│  ├─ Bus.h                # abstract byte-addressed bus
│  └─ Config.h             # build-time config (enables, addresses, prefs, I2C clock)
├─ lib/
│  ├─ bus/
│  │  ├─ BusI2C.h/.cpp     # I²C implementation of Bus
│  └─ drivers/
│     ├─ SensorHub.h/.cpp  # orchestrator: selects sources, exposes scaled sample
│     ├─ SensorsFactory.*  # creates SensorHub for the given bus
│     └─ dev/              # self-contained device drivers
│        ├─ MPU9150.*      # InvenSense IMU + AK8975 mag via internal I2C master
│        ├─ L3GD20H.*      # ST gyro
│        ├─ LSM303D.*      # ST accel + mag
│        └─ LPS25H.*       # ST baro + temp
└─ src/
   └─ main.cpp             # demo: initialize, read, print CSV
```

---

## Build (PlatformIO)

1. Open the folder in VS Code with the **PlatformIO** extension.
2. Make sure `platformio.ini` targets your board (default: `teensy41`, Arduino framework).
3. Build and upload.

Serial monitor default is `115200` (see `platformio.ini`). At high loop rates, consider increasing baud to avoid bottlenecks (e.g. 1–2 Mbps on Teensy).

---

## Configuration (`include/Config.h`)

- **I²C clock**: `I2C_CLOCK_HZ` (default 400 kHz).
- **Enable flags**: select which sensors to build for (e.g., `Enable::MPU9150`, `Enable::L3GD20H`, `Enable::LSM303D`, `Enable::LPS25H`).
- **Addresses**: override 7‑bit addresses if your hardware differs.
- **Source preferences**: when multiple sensors provide the same quantity (e.g., gyro from MPU9150 vs L3GD20H), use `Select::PreferExternal*` toggles to choose default sources.

> The **SensorHub** probes what’s actually present at runtime and chooses sources accordingly, honoring your preferences when there’s overlap.

---

## Data model and units

`SensorHub::read(SensorScaled& s)` fills this struct:

- `gx_dps, gy_dps, gz_dps` — gyroscope in **degrees per second**
- `ax_g, ay_g, az_g` — accelerometer in **g**
- `mx_uT, my_uT, mz_uT` — magnetometer in **microtesla**
- `pressure_Pa` — barometric pressure in **pascal**
- `temperature_C` — temperature in **°C**

All drivers return **scaled** values; no raw counts leak above the driver layer.

---

## Timing and loop rate

- The **hub sample rate** follows the **selected gyro** (e.g., 1000 Hz for MPU9150; 800 Hz for L3GD20H).
- In `main.cpp`, the loop is paced off `hub->getSampleRateHz()` to keep sampling uniform for fusion.
- Printing every sample at 800–1000 Hz can saturate serial. For logging, either:
  - Increase baud rate, or
  - Downsample prints (e.g., print every Nth sample).

---

## How it works (flow)

```
I2C (Wire) ─► BusI2C ─► Device drivers       ┐
                              │               │
                              └──► SensorHub ─┼─► Scaled sample (SensorScaled)
                                              │
                               main.cpp ──────┘   (loop paced by hub sample rate)
```

- **Bus/BusI2C**: abstract and concrete byte-addressed bus. Drivers don’t talk to `Wire` directly.
- **Device drivers**: self-contained per-sensor; convert raw to scaled physical units.
- **SensorHub**: probes available sensors, selects sources for each quantity, exposes one unified read.
- **SensorsFactory**: simple place to swap bus/variants later (SPI, mocks, etc.).

---

## Example usage

```cpp
#include "BusI2C.h"
#include "SensorsFactory.h"

BusI2C bus(Config::I2C_CLOCK_HZ);
auto hub = SensorsFactory::create(bus);

void setup() {
  Serial.begin(115200);
  bus.begin();
  if (!hub->begin()) { Serial.println("Sensor init failed"); while(1){} }
}

void loop() {
  static const uint32_t period_us = (uint32_t)(1e6f / hub->getSampleRateHz());
  static uint32_t next = micros();
  if ((int32_t)(micros() - next) >= 0) {
    next += period_us;
    SensorScaled s;
    if (hub->read(s)) {
      // use s.gx_dps ... s.temperature_C
    }
  }
}
```

---

## Device specifics (quick refs)

- **MPU9150**: sets ±2000 dps and ±16 g; runs ~1000 Hz; reads AK8975 mag via the MPU’s internal I²C master (single‑shot at ~80 Hz). Temp formula: `°C = raw/340 + 36.53`.
- **L3GD20H**: 800 Hz ODR, ±2000 dps; sensitivity 0.07 dps/LSB; BDU on.
- **LSM303D**: accel ±16 g @ 800 Hz (≈0.000732 g/LSB). Mag ±4 gauss @ 25 Hz (0.016 µT/LSB). Temperature channel enabled for mag bias stability.
- **LPS25H**: 25 Hz, BDU on. Pressure: `Pa = (raw/4096)*100`. Temp: `°C = 42.5 + raw/480`.

---

## Extending the library

### Add a new sensor
1. Create `lib/drivers/dev/MySensor.h/.cpp`:
   - Constructor takes `Bus&` or a concrete bus; store 7‑bit address.
   - `begin()` probes WHO_AM_I and configures ODR, ranges, BDU.
   - `read(...)` returns scaled units.
2. Add your header to `SensorHub.h` and wire it in `SensorHub.cpp`:
   - Instantiate if `Config::Enable::MySensor` is true.
   - Update `choose*Source()` to select it when present.
3. Add address & enable flag in `Config.h`.

### Add a new bus (e.g., SPI)
1. Implement `class BusSPI : public Bus` with `begin`, `writeRegister`, `readRegisters`.
2. Make drivers accept a base `Bus&` (most do already) to stay bus‑agnostic.
3. Update `SensorsFactory` if you want to choose bus at runtime.

### Add the fusion core
- Place a `Fusion` module that consumes `SensorScaled` at the hub’s sample rate.
- Maintain a **quaternion state**; output Euler angles (deg) for UI/telemetry.
- Gate magnetometer usage when disturbed; let baro/GPS/vision provide slow altitude/heading drift correction.

---

## Troubleshooting

- **NaN pressure/temperature**: ensure LPS25H WHO_AM_I is `0xBD` and BDU is set. Check wiring/voltage and that `Config::Enable::LPS25H` is true.
- **No MPU9150 mag data**: AK8975 uses single‑shot mode; the driver triggers ~every 12 ms. If you bypass SensorHub and poll faster, ST1 may be 0—respect the conversion cadence.
- **Serial stalls at high rate**: raise baud or decimate prints.
- **Wrong units**: confirm your full‑scale settings match your expected sensitivity.

---

## License

MIT (suggested).

---

## Credits

Built for Teensy/Arduino using the Wire I²C stack. Designed by modular layering so the AHRS fusion core remains portable and robust.
