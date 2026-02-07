# RP2040 Black Box

Flight data logging system using an Adafruit RP2040 microcontroller to capture IMU, GPS, barometric, and magnetometer data at up to 200Hz with Mahony AHRS filtering.

## Hardware

| Component | Description |
|-----------|-------------|
| Adafruit Feather RP2040 Adalogger | Main board with SD card slot |
| Adafruit BMP390 | Barometric altimeter (50Hz) |
| Adafruit LSM6DSOX | 6-DOF accelerometer/gyroscope (208Hz) |
| Adafruit LIS3MDL | 3-axis magnetometer (155Hz) |
| Adafruit Mini GPS PA1010D | GPS module (10Hz) |
| Adafruit PCF8523 | Real-time clock |

### Hardware Guides
- [Adafruit Feather RP2040 Adalogger](https://learn.adafruit.com/adafruit-feather-rp2040-adalogger/overview)
- [Adafruit BMP390](https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/overview)
- [Adafruit Mini GPS PA1010D](https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module/overview)
- [Adafruit PCF8523 RTC](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/overview)
- [Adafruit LSM6DSOX + LIS3MDL 9-DOF](https://learn.adafruit.com/st-9-dof-combo/overview)

### Enclosure

The first enclosure prototype is complete.

![Enclosure R1 assembled](images/enclosure_r1_assembled.jpg)

| Part | Quantity |
|------|----------|
| M2.5 heat inserts | 14 |
| M2.5 6mm standoffs | 12 |
| M2.5 6mm screws | 12 |
| M3 heat inserts | 4 |
| M3 6mm standoffs | 4 |
| M3 6mm screws | 4 |
| 50mm STEMMA QT cable | 3 |
| 100mm STEMMA QT cable | 1 |
| SD card (FAT32) | 1 |
| 3.3V battery (2-pin JST-PH) | 1 |

---

## Project Structure

```
rp2040-black-box/
├── firmware/
│   ├── quat_datalogger/       # Dual-core Mahony AHRS + SD card logging
│   └── test/
│       └── sd_card_tester/    # SD card diagnostic tool
├── software/
│   └── flight_visualizer/     # Python Streamlit web app
├── data/
│   └── samples/               # KITTI-derived test datasets
└── hardware/
    └── boards/                # STEP files and fab prints
```

---

## Getting Started

### Arduino Setup

Add this URL to Arduino IDE > File > Preferences > Additional Boards Manager URLs:
```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Install **Raspberry Pi Pico/RP2040/RP2350** by Earle F. Philhower, III

Select board: **Tools > Boards > Raspberry Pi Pico RP2040/RP2350 > Adafruit Feather RP2040 Adalogger**

> Note: If no serial port appears, that's normal - RP2040 uses USB bootloader. A port will appear after first upload.

### Arduino Libraries

Install via Library Manager (Tools > Manage Libraries):

| Library | Purpose |
|---------|---------|
| **SdFat** | SD card operations |
| **Adafruit BMP3XX** | BMP390 altimeter |
| **Adafruit GPS Library** | PA1010D GPS |
| **RTClib** | PCF8523 real-time clock |
| **Adafruit LIS3MDL** | Magnetometer |
| **Adafruit LSM6DSOX** | Accelerometer/Gyroscope |
| **Adafruit AHRS** | Mahony sensor fusion |
| **Adafruit Unified Sensor** | Common sensor interface (dependency) |
| **Adafruit BusIO** | I2C/SPI helper (dependency) |

> **Important:** This project uses the `SdFat` library (not the standard Arduino `SD.h`) for proper RP2040 Adalogger hardware support.

### SD Card

**Hardware configuration:**
- **CS Pin:** 23 (NOT pin 10!)
- **SPI Bus:** SPI1 @ 16MHz
- **Library:** SdFat (configured automatically in firmware)

**Requirements:**
- Format: FAT32 with 4KB allocation unit size
- Works with most SD/SDHC cards

**Troubleshooting:**
If SD initialization fails, check that the card is inserted and formatted as FAT32 with 4KB allocation units. Try cleaning the contacts or using a different card. The firmware retries 5 times automatically. Use `firmware/test/sd_card_tester/` to diagnose issues.

---

## Firmware

### quat_datalogger

Dual-core flight data logger with real-time AHRS filtering and signal processing.

**Core 0** reads sensors and logs to SD at 100Hz. **Core 1** runs the Mahony AHRS filter at 200Hz.

**Signal processing pipeline:**
- Adaptive Mahony filter (Kp ramps 10.0 → 1.5, Ki = 0.01 for drift correction)
- Sensor calibration: magnetometer hard/soft iron, gyro auto-cal at boot, accelerometer offsets
- Altitude: BMP390 hardware IIR → software low-pass filter → GPS complementary filter
- Auto sea-level pressure calibration from first good GPS fix
- GPS quality rejection (min satellites, max HDOP) with low-pass filtered speed/heading
- Automatic file splitting every ~1 GB to stay under FAT32 4 GB limit

All calibration constants and filter tuning parameters are editable at the top of the file.

### sd_card_tester

Diagnostic tool for verifying SD card compatibility. Tests initialization, read/write, and speed benchmarking.

---

## Data Format

**Output:** 20-column CSV at 100Hz

| Column | Description | Units |
|--------|-------------|-------|
| ms | Timestamp | milliseconds |
| qw, qx, qy, qz | Orientation quaternion | - |
| ax, ay, az | Acceleration | m/s² |
| gx, gy, gz | Angular velocity | rad/s |
| mx, my, mz | Magnetic field | uT |
| alt | Filtered altitude | meters |
| gps_fix | GPS lock status | 0/1 |
| lat, lon | GPS coordinates | decimal degrees |
| speed | Filtered GPS speed | knots |
| heading | Filtered GPS heading | degrees |

**File naming:**
- RTC available: `20260130_143022_0.csv`, `..._1.csv`, `..._2.csv`
- RTC unavailable: `log0_0.csv`, `log0_1.csv`, ...
- Header: `# Start: YYYY-MM-DDTHH:MM:SS`

---

## Calibration

All calibration constants are at the top of `firmware/quat_datalogger/quat_datalogger.ino`.

### 1. Gyroscope Bias (automatic)

The firmware averages 500 samples at boot while stationary. Keep the device still for 5 seconds after power-on. The computed bias is printed to serial.

To skip the wait, note the printed values and hardcode them:
```cpp
static const bool GYRO_AUTO_CAL = false;
static float gyro_bias[3] = { -0.0023f, 0.0041f, -0.0012f };
```

### 2. Magnetometer Hard/Soft Iron (manual, critical)

This is the most important calibration. Without it, heading (yaw) will be unreliable.

1. Log raw `mx, my, mz` values while slowly rotating the device through all orientations (figure-8 pattern, 60+ seconds)
2. Feed the data into [MotionCal](https://www.pjrc.com/store/prop_shield.html) (free) to compute hard-iron offsets and soft-iron correction matrix
3. Enter the values in the firmware:
```cpp
static const float MAG_HARD_IRON[3] = { offset_x, offset_y, offset_z };
static const float MAG_SOFT_IRON[3][3] = {
  { m00, m01, m02 },
  { m10, m11, m12 },
  { m20, m21, m22 }
};
```
4. Validate by plotting corrected mag data in 3D — it should form a sphere centered at the origin (not required but helpful for understanding of calibration)

Do this calibration with the device in its final enclosure. Nearby metal and magnets affect the result.

### 3. Accelerometer Offsets (manual)

Six-position calibration:

1. Place the device flat, on its back, on each of its four sides (6 orientations total)
2. Record ~100 samples in each position
3. For each axis, compute the offset as the midpoint of the two opposing readings (axis direction listed on board). It is recommended to use the serial monitor to copy data if possbile, as to prevent extra work from sifting through multiple CSV's:
```
offset_x = (ax_left + ax_right) / 2
offset_y = (ay_nose_down + ay_nose_up) / 2
offset_z = (az_flat + az_flipped) / 2
```
4. Enter the values:
```cpp
static const float ACCEL_OFFSET[3] = { offset_x, offset_y, offset_z };
```

### 4. Sea-Level Pressure (automatic)

The firmware auto-calibrates from the first good GPS fix. No action needed.

For manual override, set your local QNH from a weather service:
```cpp
static float SEA_LEVEL_HPA = 1018.5f;
static const bool BARO_GPS_AUTO_CAL = false;
```

### 5. Validation

After calibrating, run these checks:

| Test | Procedure | Expected Result |
|------|-----------|-----------------|
| Static | Place flat and still for 30s after boot | Quaternion settles to ~[1, 0, 0, 0] |
| Rotation | Rotate 90° and return | Quaternion returns to original value |
| GPS walk | Walk a known rectangle outdoors | Lat/lon tracks your path, altitude is stable |

---

## Flight Visualizer

Interactive web-based visualization using Streamlit.

```bash
pip install streamlit pandas numpy plotly scipy
cd software/flight_visualizer
streamlit run app.py
```

Live demo: **[rp2040-fdr-visualizer.streamlit.app](https://rp2040-fdr-visualizer.streamlit.app/)**

---

## Future Plans

### Enclosure
- Ruggedize with TPU secondary enclosure for crash survivability

### Hardware
- Upgrade GPS module to use a larger antenna for better signal
- Design custom PCB integrating RP2040 and all sensors onto a single board
- Add Bluetooth for wireless flight data streaming
- **Consider BNO085 IMU**: Replace LSM6DSOX + LIS3MDL with BNO085 for onboard hardware EKF fusion — reduces CPU load with hardware-accelerated quaternion output and automatic calibration

### Power & Monitoring
- Optimize battery usage and create power consumption metrics
- Add internal telemetry: battery life, board temperature

### Software
- Add Simulink visualization for real-time 3D flight simulation and analysis

### AI Integration
- Transition to an AI-optimized microcontroller for on-device inference
- AI-enhanced dead reckoning to reduce position drift when GPS is unavailable
- Altitude fusion model to correct for local pressure variations
- Predict remaining battery life based on flight behavior patterns
- Auto-label flight phases (takeoff, cruise, landing, idle) from sensor signatures
- Anomaly detection to flag unusual vibration or orientation patterns
