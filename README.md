# RP2040 Black Box

Flight data logging system using an Adafruit RP2040 microcontroller to capture IMU, GPS, barometric, and magnetometer data at 100Hz.

## Project Structure

```
rp2040-black-box/
├── firmware/
│   ├── quat_datalogger/          # Dual-core AHRS + SD card (20-col CSV)
│   ├── quat_datalogger_serial/   # Dual-core AHRS, serial only (20-col CSV)
│   ├── raw_datalogger/           # Raw sensor data + SD card (16-col CSV)
│   ├── raw_datalogger_serial/    # Raw sensor data, serial only (16-col CSV)
│   └── test/
│       └── sd_card_tester/       # SD card diagnostic tool
├── software/
│   ├── analysis_scripts/    # MATLAB plotters
│   │   ├── filtered_quat_dataplotter.m   # For pre-filtered data
│   │   └── raw_filtering_dataplotter.m   # With sensor fusion
│   └── flight_visualizer/   # Python Streamlit app
├── data/
│   └── samples/             # KITTI-derived test datasets
└── hardware/
    └── boards/              # STEP files and fab prints
```

## Hardware

| Component | Description |
|-----------|-------------|
| Adafruit Feather RP2040 Adalogger | Main board with SD card slot |
| Adafruit BMP390 | Barometric altimeter (50Hz) |
| Adafruit LSM6DSOX | 6-DOF accelerometer/gyroscope (104Hz) |
| Adafruit LIS3MDL | 3-axis magnetometer (155Hz) |
| Adafruit Mini GPS PA1010D | GPS module (10Hz) |
| Adafruit PCF8523 | Real-time clock |
| **Alternative:** Adafruit BNO085 | 9-DOF IMU with onboard EKF fusion (100Hz quaternions) |

### Hardware Guides
- [Adafruit Feather RP2040 Adalogger](https://learn.adafruit.com/adafruit-feather-rp2040-adalogger/overview)
- [Adafruit BMP390](https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/overview)
- [Adafruit Mini GPS PA1010D](https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module/overview)
- [Adafruit PCF8523 RTC](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/overview)
- [Adafruit LSM6DSOX + LIS3MDL 9-DOF](https://learn.adafruit.com/st-9-dof-combo/overview)
- [Adafruit BNO085 9-DOF IMU](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/overview) (alternative with onboard EKF)

---

## Arduino Setup

### Board Manager
Add this URL to Arduino IDE > File > Preferences > Additional Boards Manager URLs:
```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Install **Raspberry Pi Pico/RP2040/RP2350** by Earle F. Philhower, III

Select board: **Tools > Boards > Raspberry Pi Pico RP2040/RP2350 > Adafruit Feather RP2040 Adalogger**

> Note: If no serial port appears, that's normal - RP2040 uses USB bootloader. A port will appear after first upload.

### Arduino Libraries

Install via Library Manager (Tools > Manage Libraries):

| Library | Required For |
|---------|--------------|
| **SdFat** | SD card operations (required for SD-enabled firmwares) |
| **Adafruit BMP3XX** | BMP390 altimeter |
| **Adafruit GPS Library** | PA1010D GPS |
| **RTClib** | PCF8523 real-time clock (SD-enabled firmwares) |
| **Adafruit LIS3MDL** | Magnetometer |
| **Adafruit LSM6DSOX** | Accelerometer/Gyroscope |
| **Adafruit AHRS** | Madgwick/Mahony sensor fusion (quat_datalogger variants) |
| **Adafruit Unified Sensor** | Common sensor interface (dependency) |
| **Adafruit BusIO** | I2C/SPI helper (dependency) |

> **Important:** This project uses the `SdFat` library (not the standard Arduino `SD.h`) for proper RP2040 Adalogger hardware support.

---

## SD Card Setup

### Hardware Configuration
The Adafruit RP2040 Adalogger uses specific SD card pins:
- **CS Pin:** 23 (NOT pin 10!)
- **SPI Bus:** SPI1 @ 16MHz
- **Library:** SdFat (configured automatically in firmware)

### SD Card Requirements
- **Format:** FAT32
- **Recommended:** 4KB allocation unit size
- **Capacity:** Works with most SD/SDHC cards

### Formatting Instructions
**Windows:**
1. Right-click SD card in File Explorer > Format
2. File system: FAT32
3. Allocation unit size: 4096 bytes
4. Quick Format: checked

**macOS/Linux:**
```bash
# Find the SD card device (e.g., /dev/sdb1)
lsblk

# Format as FAT32 with 4KB clusters
sudo mkfs.vfat -F 32 -s 8 /dev/sdX1
```

### Troubleshooting SD Card Issues
If SD initialization fails:
1. Ensure card is inserted properly
2. Verify card is formatted as FAT32
3. Try formatting with 4KB allocation unit size
4. Clean card contacts with isopropyl alcohol
5. Try a different SD card (some cheap cards are incompatible)
6. Check Serial Monitor for detailed error messages

The firmware includes automatic retry (5 attempts) with troubleshooting hints.

### Testing Your SD Card
Use the diagnostic tool to verify SD card functionality:
```
firmware/test/sd_card_tester/
```
This tool tests initialization, read/write operations, and speed benchmarking.

---

## MATLAB Setup

### For filtered_quat_dataplotter.m
- **MATLAB** (no special toolboxes required)

### For raw_filtering_dataplotter.m
- **MATLAB**
- **Navigation Toolbox** (for `ahrsfilter` sensor fusion)

---

## Python Setup

### Dependencies
```bash
pip install streamlit pandas numpy plotly scipy
```

### Run Visualizer
```bash
cd software/flight_visualizer
streamlit run app.py
```

Or try the live demo: **[rp2040-fdr-visualizer.streamlit.app](https://rp2040-fdr-visualizer.streamlit.app/)**

---

## Firmware Variants

### Production Firmwares (with SD Card)

#### quat_datalogger (20 columns)
- **Purpose:** Flight data logging with real-time AHRS filtering
- **Features:**
  - Dual-core processing: Core 0 reads sensors, Core 1 runs AHRS filter
  - Selectable filter: Madgwick or Mahony (default: Mahony for faster convergence)
  - SD card logging with RTC timestamps
  - 100Hz sampling rate
- **Output:** `ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading`
- **Use case:** Flight recording with orientation tracking

#### raw_datalogger (16 columns)
- **Purpose:** Flight data logging without filtering
- **Features:**
  - Single-core, no AHRS processing
  - SD card logging with RTC timestamps
  - Raw sensor data for offline analysis
  - 100Hz sampling rate
- **Output:** `ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading`
- **Use case:** Offline sensor fusion with MATLAB or custom algorithms

### Development Firmwares (Serial Output Only)

#### quat_datalogger_serial (20 columns)
- **Purpose:** Bench testing and development without SD card
- **Features:**
  - Dual-core AHRS filtering (same as quat_datalogger)
  - Serial output only (no SD card or RTC)
  - Real-time monitoring via Serial Monitor/Plotter
- **Output:** Same 20-column format as quat_datalogger
- **Use case:** Development, debugging, real-time visualization

#### raw_datalogger_serial (16 columns)
- **Purpose:** Bench testing and development without SD card
- **Features:**
  - Raw sensor data (same as raw_datalogger)
  - Serial output only (no SD card or RTC)
  - Real-time monitoring via Serial Monitor/Plotter
- **Output:** Same 16-column format as raw_datalogger
- **Use case:** Development, debugging, sensor calibration

### Diagnostic Tools

#### sd_card_tester
- **Purpose:** SD card diagnostic and troubleshooting
- **Location:** `firmware/test/sd_card_tester/`
- **Tests:**
  - SD card initialization with retry mechanism
  - Card type and size detection
  - File write/read operations
  - Speed benchmarking
  - Directory listing
- **Use case:** Verify SD card compatibility before flight

---

## Data Format

| Column | Description | Units |
|--------|-------------|-------|
| ms | Timestamp | milliseconds |
| qw, qx, qy, qz | Orientation quaternion | - |
| ax, ay, az | Acceleration | m/s² |
| gx, gy, gz | Angular velocity | rad/s |
| mx, my, mz | Magnetic field | µT |
| alt | Barometric altitude | meters |
| gps_fix | GPS lock status | 0/1 |
| lat, lon | GPS coordinates | decimal degrees |
| speed | GPS ground speed | knots |
| heading | GPS heading | degrees |

---

## CSV File Features

- RTC timestamp header: `# Start: YYYY-MM-DDTHH:MM:SS`
- Automatic file naming from RTC (e.g., `20260130_143022.csv`)
- Fallback to sequential naming if RTC unavailable (`log0.csv`, `log1.csv`, ...)

---

## Future Plans

### Enclosure
- Design and 3D print enclosure using Adafruit breakout boards (once arrived)
- Ruggedize with TPU secondary enclosure for crash survivability

### Hardware Improvements
- Upgrade GPS module to use a larger antenna for better signal
- Design custom PCB integrating RP2040 and all sensors onto a single board for reduced size, fewer cables, and lower cost
- Add Bluetooth for wireless flight data streaming and capture
- **Consider BNO085 IMU**: Replace LSM6DSOX + LIS3MDL with BNO085 for onboard hardware EKF fusion
  - Reduces RP2040 CPU load (no Core 1 filtering needed)
  - Hardware-accelerated quaternion output at 100Hz
  - Game Rotation Vector mode for drift-free orientation
  - Would require firmware modification to read quaternions from BNO085 instead of computing them

### Power & Monitoring
- Optimize battery usage and create power consumption metrics
- Add internal telemetry: battery life, board temperature
- Use thermal data to optimize cooling and ensure stability throughout battery life

### Software
- Add Simulink visualization for real-time 3D flight simulation and analysis

### AI Integration
- Transition to an AI-optimized microcontroller for on-device inference
- AI-enhanced dead reckoning to reduce position drift when GPS is unavailable
- Altitude fusion model to correct for local pressure variations
- Predict remaining battery life based on flight behavior patterns
- Auto-label flight phases (takeoff, cruise, landing, idle) from sensor signatures
- Anomaly detection to flag unusual vibration or orientation patterns
