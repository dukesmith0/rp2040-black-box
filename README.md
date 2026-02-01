# RP2040 Black Box

Flight data logging system using an Adafruit RP2040 microcontroller to capture IMU, GPS, barometric, and magnetometer data at 100Hz.

## Project Structure

```
rp2040-black-box/
├── firmware/
│   ├── quat_datalogger/     # Dual-core Madgwick AHRS (20-col CSV output)
│   └── raw_datalogger/      # Raw sensor data (16-col CSV output)
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

### Hardware Guides
- [Adafruit Feather RP2040 Adalogger](https://learn.adafruit.com/adafruit-feather-rp2040-adalogger/overview)
- [Adafruit BMP390](https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/overview)
- [Adafruit Mini GPS PA1010D](https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module/overview)
- [Adafruit PCF8523 RTC](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/overview)
- [Adafruit LSM6DSOX + LIS3MDL 9-DOF](https://learn.adafruit.com/st-9-dof-combo/overview)

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
| **Adafruit BMP3XX** | BMP390 altimeter |
| **Adafruit GPS Library** | PA1010D GPS |
| **RTClib** | PCF8523 real-time clock |
| **Adafruit LIS3MDL** | Magnetometer |
| **Adafruit LSM6DS** | Accelerometer/Gyroscope |
| **Adafruit AHRS** | Madgwick/Mahony sensor fusion (quat_datalogger only) |
| **Adafruit Unified Sensor** | Common sensor interface (dependency) |
| **Adafruit BusIO** | I2C/SPI helper (dependency) |

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

### quat_datalogger (20 columns)
- Dual-core processing: Core 0 reads sensors, Core 1 runs Madgwick filter
- Real-time quaternion output
- Output: `ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading`

### raw_datalogger (16 columns)
- Single-core, no filtering
- Raw sensor data for offline processing
- Output: `ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading`

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

### Power & Monitoring
- Optimize battery usage and create power consumption metrics
- Add internal telemetry: battery life, board temperature
- Use thermal data to optimize cooling and ensure stability throughout battery life

### AI Integration
- Transition to an AI-optimized microcontroller for on-device inference
- AI-enhanced dead reckoning to reduce position drift when GPS is unavailable
- Altitude fusion model to correct for local pressure variations
- Predict remaining battery life based on flight behavior patterns
- Auto-label flight phases (takeoff, cruise, landing, idle) from sensor signatures
- Anomaly detection to flag unusual vibration or orientation patterns
