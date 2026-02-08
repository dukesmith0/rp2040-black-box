/*
  RP2040 Black Box — Mahony AHRS Data Logger (Dual-Core)

  General architecture and output format:

  Core 0: Sensor reading, complementary filtering of altitude/GPS, data logging to SD
  Core 1: Mahony AHRS filter with adaptive gain

  CSV output (20 columns):
    ms, qw, qx, qy, qz, ax, ay, az, gx, gy, gz, mx, my, mz, alt, gps_fix, lat, lon, speed, heading

  File splitting: Creates new CSV every ~1 GB to stay under FAT32 4 GB limit.
*/

#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <RTClib.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>
#include <Adafruit_AHRS.h>

// ============================================================
//  CALIBRATION CONSTANTS — Edit these post calibration (see instructions in README)
// ============================================================

// --- Magnetometer Hard-Iron Offsets (µT) ---
// Measured by rotating device in figure-8 and finding sphere center.
// Use MotionCal (https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal_)
// or log raw mag data and compute in Python/MATLAB.
static float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };

// --- Magnetometer Soft-Iron Matrix ---
// Corrects ellipsoid distortion back to a sphere.
// Identity matrix = no correction. Replace with your 3×3 result.
static float MAG_SOFT_IRON[3][3] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f }
};

// --- Gyroscope Bias (rad/s) ---
// Measured at startup automatically (average of first N samples at rest).
// You can also hardcode known offsets here and skip auto-cal.
static const bool  GYRO_AUTO_CAL       = true;   // true = auto-calibrate at boot, false = use hardcoded values below
static const int   GYRO_CAL_SAMPLES    = 500;     // Samples to average (500 samples / 100 HZ sample rate = 5 seconds)
static float gyro_bias[3]              = { 0.0f, 0.0f, 0.0f };

// --- Accelerometer Offsets (m/s²) ---
// From 6-position calibration. Subtract from raw readings.
static float ACCEL_OFFSET[3] = { 0.0f, 0.0f, 0.0f };

// --- Barometric Altitude ---
// Set to local sea-level pressure (hPa) for accurate altitude.
// Check weather service before each session, or use GPS auto-cal.
static float       SEA_LEVEL_HPA       = 1013.25f;
static const bool  BARO_GPS_AUTO_CAL   = true;    // Auto-set from first good GPS fix

// ============================================================
//  FILTER TUNING
// ============================================================

// --- Mahony AHRS (Core 1) ---
static const float MAHONY_KP_STARTUP   = 10.0f;   // High gain for fast initial convergence
static const float MAHONY_KP_STEADY    = 1.5f;    // Low gain for smooth steady-state
static const float MAHONY_KI           = 0.01f;   // Integral gain for gyro bias correction
static const float ADAPTIVE_TRANSITION = 5.0f;    // Seconds until Kp drops to steady value

// --- Filter / Logging Rates ---
// Gigher filter rate allows for better sensor fusion, slower update rate means lower CPU load due to SD card writes.
static const int   FILTER_RATE_HZ      = 200;     // AHRS update rate (Core 1), matches IMU 208 Hz
static const int   LOG_RATE_HZ         = 100;     // SD card logging rate (Core 0)
static const int   FILTER_PER_LOG      = FILTER_RATE_HZ / LOG_RATE_HZ;

// --- Altitude Low-Pass Filter ---
// Gets rid of high-frequency noise in barometric altitude. Good for blending with GPS altitude (noisy but stable).
static const float ALT_LPF_ALPHA       = 0.1f;    // 0.0 = no update, 1.0 = no filtering

// --- Altitude Complementary Filter (baro + GPS) ---
static const float ALT_COMP_BARO_WEIGHT = 0.98f;  // Trust baro short-term
// GPS weight = 1.0 - ALT_COMP_BARO_WEIGHT

// --- GPS Low-Pass Filter ---
// Both outputs are noisy, need low pass filtering. 
static const float GPS_SPEED_LPF_ALPHA  = 0.15f;  // Smoothing for GPS speed
static const float GPS_HEADING_LPF_ALPHA = 0.15f;  // Smoothing for GPS heading

// --- GPS Quality Rejection ---
// Reject GPS fixes that don't meet these criteria to avoid contaminating the filter with crappy data.
// Adjust lower if you want to accept more fixes (at the risk of noisier data), or raise if you want only the best fixes (but more dropouts).
static const int   GPS_MIN_SATELLITES   = 4;       // Minimum sats for a "good" fix
static const float GPS_MAX_HDOP         = 5.0f;    // Maximum HDOP for a "good" fix

// ============================================================
//  FILE SPLITTING
// ============================================================

static const uint32_t FILE_SPLIT_BYTES  = 1000000000UL; // ~1 GB per file

// ============================================================
//  HARDWARE OBJECTS
// ============================================================

Adafruit_BMP3XX   bmp;
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  lis3mdl;
Adafruit_GPS      GPS_sensor(&Wire);
RTC_PCF8523       rtc;
Adafruit_Mahony   filter;

// SD card setup from Adafruit RP2040 Adalogger: CS pin 23, SPI1
#define SD_CS_PIN 23
SdFat  SD;
FsFile logFile;
SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// ============================================================
//  SHARED DATA BETWEEN CORES
// ============================================================

// Core 0 to Core 1: raw calibrated sensor data for Mahony filtering, indicator if the data is new.
struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  volatile bool newData;
};

// Core 1 to Core 0: quaternion output for logging, indicator if quaternion data is ready
struct FilterOutput {
  float qw, qx, qy, qz;
  volatile bool ready;
};

volatile SensorData sensorData   = { 0, 0, 0, 0, 0, 0, 0, 0, 0, false };
volatile FilterOutput filterOutput = { 1.0f, 0, 0, 0, false };

// ============================================================
//  CORE 0 STATE
// ============================================================

static String   fileBaseName    = "data"; // default naming convention for output files (overridden by RTC if available)
static String   fileExt         = ".csv"; // defines output file extension
static int      fileSplitIndex  = 0; // number of times file has been split
static uint32_t bytesWritten    = 0; // track bytes for splitting
static bool     rtcValid        = false; // see if RTC time can be used for filename generation

// Timing
static uint32_t lastSampleTime  = 0; // For main loop timing, filter update rate
static int      filterTickCount = 0; // Counts filter ticks to decimate for logging

// Filtered altitude state
static float    altFiltered     = 0.0f; // complementary filter output for altitude (baro + GPS)
static bool     altInitialized  = false; // flag to initialize altFiltered on first run
static float    lastBaroAlt     = 0.0f; // Last raw barometric altitude reading for complementary filter

// Filtered GPS state
static float    gpsSpeedFiltered   = 0.0f; // low-pass filtered GPS speed for smoother output
static float    gpsHeadingFiltered = 0.0f; // low-pass filtered GPS heading for smoother output (handles 0/360 wraparound)
static bool     gpsFilterInit      = false; // flag to initialize GPS filters on first good fix
static bool     baroPressureCaled  = false; // flag to indicate if barometric pressure has been auto-calibrated from GPS altitude

// Baro read decimation (BMP390 runs at 50 Hz, main loop at 200 Hz → read every 4th tick)
static const int BARO_DECIMATION = FILTER_RATE_HZ / 50; // Counter to track when to read barometer
static int       baroTickCount   = 0; // Counts main loop ticks to decimate barometer reads

// ============================================================
//  CORE 1: Mahony AHRS Filter with Adaptive Gain
// ============================================================

// setting up and adding filter variables previosuly declared as global volatile
void setup1() {
  filter.begin(FILTER_RATE_HZ);
  filter.setKp(MAHONY_KP_STARTUP);
  filter.setKi(MAHONY_KI);
}

// If there is sensor data, copy it locally and run the Mahony filter update, then publish the quaternion output for logging.
void loop1() {
  if (!sensorData.newData) return;

  // Copy shared data locally
  float ax = sensorData.ax;
  float ay = sensorData.ay;
  float az = sensorData.az;
  float gx = sensorData.gx;
  float gy = sensorData.gy;
  float gz = sensorData.gz;
  float mx = sensorData.mx;
  float my = sensorData.my;
  float mz = sensorData.mz;
  sensorData.newData = false;

  // --- ADAPTIVE GAIN ---
  // Ramp Kp from startup value down to steady value over ADAPTIVE_TRANSITION seconds
  float elapsed = millis() / 1000.0f;
  if (elapsed < ADAPTIVE_TRANSITION) {
    float t = elapsed / ADAPTIVE_TRANSITION;               // 0 → 1
    float kp = MAHONY_KP_STARTUP + t * (MAHONY_KP_STEADY - MAHONY_KP_STARTUP);
    filter.setKp(kp);
  } else {
    filter.setKp(MAHONY_KP_STEADY);
  }

  // Convert gyro from rad/s → deg/s (Adafruit AHRS expects deg/s)
  float gx_dps = gx * SENSORS_RADS_TO_DPS;
  float gy_dps = gy * SENSORS_RADS_TO_DPS;
  float gz_dps = gz * SENSORS_RADS_TO_DPS;

  // MAHONY FILTER UPDATE
  filter.update(gx_dps, gy_dps, gz_dps, ax, ay, az, mx, my, mz);

  // Publish quaternion
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  filterOutput.qw = qw;
  filterOutput.qx = qx;
  filterOutput.qy = qy;
  filterOutput.qz = qz;
  filterOutput.ready = true;
}

// ============================================================
//  CORE 0: Sensor Reading and Data Logging
// ============================================================

// --- Helper: Load calibration from SD card config file ---
// Reads "calibration.cfg" from SD root. Falls back to compiled defaults if missing.
// Format: key = value (comma-separated floats). Lines starting with # are comments.
static const char* CAL_FILENAME = "calibration.cfg";

static bool parseFloats(const char* str, float* out, int count) {
  for (int i = 0; i < count; i++) {
    char* end;
    out[i] = strtof(str, &end);
    if (end == str) return false; // parse failure
    str = end;
    // Skip comma and whitespace
    while (*str == ',' || *str == ' ' || *str == '\t') str++;
  }
  return true;
}

static void loadCalibration() {
  if (!SD.exists(CAL_FILENAME)) {
    Serial.println("No calibration.cfg found, using defaults.");
    return;
  }

  FsFile cfg = SD.open(CAL_FILENAME, FILE_READ);
  if (!cfg) {
    Serial.println("Failed to open calibration.cfg, using defaults.");
    return;
  }

  Serial.println("Loading calibration.cfg...");
  int loaded = 0;
  char line[256];

  while (cfg.available()) {
    // Read one line
    int len = 0;
    while (cfg.available() && len < (int)sizeof(line) - 1) {
      char c = cfg.read();
      if (c == '\n') break;
      if (c != '\r') line[len++] = c;
    }
    line[len] = '\0';

    // Skip empty lines and comments
    if (len == 0 || line[0] == '#') continue;

    // Find '=' separator
    char* eq = strchr(line, '=');
    if (!eq) continue;

    // Split key and value
    *eq = '\0';
    char* key = line;
    char* val = eq + 1;

    // Trim trailing spaces from key
    char* kend = eq - 1;
    while (kend > key && (*kend == ' ' || *kend == '\t')) *kend-- = '\0';

    // Trim leading spaces from value
    while (*val == ' ' || *val == '\t') val++;

    // Match keys
    if (strcmp(key, "mag_hard_iron") == 0) {
      if (parseFloats(val, MAG_HARD_IRON, 3)) {
        Serial.print("  mag_hard_iron = ");
        Serial.print(MAG_HARD_IRON[0], 4); Serial.print(", ");
        Serial.print(MAG_HARD_IRON[1], 4); Serial.print(", ");
        Serial.println(MAG_HARD_IRON[2], 4);
        loaded++;
      }
    } else if (strcmp(key, "mag_soft_iron") == 0) {
      float flat[9];
      if (parseFloats(val, flat, 9)) {
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++)
            MAG_SOFT_IRON[r][c] = flat[r * 3 + c];
        Serial.println("  mag_soft_iron = (3x3 matrix loaded)");
        loaded++;
      }
    } else if (strcmp(key, "accel_offset") == 0) {
      if (parseFloats(val, ACCEL_OFFSET, 3)) {
        Serial.print("  accel_offset = ");
        Serial.print(ACCEL_OFFSET[0], 4); Serial.print(", ");
        Serial.print(ACCEL_OFFSET[1], 4); Serial.print(", ");
        Serial.println(ACCEL_OFFSET[2], 4);
        loaded++;
      }
    } else if (strcmp(key, "sea_level_hpa") == 0) {
      float v;
      if (parseFloats(val, &v, 1) && v > 800.0f && v < 1200.0f) {
        SEA_LEVEL_HPA = v;
        Serial.print("  sea_level_hpa = ");
        Serial.println(SEA_LEVEL_HPA, 2);
        loaded++;
      }
    }
  }

  cfg.close();
  Serial.print("Calibration loaded (");
  Serial.print(loaded);
  Serial.println(" values).");
}

// --- Helper, build split filename ---
static String buildFileName() {
  return fileBaseName + "_" + String(fileSplitIndex) + fileExt;
}

// --- Helper, open a new log file (or next split) ---
static bool openLogFile() {
  if (logFile) {
    logFile.close();
  }

  String name = buildFileName();
  logFile = SD.open(name.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.print("Failed to open: ");
    Serial.println(name);
    return false;
  }

  // Write header if rtc working
  if (rtcValid) {
    DateTime now = rtc.now();
    char ts[40];
    snprintf(ts, sizeof(ts), "# Start: %04d-%02d-%02dT%02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    logFile.println(ts);
  }
  // Header line for CSV columns
  logFile.println("ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading");
  logFile.flush(); // ensure header is written immediately
  bytesWritten = 0;

  Serial.print("Logging to: ");
  Serial.println(name);
  return true;
}

// --- Helper: Freaky gyro auto-calibration at startup ---
static void calibrateGyro() {
  Serial.print("Gyro calibration (hold still)...");
  float sum[3] = { 0, 0, 0 };
  sensors_event_t accel, gyro, temp;

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sox.getEvent(&accel, &gyro, &temp);
    sum[0] += gyro.gyro.x;
    sum[1] += gyro.gyro.y;
    sum[2] += gyro.gyro.z;
    delay(10);
  }

  gyro_bias[0] = sum[0] / GYRO_CAL_SAMPLES;
  gyro_bias[1] = sum[1] / GYRO_CAL_SAMPLES;
  gyro_bias[2] = sum[2] / GYRO_CAL_SAMPLES;

  Serial.print(" done (bias: ");
  Serial.print(gyro_bias[0], 4); Serial.print(", ");
  Serial.print(gyro_bias[1], 4); Serial.print(", ");
  Serial.print(gyro_bias[2], 4); Serial.println(")");
}

// --- Helper: Apply magnetometer calibration (if not done super inaccurate!) ---
static void calibrateMag(float &mx, float &my, float &mz) {
  // Hard-iron removal
  float x = mx - MAG_HARD_IRON[0];
  float y = my - MAG_HARD_IRON[1];
  float z = mz - MAG_HARD_IRON[2];

  // Soft-iron correction
  mx = MAG_SOFT_IRON[0][0] * x + MAG_SOFT_IRON[0][1] * y + MAG_SOFT_IRON[0][2] * z;
  my = MAG_SOFT_IRON[1][0] * x + MAG_SOFT_IRON[1][1] * y + MAG_SOFT_IRON[1][2] * z;
  mz = MAG_SOFT_IRON[2][0] * x + MAG_SOFT_IRON[2][1] * y + MAG_SOFT_IRON[2][2] * z;
}

// --- Helper: Check if GPS fix is trustworthy and not destroying dead reckoning ---
static bool isGpsGoodFix() {
  if (!GPS_sensor.fix) return false;
  // PA1010D provides satellites via GPGGA and HDOP available too
  if (GPS_sensor.satellites < GPS_MIN_SATELLITES) return false;
  if (GPS_sensor.HDOP > GPS_MAX_HDOP) return false;
  return true;
}

// --- Helper: Low-pass filter for angles (handles 0/360 wraparound so no weird graphs) ---
static float lerpAngle(float filtered, float raw, float alpha) {
  float diff = raw - filtered;
  // Wrap to [-180, 180]
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  float result = filtered + alpha * diff;
  // Wrap to [0, 360]
  while (result < 0.0f)    result += 360.0f;
  while (result >= 360.0f) result -= 360.0f;
  return result;
}

// Setup is about 5-6 seconds, KEEP STILL DURING THIS if GYRO_AUTO_CAL is enabled
void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C for potential faster sensor reads

  // --- Initialize BMP390 (barometric altimeter) ---
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not found!");
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);  // Heavy hardware smoothing to reduce altitude noise
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // --- Initialize LSM6DSOX (accel + gyro) ---
  if (!sox.begin_I2C()) {
    Serial.println("LSM6DSOX not found!");
    while (1) delay(10);
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_208_HZ);   // Matches (slightly higher than) 200 Hz filter rate
  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);

  // --- Initialize LIS3MDL (magnetometer) ---
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL not found!");
    while (1) delay(10);
  }
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ); // LIS3MDL trying its best
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // --- Gyro Auto-Calibration ---
  if (GYRO_AUTO_CAL) {
    calibrateGyro();
  }

  // --- Initialize GPS ---
  GPS_sensor.begin(0x10);
  GPS_sensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_sensor.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS_sensor.sendCommand(PGCMD_ANTENNA);

  // --- Initialize RTC ---
  if (!rtc.begin()) {
    Serial.println("RTC not found");
  } else {
    // If a computer is connected via USB and the RTC needs setting,
    // set it to the compile timestamp (accurate right after upload).
    // Wait briefly for USB to enumerate — on battery this times out quickly.
    if (!rtc.initialized() || rtc.lostPower()) {
      uint32_t usbWaitStart = millis();
      while (!Serial && (millis() - usbWaitStart < 2000)) delay(10);

      if (Serial) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        rtc.start();
        Serial.println("RTC set to compile time (USB detected).");
      } else {
        Serial.println("RTC not set, using fallback naming");
      }
    }

    if (rtc.initialized() && !rtc.lostPower()) {
      rtcValid = true;
      DateTime now = rtc.now();
      char ts[30];
      snprintf(ts, sizeof(ts), "RTC: %04d-%02d-%02d %02d:%02d:%02d",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
      Serial.println(ts);
    }
  }

  // --- Initialize SD Card ---
  Serial.print("Initializing SD card...");
  delay(100);

  // Added retry logic due to some cheap SD card issues in the past.
  int retries = 0;
  while (!SD.begin(sdConfig)) {
    retries++;
    if (retries >= 5) {
      Serial.println("\nSD Card failed after 5 retries!");
      break;
    }
    Serial.print(".");
    delay(1000);
  }

  if (retries < 5) {
    Serial.println(" OK");

    // Load calibration from SD card (before logging starts)
    loadCalibration();

    // Build base filename from RTC or sequential fallback
    if (rtcValid) {
      DateTime now = rtc.now();
      char buf[20];
      snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
      fileBaseName = String(buf);
    } else {
      for (int i = 0; i < 1000; i++) {
        fileBaseName = "log" + String(i);
        String test = fileBaseName + "_0" + fileExt;
        if (!SD.exists(test.c_str())) break;
      }
    }

    fileSplitIndex = 0;
    openLogFile();
  }

  Serial.println("Mahony AHRS active (adaptive Kp, Ki enabled)");
  Serial.print("Filter: "); Serial.print(FILTER_RATE_HZ);
  Serial.print(" Hz, Log: "); Serial.print(LOG_RATE_HZ);
  Serial.println(" Hz");
}

void loop() {
  // --- GPS: Read continuously (non-blocking) ---
  GPS_sensor.read();
  if (GPS_sensor.newNMEAreceived()) {
    GPS_sensor.parse(GPS_sensor.lastNMEA());
  }

  // --- Main Loop at FILTER_RATE_HZ (200 Hz) ---
  uint32_t now = millis();
  if (now - lastSampleTime < (1000 / FILTER_RATE_HZ)) return;
  lastSampleTime += (1000 / FILTER_RATE_HZ);
  if (now - lastSampleTime > (1000 / FILTER_RATE_HZ)) {
    lastSampleTime = now;  // Guard against falling behind
  }

  // ---- READ IMU SENSORS ----
  sensors_event_t accel, gyro, temp, mag;
  sox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  // ---- APPLY CALIBRATION ----

  // Accelerometer offset
  float ax = accel.acceleration.x - ACCEL_OFFSET[0];
  float ay = accel.acceleration.y - ACCEL_OFFSET[1];
  float az = accel.acceleration.z - ACCEL_OFFSET[2];

  // Gyroscope bias removal
  float gx = gyro.gyro.x - gyro_bias[0];
  float gy = gyro.gyro.y - gyro_bias[1];
  float gz = gyro.gyro.z - gyro_bias[2];

  // Magnetometer hard/soft iron correction
  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;
  calibrateMag(mx, my, mz);

  // ---- PASS TO CORE 1 (AHRS MAHONY FILTER) ----
  sensorData.ax = ax;
  sensorData.ay = ay;
  sensorData.az = az;
  sensorData.gx = gx;
  sensorData.gy = gy;
  sensorData.gz = gz;
  sensorData.mx = mx;
  sensorData.my = my;
  sensorData.mz = mz;
  sensorData.newData = true;

  // ---- READ BAROMETER (decimated to ~50 Hz) ----
  baroTickCount++;
  if (baroTickCount >= BARO_DECIMATION) {
    baroTickCount = 0;
    if (bmp.performReading()) {
      lastBaroAlt = 44330.0f * (1.0f - powf((bmp.pressure / 100.0f) / SEA_LEVEL_HPA, 0.1903f));
    }
  }

  // ---- ALTITUDE LOW-PASS FILTER ----
  if (!altInitialized) {
    altFiltered = lastBaroAlt;
    altInitialized = true;
  } else {
    altFiltered = ALT_LPF_ALPHA * lastBaroAlt + (1.0f - ALT_LPF_ALPHA) * altFiltered;
  }

  // ---- ALTITUDE COMPLEMENTARY FILTER (baro + GPS) ----
  if (isGpsGoodFix() && GPS_sensor.altitude > 0.0f) {
    float gpsAlt = GPS_sensor.altitude;
    altFiltered = ALT_COMP_BARO_WEIGHT * altFiltered
                + (1.0f - ALT_COMP_BARO_WEIGHT) * gpsAlt;

    // Auto-calibrate sea-level pressure from first good GPS fix
    if (BARO_GPS_AUTO_CAL && !baroPressureCaled) {
      // Reverse the barometric formula: P0 = P / (1 - alt/44330)^5.255 , thanks to Wikipedia
      float rawPressure = bmp.pressure / 100.0f;  // Pa to hPa
      float ratio = 1.0f - (gpsAlt / 44330.0f);
      if (ratio > 0.0f && rawPressure > 300.0f && rawPressure < 1200.0f) {
        float calP0 = rawPressure / powf(ratio, 5.255f);
        if (calP0 > 800.0f && calP0 < 1200.0f) {
          SEA_LEVEL_HPA = calP0;
          baroPressureCaled = true;
          Serial.print("Baro auto-cal: sea level = ");
          Serial.print(SEA_LEVEL_HPA, 2);
          Serial.println(" hPa");
        }
      }
    }
  }

  // ---- GPS SPEED / HEADING LOW-PASS FILTER ----
  bool goodGps = isGpsGoodFix();
  float gpsSpeed   = 0.0f;
  float gpsHeading = 0.0f;
  bool  gpsFix     = false;

  if (goodGps) {
    gpsFix = true;
    float rawSpeed   = GPS_sensor.speed;
    float rawHeading = GPS_sensor.angle;

    if (!gpsFilterInit) {
      gpsSpeedFiltered   = rawSpeed;
      gpsHeadingFiltered = rawHeading;
      gpsFilterInit      = true;
    } else {
      gpsSpeedFiltered   = GPS_SPEED_LPF_ALPHA * rawSpeed
                         + (1.0f - GPS_SPEED_LPF_ALPHA) * gpsSpeedFiltered;
      gpsHeadingFiltered = lerpAngle(gpsHeadingFiltered, rawHeading, GPS_HEADING_LPF_ALPHA);
    }

    gpsSpeed   = gpsSpeedFiltered;
    gpsHeading = gpsHeadingFiltered;
  }

  // ---- LOG TO SD (decimated to LOG_RATE_HZ) ----
  filterTickCount++;
  if (filterTickCount < FILTER_PER_LOG) return;
  filterTickCount = 0;

  // Get latest quaternion from core 1
  float qw = filterOutput.qw;
  float qx = filterOutput.qx;
  float qy = filterOutput.qy;
  float qz = filterOutput.qz;

  // Build CSV line using char buffer (avoids String heap fragmentation, learned from Adafruit examples and personal experience)
  char line[256];
  int len = snprintf(line, sizeof(line),
    "%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%d,%.6f,%.6f,%.2f,%.2f",
    (unsigned long)millis(),
    qw, qx, qy, qz,
    ax, ay, az,
    gx, gy, gz,
    mx, my, mz,
    altFiltered,
    gpsFix ? 1 : 0,
    gpsFix ? GPS_sensor.latitudeDegrees : 0.0f,
    gpsFix ? GPS_sensor.longitudeDegrees : 0.0f,
    gpsSpeed,
    gpsHeading
  );

  // Serial output
  Serial.println(line);

  // SD output
  if (logFile) {
    logFile.println(line);
    bytesWritten += len + 2;  // +2 for \r\n

    // Flush every second 
    // (data is buffered in RAM and written every second, improving performance but not risking too much data loss)
    static int flushCount = 0;
    if (++flushCount >= LOG_RATE_HZ) {
      logFile.flush();
      flushCount = 0;
    }

    // ---- FILE SPLITTING (every ~1 GB) ----
    if (bytesWritten >= FILE_SPLIT_BYTES) {
      fileSplitIndex++;
      openLogFile();
      Serial.print("Split → file #");
      Serial.println(fileSplitIndex);
    }
  }
}
