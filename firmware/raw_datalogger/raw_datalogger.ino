/*
  RP2040 Black Box Raw Data Logger
  Outputs CSV format: ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading

  Raw sensor data only - no AHRS filtering. Use for offline sensor fusion.
*/

#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <RTClib.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>

// Sensors
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;
Adafruit_GPS GPS(&Wire);
RTC_PCF8523 rtc;

// SD Card configuration for Adafruit RP2040 Adalogger
#define SD_CS_PIN 23  // CS pin for RP2040 Adalogger (NOT pin 10!)
SdFat SD;
FsFile logFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// Settings
#define LOG_RATE_HZ 100
#define GPSECHO false

// Globals
String fileName = "data.csv";
uint32_t last_log_time = 0;
bool rtcValid = false;

void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10); // Uncomment to wait for serial console

  Wire.begin();

  // Initialize BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not found!");
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize LSM6DSOX
  if (!sox.begin_I2C()) {
    Serial.println("LSM6DSOX not found!");
    while (1) delay(10);
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // Initialize LIS3MDL
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL not found!");
    while (1) delay(10);
  }
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Initialize GPS
  GPS.begin(0x10); // Default I2C address
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC not set, using fallback naming");
  } else {
    rtcValid = true;
    Serial.println("RTC initialized");
  }

  // Initialize SD Card
  // CS pin 23 for Adafruit RP2040 Adalogger, SPI1 @ 16MHz
  Serial.print("Initializing SD card...");
  delay(100);  // Give SD card time to power up

  // Retry mechanism for SD card initialization
  int retryCount = 0;
  const int maxRetries = 5;
  bool sdInitialized = false;

  while (!SD.begin(config)) {
    retryCount++;
    if (retryCount >= maxRetries) {
      Serial.println();
      Serial.println("SD Card failed after 5 retries!");
      Serial.println("Troubleshooting:");
      Serial.println("  1. Is the SD card inserted properly?");
      Serial.println("  2. Is the card formatted as FAT32?");
      Serial.println("  3. Try formatting with 4KB allocation unit size");
      Serial.println("  4. Are the card contacts clean?");
      Serial.println("  5. Try a different SD card (some cheap cards don't work)");
      Serial.println("  6. Verify you're using CS pin 23 (NOT pin 10)");
      break;
    }
    Serial.print(".");
    delay(1000); // Wait before retrying
  }

  if (retryCount < maxRetries) {
    sdInitialized = true;
    Serial.println();
    Serial.println("SD Card initialized successfully");
    // Create filename from RTC or fall back to sequential
    if (rtcValid) {
      DateTime now = rtc.now();
      char buf[20];
      sprintf(buf, "%04d%02d%02d_%02d%02d%02d.csv",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());
      fileName = String(buf);
    } else {
      // Fallback to sequential naming
      for (int i = 0; i < 1000; i++) {
        fileName = "log" + String(i) + ".csv";
        if (!SD.exists(fileName.c_str())) break;
      }
    }

    logFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (logFile) {
      // Write start timestamp header if RTC is valid
      if (rtcValid) {
        DateTime now = rtc.now();
        char timestamp[32];
        sprintf(timestamp, "# Start: %04d-%02d-%02dT%02d:%02d:%02d",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());
        logFile.println(timestamp);
      }
      logFile.println("ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading");
      logFile.flush();
      Serial.print("Logging to: "); Serial.println(fileName);
    } else {
      Serial.println("Failed to open log file!");
    }
  }
}

void loop() {
  // Read GPS
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Main Loop @ 100Hz
  if (millis() - last_log_time >= (1000 / LOG_RATE_HZ)) {
    last_log_time = millis();

    // Read IMU
    sensors_event_t accel, gyro, temp, mag;
    sox.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    // Read Altitude
    bmp.performReading();

    // Build CSV String
    String data = "";
    data += String(millis()) + ",";

    // Accel (m/s^2)
    data += String(accel.acceleration.x, 2) + "," + String(accel.acceleration.y, 2) + "," + String(accel.acceleration.z, 2) + ",";

    // Gyro (rad/s)
    data += String(gyro.gyro.x, 4) + "," + String(gyro.gyro.y, 4) + "," + String(gyro.gyro.z, 4) + ",";

    // Mag (uT)
    data += String(mag.magnetic.x, 2) + "," + String(mag.magnetic.y, 2) + "," + String(mag.magnetic.z, 2) + ",";

    // Altitude (m)
    data += String(bmp.readAltitude(1013.25), 2) + ",";

    // GPS
    data += String(GPS.fix) + ",";
    if (GPS.fix) {
      data += String(GPS.latitudeDegrees, 6) + "," + String(GPS.longitudeDegrees, 6) + ",";
      data += String(GPS.speed) + "," + String(GPS.angle);
    } else {
      data += "0.0,0.0,0.0,0.0";
    }

    // Output
    Serial.println(data);
    if (logFile) {
      logFile.println(data);

      // Flush periodically (e.g. every 1 second)
      static int flushCounter = 0;
      if (flushCounter++ > LOG_RATE_HZ) {
        logFile.flush();
        flushCounter = 0;
      }
    }
  }
}
