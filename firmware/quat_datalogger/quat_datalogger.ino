/*
  RP2040 Black Box Data Logger - Dual Core with Madgwick Filter

  Core 0: Sensor reading, data logging to SD card
  Core 1: Madgwick AHRS filter processing

  Outputs CSV format: ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>
#include <Adafruit_AHRS.h>

// Sensors (Core 0 only)
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;
Adafruit_GPS GPS(&Wire);
RTC_PCF8523 rtc;

// Madgwick filter (Core 1 only)
Adafruit_Madgwick filter;

// Settings
#define FILTER_UPDATE_RATE_HZ 100
#define GPSECHO false

// Shared sensor data structure (Core 0 writes, Core 1 reads)
struct SensorData {
  float ax, ay, az;       // Acceleration (m/s^2)
  float gx, gy, gz;       // Gyro (rad/s)
  float mx, my, mz;       // Magnetometer (uT)
  volatile bool newData;  // Flag: new data available
};

// Shared filter output (Core 1 writes, Core 0 reads)
struct FilterOutput {
  float qw, qx, qy, qz;   // Quaternion
  volatile bool ready;    // Flag: filter has produced output
};

// Volatile shared data between cores
volatile SensorData sensorData = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};
volatile FilterOutput filterOutput = {1.0f, 0, 0, 0, false};

// Core 0 globals
File logFile;
String fileName = "data.csv";
uint32_t last_sample_time = 0;
bool rtcValid = false;

// ============================================================
// CORE 1: Madgwick Filter Processing
// ============================================================

void setup1() {
  // Initialize Madgwick filter on Core 1
  filter.begin(FILTER_UPDATE_RATE_HZ);
}

void loop1() {
  // Wait for new sensor data from Core 0
  if (sensorData.newData) {
    // Copy sensor data locally (minimize time holding shared data)
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

    // Convert gyro to deg/s for Madgwick filter
    float gx_deg = gx * SENSORS_RADS_TO_DPS;
    float gy_deg = gy * SENSORS_RADS_TO_DPS;
    float gz_deg = gz * SENSORS_RADS_TO_DPS;

    // Update Madgwick filter
    filter.update(gx_deg, gy_deg, gz_deg, ax, ay, az, mx, my, mz);

    // Get quaternion output
    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);

    // Update shared output
    filterOutput.qw = qw;
    filterOutput.qx = qx;
    filterOutput.qy = qy;
    filterOutput.qz = qz;
    filterOutput.ready = true;
  }
}

// ============================================================
// CORE 0: Sensor Reading and Data Logging
// ============================================================

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
  // Uses default SS pin for the board variant
  if (!SD.begin()) {
    Serial.println("SD Card failed!");
  } else {
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
        if (!SD.exists(fileName)) break;
      }
    }

    logFile = SD.open(fileName, FILE_WRITE);
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
      logFile.println("ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading");
      logFile.flush();
      Serial.print("Logging to: "); Serial.println(fileName);
    }
  }

  Serial.println("Dual-core Madgwick filter active");
}

void loop() {
  // Read GPS continuously
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Main Loop @ 100Hz
  if (millis() - last_sample_time >= (1000 / FILTER_UPDATE_RATE_HZ)) {
    last_sample_time = millis();

    // Read IMU sensors
    sensors_event_t accel, gyro, temp, mag;
    sox.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    // Pass sensor data to Core 1 for filtering
    sensorData.ax = accel.acceleration.x;
    sensorData.ay = accel.acceleration.y;
    sensorData.az = accel.acceleration.z;
    sensorData.gx = gyro.gyro.x;
    sensorData.gy = gyro.gyro.y;
    sensorData.gz = gyro.gyro.z;
    sensorData.mx = mag.magnetic.x;
    sensorData.my = mag.magnetic.y;
    sensorData.mz = mag.magnetic.z;
    sensorData.newData = true;

    // Read Altitude
    bmp.performReading();

    // Get quaternion from Core 1 (use last available if not ready)
    float qw = filterOutput.qw;
    float qx = filterOutput.qx;
    float qy = filterOutput.qy;
    float qz = filterOutput.qz;

    // Build CSV String
    String data = "";
    data += String(millis()) + ",";

    // Quaternion (from Core 1 Madgwick filter)
    data += String(qw, 4) + "," + String(qx, 4) + "," + String(qy, 4) + "," + String(qz, 4) + ",";

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
      if (flushCounter++ > FILTER_UPDATE_RATE_HZ) {
        logFile.flush();
        flushCounter = 0;
      }
    }
  }
}
