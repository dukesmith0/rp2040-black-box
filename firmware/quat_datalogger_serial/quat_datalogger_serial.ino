/*
  RP2040 Black Box Data Logger - Dual Core with AHRS Filter (Serial Output Only)

  Core 0: Sensor reading, data output to serial
  Core 1: AHRS filter processing (Madgwick or Mahony)

  Outputs CSV format to Serial: ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading

  This version is for bench testing without SD card dependency.
*/

// ============================================================
// FILTER SELECTION (choose one)
// ============================================================
//#define USE_MADGWICK  // Alternative: Madgwick filter (~10s convergence, beta ~0.041)
#define USE_MAHONY  // Default: Mahony filter (~2-3s convergence with Kp=10.0)

#include <Wire.h>
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

// AHRS filter (Core 1 only)
#ifdef USE_MADGWICK
  Adafruit_Madgwick filter;
#elif defined(USE_MAHONY)
  Adafruit_Mahony filter;
#else
  #error "Must define either USE_MADGWICK or USE_MAHONY"
#endif

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
uint32_t last_sample_time = 0;

// ============================================================
// CORE 1: Madgwick Filter Processing
// ============================================================

void setup1() {
  // Initialize AHRS filter on Core 1
  filter.begin(FILTER_UPDATE_RATE_HZ);

#ifdef USE_MAHONY
  // Mahony filter tuning (fast convergence)
  filter.setKp(10.0f);  // Proportional gain (higher = faster convergence)
  filter.setKi(0.0f);   // Integral gain (0 = no drift correction)
#endif
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
// CORE 0: Sensor Reading and Serial Output
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial console

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

  // Print CSV header
  Serial.println("ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading");

  // Print active filter
#ifdef USE_MADGWICK
  Serial.println("Dual-core Madgwick filter active - Serial output only");
#elif defined(USE_MAHONY)
  Serial.println("Dual-core Mahony filter active (Kp=10.0) - Serial output only");
#endif
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

    // Output to Serial only
    Serial.println(data);
  }
}
