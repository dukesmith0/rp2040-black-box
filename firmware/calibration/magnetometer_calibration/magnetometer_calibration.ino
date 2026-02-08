/*
  RP2040 Black Box — Magnetometer Calibration for MotionCal

  Streams IMU data over serial in the format expected by PJRC MotionCal:
    Raw:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>
    Uni:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>

  Usage:
    1. Upload this sketch to the RP2040 Adalogger.
    2. Open MotionCal (https://www.pjrc.com/store/prop_shield.html) and select the serial port.
    3. Slowly rotate the device in all orientations (figure-8 pattern) until the sphere fills in.
    4. Read the calibration values from MotionCal:
       - "Magnetic Offset" → hard-iron offsets (3 values)
       - "Magnetic Mapping" → soft-iron matrix (3x3 = 9 values)
    5. Add to calibration.cfg on the SD card:

       # Magnetometer Hard-Iron Offsets (uT)
       mag_hard_iron = <X>, <Y>, <Z>

       # Magnetometer Soft-Iron Matrix (row-major)
       mag_soft_iron = <M00>, <M01>, <M02>, <M10>, <M11>, <M12>, <M20>, <M21>, <M22>

       The quat_datalogger firmware reads calibration.cfg at boot.

  Hardware: LSM6DSOX (accel/gyro) + LIS3MDL (magnetometer) on I2C
*/

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  lis3mdl;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  // --- Initialize LSM6DSOX (accel + gyro) ---
  if (!sox.begin_I2C()) {
    Serial.println("LSM6DSOX not found!");
    while (1) delay(10);
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // --- Initialize LIS3MDL (magnetometer) ---
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL not found!");
    while (1) delay(10);
  }
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  Serial.println("Magnetometer calibration ready. Open MotionCal and rotate the device.");
}

void loop() {
  sensors_event_t accel, gyro, temp, mag;
  sox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  // MotionCal "Raw:" line — scaled integers
  // Accel: m/s² → pseudo-raw (×8192/9.8)
  // Gyro:  rad/s → scaled deg/s (×RAD_TO_DEG×16)
  // Mag:   µT → scaled (×10)
  int raw_ax = (int)(accel.acceleration.x * 8192.0f / 9.8f);
  int raw_ay = (int)(accel.acceleration.y * 8192.0f / 9.8f);
  int raw_az = (int)(accel.acceleration.z * 8192.0f / 9.8f);
  int raw_gx = (int)(gyro.gyro.x * RAD_TO_DEG * 16.0f);
  int raw_gy = (int)(gyro.gyro.y * RAD_TO_DEG * 16.0f);
  int raw_gz = (int)(gyro.gyro.z * RAD_TO_DEG * 16.0f);
  int raw_mx = (int)(mag.magnetic.x * 10.0f);
  int raw_my = (int)(mag.magnetic.y * 10.0f);
  int raw_mz = (int)(mag.magnetic.z * 10.0f);

  Serial.print("Raw:");
  Serial.print(raw_ax); Serial.print(',');
  Serial.print(raw_ay); Serial.print(',');
  Serial.print(raw_az); Serial.print(',');
  Serial.print(raw_gx); Serial.print(',');
  Serial.print(raw_gy); Serial.print(',');
  Serial.print(raw_gz); Serial.print(',');
  Serial.print(raw_mx); Serial.print(',');
  Serial.print(raw_my); Serial.print(',');
  Serial.println(raw_mz);

  // MotionCal "Uni:" line — native float units
  Serial.print("Uni:");
  Serial.print(accel.acceleration.x, 2); Serial.print(',');
  Serial.print(accel.acceleration.y, 2); Serial.print(',');
  Serial.print(accel.acceleration.z, 2); Serial.print(',');
  Serial.print(gyro.gyro.x, 4);         Serial.print(',');
  Serial.print(gyro.gyro.y, 4);         Serial.print(',');
  Serial.print(gyro.gyro.z, 4);         Serial.print(',');
  Serial.print(mag.magnetic.x, 2);      Serial.print(',');
  Serial.print(mag.magnetic.y, 2);      Serial.print(',');
  Serial.println(mag.magnetic.z, 2);

  delay(100); // ~10 Hz output rate
}
