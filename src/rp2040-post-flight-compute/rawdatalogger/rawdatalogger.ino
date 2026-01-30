#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GPS.h>
#include <RTClib.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_AHRS_NXPFusion.h> // For Quaternion math

// Sensor Objects
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;
Adafruit_BMP3XX bmp;
Adafruit_GPS GPS(&Wire); // Configured for I2C
RTC_PCF8523 rtc;
Adafruit_NXPFusion filter; // NXP Fusion EKF for Quaternions

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing I2C Stemma QT Chain...");

  // Initialize I2C Bus
  Wire.begin();

  // Initialize Sensors
  if (!lsm6ds.begin_I2C() || !lis3mdl.begin_I2C() || !bmp.begin_I2C() || !rtc.begin() || !GPS.begin(0x10)) {
    Serial.println("Check I2C connections! One or more sensors failed to initialize.");
    while (1);
  }

  // BMP390 Settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  // GPS I2C Settings
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); 

  filter.begin(100); // 100Hz filter rate

  // Header: Time, Quaternions(w,x,y,z), Accel, Gyro, Mag, Alt, GPS
  Serial.println("ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix");
}

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);
  bmp.performReading();

  // Update GPS over I2C
  GPS.read();
  if (GPS.newNMEAreceived()) { GPS.parse(GPS.lastNMEA()); }

  // Update Quaternion Filter (Units: rad/s and m/s^2)
  filter.update(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // Get Quaternion components
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  // Output CSV Data
  Serial.print(millis()); Serial.print(",");
  
  // Quaternions
  Serial.print(qw, 4); Serial.print(",");
  Serial.print(qx, 4); Serial.print(",");
  Serial.print(qy, 4); Serial.print(",");
  Serial.print(qz, 4); Serial.print(",");

  // Raw Accel (m/s^2)
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");

  // Raw Gyro (rad/s)
  Serial.print(gyro.gyro.x); Serial.print(",");
  Serial.print(gyro.gyro.y); Serial.print(",");
  Serial.print(gyro.gyro.z); Serial.print(",");

  // Raw Mag (uT)
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.print(",");

  // Altimeter & GPS Fix
  Serial.print(bmp.altitude); Serial.print(",");
  Serial.print((int)GPS.fix);
  
  Serial.println();
  
  delay(10); // Maintain ~100Hz loop
}