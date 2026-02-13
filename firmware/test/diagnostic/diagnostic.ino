/*
  RP2040 Black Box — Serial & Sensor Diagnostic

  Tests each I2C sensor individually with detailed bus diagnostics.
  Upload this sketch and open Serial Monitor at 115200 baud.

  I2C chain order: RP2040 → LSM6DSOX/LIS3MDL (accel/mag) → GPS → BMP390

  Expected I2C addresses:
    0x10  PA1010D GPS
    0x1C  LIS3MDL magnetometer  (alt: 0x1E)
    0x6A  LSM6DSOX accel/gyro   (alt: 0x6B)
    0x77  BMP390 barometer      (alt: 0x76)
*/

#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>
#include <Adafruit_AHRS.h>

// Hardware objects (same as datalogger)
Adafruit_BMP3XX   bmp;
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  lis3mdl;
Adafruit_GPS      GPS_sensor(&Wire);
Adafruit_Mahony   filter;

#define SD_CS_PIN 23
SdFat  SD;
SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// Expected addresses for each sensor
struct ExpectedDevice {
  uint8_t addr;
  const char* name;
};

ExpectedDevice expectedDevices[] = {
  { 0x10, "PA1010D GPS" },
  { 0x1C, "LIS3MDL magnetometer" },
  { 0x1E, "LIS3MDL magnetometer (alt)" },
  { 0x6A, "LSM6DSOX accel/gyro" },
  { 0x6B, "LSM6DSOX accel/gyro (alt)" },
  { 0x76, "BMP390 barometer (alt)" },
  { 0x77, "BMP390 barometer" },
};
const int NUM_EXPECTED = sizeof(expectedDevices) / sizeof(expectedDevices[0]);

const char* lookupDevice(uint8_t addr) {
  for (int i = 0; i < NUM_EXPECTED; i++) {
    if (expectedDevices[i].addr == addr) return expectedDevices[i].name;
  }
  return "unknown";
}

void i2cScan(const char* label) {
  Serial.print("     ");
  Serial.print(label);
  Serial.println(":");
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("       0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print("  ");
      Serial.println(lookupDevice(addr));
      count++;
    }
  }
  if (count == 0) Serial.println("       (no devices found)");
  Serial.print("       Total: ");
  Serial.println(count);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);

  Serial.println("========================================");
  Serial.println("  RP2040 Black Box — Diagnostic v2");
  Serial.println("========================================");
  Serial.println();
  Serial.println("  I2C chain: RP2040 -> IMU -> GPS -> Baro");
  Serial.println("  If GPS & Baro missing, check wiring");
  Serial.println("  between IMU and GPS boards.");
  Serial.println();

  // --- Test 1: I2C Bus at 400kHz ---
  Serial.println("[1] I2C scan at 400 kHz (normal speed):");
  Wire.begin();
  Wire.setClock(400000);
  i2cScan("400 kHz");
  Serial.println();

  // --- Test 2: I2C Bus at 100kHz (slower, more tolerant of wiring issues) ---
  Serial.println("[2] I2C scan at 100 kHz (slow, for bad wiring):");
  Wire.setClock(100000);
  i2cScan("100 kHz");
  Wire.setClock(400000);  // restore
  Serial.println();

  // --- Test 3: Check expected addresses directly ---
  Serial.println("[3] Direct address probe:");
  uint8_t checkAddrs[] = { 0x10, 0x1C, 0x1E, 0x6A, 0x6B, 0x76, 0x77 };
  for (int i = 0; i < 7; i++) {
    Wire.beginTransmission(checkAddrs[i]);
    uint8_t err = Wire.endTransmission();
    Serial.print("     0x");
    if (checkAddrs[i] < 16) Serial.print("0");
    Serial.print(checkAddrs[i], HEX);
    Serial.print(" (");
    Serial.print(lookupDevice(checkAddrs[i]));
    Serial.print("): ");
    switch (err) {
      case 0: Serial.println("ACK — present"); break;
      case 1: Serial.println("NACK — buffer overflow"); break;
      case 2: Serial.println("NACK — address not acknowledged"); break;
      case 3: Serial.println("NACK — data not acknowledged"); break;
      case 4: Serial.println("ERROR — bus error"); break;
      case 5: Serial.println("TIMEOUT — bus hung?"); break;
      default: Serial.print("ERROR code "); Serial.println(err); break;
    }
  }
  Serial.println();

  // --- Test 4: Sensor library init ---
  Serial.println("[4] Sensor init (library level):");

  Serial.print("     LSM6DSOX... ");
  if (sox.begin_I2C()) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }

  Serial.print("     LIS3MDL... ");
  if (lis3mdl.begin_I2C()) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }

  Serial.print("     GPS (0x10)... ");
  if (GPS_sensor.begin(0x10)) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }

  Serial.print("     BMP390 (0x77)... ");
  if (bmp.begin_I2C(0x77)) {
    Serial.println("PASS");
  } else {
    Serial.print("FAIL — trying 0x76... ");
    if (bmp.begin_I2C(0x76)) {
      Serial.println("PASS (at alt address)");
    } else {
      Serial.println("FAIL");
    }
  }
  Serial.println();

  // --- Test 5: SD Card ---
  Serial.print("[5] SD card (CS=23, SPI1)... ");
  if (SD.begin(sdConfig)) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }

  // --- Test 6: Mahony Filter ---
  Serial.print("[6] Mahony AHRS filter... ");
  filter.begin(200);
  Serial.println("PASS");

  // --- Summary ---
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Diagnostic complete.");
  Serial.println();
  Serial.println("  If GPS & Baro show NACK but IMU works:");
  Serial.println("  -> I2C bus broken after IMU in chain");
  Serial.println("  -> Check solder joints / wiring between");
  Serial.println("     IMU board and GPS board");
  Serial.println("  -> Check SDA/SCL continuity with meter");
  Serial.println();
  Serial.println("  If 100kHz finds more devices than 400kHz:");
  Serial.println("  -> Long wires or missing pullups");
  Serial.println("  -> Add 4.7k pullups on SDA/SCL");
  Serial.println("  -> Shorten I2C wires");
  Serial.println("========================================");
}

void loop() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500);
}
