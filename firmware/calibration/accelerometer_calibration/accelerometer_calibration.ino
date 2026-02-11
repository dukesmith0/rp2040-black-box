/*
  RP2040 Black Box — Accelerometer 6-Position Calibration

  Guides the user through 6 orientations via serial prompts to compute
  accelerometer zero-g offsets. Results are saved to calibration.cfg on
  the SD card (readable by quat_datalogger) and printed to serial.

  Method:
    For each axis, the device is placed so that axis points straight up (+g)
    then straight down (-g). The average of the two means gives the zero-g
    offset for that axis:
      offset = (mean_positive + mean_negative) / 2

  Usage:
    1. Upload this sketch to the RP2040 Adalogger.
    2. Open the Serial Monitor at 115200 baud.
    3. Follow the prompts — place the device in the requested orientation,
       then press Enter (send any character) to start sampling.
    4. Hold the device perfectly still during each measurement.
    5. After all 6 positions, offsets are saved to calibration.cfg on SD
       and printed to serial.

  Hardware: LSM6DSOX on I2C, SD card on SPI1 (CS pin 23)
*/

#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_LSM6DSOX.h>

Adafruit_LSM6DSOX sox;

// SD card setup (same as quat_datalogger)
#define SD_CS_PIN 23
SdFat  SD;
SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
static bool sdReady = false;

static const char* CAL_FILENAME = "calibration.cfg";

static const int   CAL_SAMPLES   = 500;   // Samples per orientation
static const int   SAMPLE_DELAY  = 10;    // ms between samples (~100 Hz)
static const int   SETTLE_TIME   = 500;   // ms to let sensor settle after user confirms

// Storage for the 6-position averages (m/s²)
static float means[6][3]; // [orientation][axis]
// Order: X+, X-, Y+, Y-, Z+, Z-

static const char* orientationNames[] = {
  "X-axis UP   (X-arrow pointing UP)",
  "X-axis DOWN (X-arrow pointing DOWN)",
  "Y-axis UP   (Y-arrow pointing UP)",
  "Y-axis DOWN (Y-arrow pointing DOWN)",
  "Z-axis UP   (components facing UP)",
  "Z-axis DOWN (components facing DOWN)"
};

// Wait for user to send any character over serial
static void waitForUser() {
  // Flush any leftover input
  while (Serial.available()) Serial.read();

  Serial.println(">> Press Enter when ready...");
  while (!Serial.available()) delay(10);

  // Consume the input
  while (Serial.available()) Serial.read();
}

// Collect CAL_SAMPLES and store the mean accel for each axis
static void collectSamples(int orientationIndex) {
  // Let sensor settle
  delay(SETTLE_TIME);

  float sum[3] = { 0, 0, 0 };
  sensors_event_t accel, gyro, temp;

  Serial.print("   Sampling");
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sox.getEvent(&accel, &gyro, &temp);
    sum[0] += accel.acceleration.x;
    sum[1] += accel.acceleration.y;
    sum[2] += accel.acceleration.z;
    delay(SAMPLE_DELAY);

    // Progress dots
    if (i % 40 == 0) Serial.print(".");
  }
  Serial.println(" done!");

  means[orientationIndex][0] = sum[0] / CAL_SAMPLES;
  means[orientationIndex][1] = sum[1] / CAL_SAMPLES;
  means[orientationIndex][2] = sum[2] / CAL_SAMPLES;

  Serial.print("   Measured: ax=");
  Serial.print(means[orientationIndex][0], 4);
  Serial.print("  ay=");
  Serial.print(means[orientationIndex][1], 4);
  Serial.print("  az=");
  Serial.println(means[orientationIndex][2], 4);
  Serial.println();
}

// Read existing calibration.cfg into a buffer, preserving other lines.
// Writes back with accel_offset updated (or appended).
static void saveToSD(float ox, float oy, float oz) {
  if (!sdReady) {
    Serial.println("SD card not available, skipping file save.");
    return;
  }

  // Build the new accel_offset line
  char newLine[80];
  snprintf(newLine, sizeof(newLine), "accel_offset = %.4f, %.4f, %.4f", ox, oy, oz);

  // Read existing file content (if any), filtering out old accel_offset line
  char buf[2048];
  int bufLen = 0;

  if (SD.exists(CAL_FILENAME)) {
    FsFile cfg = SD.open(CAL_FILENAME, FILE_READ);
    if (cfg) {
      char line[256];
      while (cfg.available()) {
        int len = 0;
        while (cfg.available() && len < (int)sizeof(line) - 1) {
          char c = cfg.read();
          if (c == '\n') break;
          if (c != '\r') line[len++] = c;
        }
        line[len] = '\0';

        // Skip old accel_offset lines
        if (strncmp(line, "accel_offset", 12) == 0) {
          continue;
        }

        // Copy line to buffer
        if (bufLen + len + 1 < (int)sizeof(buf)) {
          memcpy(buf + bufLen, line, len);
          bufLen += len;
          buf[bufLen++] = '\n';
        }
      }
      cfg.close();
    }
  }

  // Append the new accel_offset line
  int nlLen = strlen(newLine);
  if (bufLen + nlLen + 1 < (int)sizeof(buf)) {
    memcpy(buf + bufLen, newLine, nlLen);
    bufLen += nlLen;
    buf[bufLen++] = '\n';
  }
  buf[bufLen] = '\0';

  // Write back
  SD.remove(CAL_FILENAME);
  FsFile cfg = SD.open(CAL_FILENAME, FILE_WRITE);
  if (cfg) {
    cfg.write(buf, bufLen);
    cfg.close();
    Serial.print("Saved to SD: ");
    Serial.println(CAL_FILENAME);
  } else {
    Serial.println("Failed to write calibration.cfg!");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  // --- Initialize LSM6DSOX ---
  if (!sox.begin_I2C()) {
    Serial.println("LSM6DSOX not found!");
    while (1) delay(10);
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // --- Initialize SD Card ---
  Serial.print("Initializing SD card...");
  delay(100);
  int retries = 0;
  while (!SD.begin(sdConfig)) {
    retries++;
    if (retries >= 5) {
      Serial.println("\nSD Card failed (results will only print to serial).");
      break;
    }
    Serial.print(".");
    delay(1000);
  }
  if (retries < 5) {
    Serial.println(" OK");
    sdReady = true;
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  Accelerometer 6-Position Calibration");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("You will be prompted to place the device in 6 orientations.");
  Serial.println("Hold the device PERFECTLY STILL during each measurement.");
  Serial.print("Each measurement takes ~");
  Serial.print((CAL_SAMPLES * SAMPLE_DELAY) / 1000);
  Serial.println(" seconds.");
  if (sdReady) {
    Serial.println("Results will be saved to calibration.cfg on the SD card.");
  }
  Serial.println();

  // --- Collect all 6 orientations ---
  for (int i = 0; i < 6; i++) {
    Serial.print("--- Position ");
    Serial.print(i + 1);
    Serial.print("/6: ");
    Serial.print(orientationNames[i]);
    Serial.println(" ---");
    waitForUser();
    collectSamples(i);
  }

  // --- Compute offsets ---
  // offset_x = (mean_x_when_x_up + mean_x_when_x_down) / 2
  // offset_y = (mean_y_when_y_up + mean_y_when_y_down) / 2
  // offset_z = (mean_z_when_z_up + mean_z_when_z_down) / 2
  float offset_x = (means[0][0] + means[1][0]) / 2.0f;
  float offset_y = (means[2][1] + means[3][1]) / 2.0f;
  float offset_z = (means[4][2] + means[5][2]) / 2.0f;

  // --- Save to SD card ---
  saveToSD(offset_x, offset_y, offset_z);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  CALIBRATION COMPLETE");
  Serial.println("==============================================");
  Serial.println();

  Serial.print("accel_offset = ");
  Serial.print(offset_x, 4);
  Serial.print(", ");
  Serial.print(offset_y, 4);
  Serial.print(", ");
  Serial.println(offset_z, 4);
  Serial.println();

  // Also print scale factors for reference (optional advanced use)
  float scale_x = (means[0][0] - means[1][0]) / (2.0f * 9.80665f);
  float scale_y = (means[2][1] - means[3][1]) / (2.0f * 9.80665f);
  float scale_z = (means[4][2] - means[5][2]) / (2.0f * 9.80665f);

  Serial.println("Scale factors (should be ~1.0, for reference only):");
  Serial.print("  X: "); Serial.println(scale_x, 4);
  Serial.print("  Y: "); Serial.println(scale_y, 4);
  Serial.print("  Z: "); Serial.println(scale_z, 4);
  Serial.println();

  Serial.println("Raw measurements summary:");
  for (int i = 0; i < 6; i++) {
    Serial.print("  ");
    Serial.print(orientationNames[i]);
    Serial.print(": (");
    Serial.print(means[i][0], 4);
    Serial.print(", ");
    Serial.print(means[i][1], 4);
    Serial.print(", ");
    Serial.print(means[i][2], 4);
    Serial.println(")");
  }
  Serial.println();
  Serial.println("Done! You can reset the board or upload a different sketch.");
}

void loop() {
  // Nothing to do — calibration runs once in setup()
  delay(1000);
}
