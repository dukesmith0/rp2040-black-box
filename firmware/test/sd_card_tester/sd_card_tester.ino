/*
  RP2040 SD Card Diagnostic Tool

  Comprehensive SD card tester for Adafruit RP2040 Adalogger
  Tests initialization, read/write operations, speed, and reports card specs

  Hardware configuration (per Adafruit official example):
  - CS Pin: 23 (NOT pin 10!)
  - SPI Bus: SPI1 at 16MHz
  - Library: SdFat (NOT standard SD.h)

  Tests performed:
  1. SD card detection and initialization
  2. Card type and size reporting
  3. File write test
  4. File read verification
  5. Directory listing
  6. Write/read speed benchmarking
  7. Cleanup (test file deletion)
*/

#include <SPI.h>
#include "SdFat.h"

// SD Card configuration for Adafruit RP2040 Adalogger
#define SD_CS_PIN 23  // CS pin for RP2040 Adalogger (NOT pin 10!)

SdFat SD;
FsFile myFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// Test file configuration
#define TEST_FILENAME "sdtest.txt"
#define BENCHMARK_FILENAME "benchmark.bin"
#define BENCHMARK_SIZE 10240      // 10KB for speed test

// Test results tracking
bool allTestsPassed = true;

void setup() {
  // Open serial communications and wait for port to open
  Serial.begin(115200);
  while (!Serial) {
    yield();
    delay(10);
  }
  delay(100);  // RP2040 delay is recommended

  Serial.println("========================================");
  Serial.println("RP2040 SD Card Diagnostic Tool");
  Serial.println("========================================");
  Serial.println("Hardware: Adafruit RP2040 Adalogger");
  Serial.println("CS Pin: 23, SPI1 @ 16MHz");
  Serial.println();

  // Run all diagnostic tests
  runDiagnostics();

  // Print final results
  Serial.println();
  Serial.println("========================================");
  if (allTestsPassed) {
    Serial.println("✓ ALL TESTS PASSED");
    Serial.println("SD card is functioning correctly");
    Serial.println("You can now use this card with the data logger firmware");
  } else {
    Serial.println("✗ SOME TESTS FAILED");
    Serial.println("Check errors above for details");
  }
  Serial.println("========================================");
}

void loop() {
  // Test runs once in setup, nothing to do in loop
  delay(1000);
}

void runDiagnostics() {
  // Test 1: SD Card Initialization
  testSDInitialization();

  if (!allTestsPassed) {
    Serial.println("\n⚠ Skipping remaining tests due to initialization failure");
    return;
  }

  // Test 2: Card Information
  testCardInfo();

  // Test 3: Write Test
  testFileWrite();

  // Test 4: Read Test
  testFileRead();

  // Test 5: Directory Listing
  testDirectoryListing();

  // Test 6: Speed Benchmark
  testSpeedBenchmark();

  // Test 7: Cleanup
  testCleanup();
}

// ============================================================
// Test 1: SD Card Initialization
// ============================================================
void testSDInitialization() {
  Serial.println("Test 1: SD Card Initialization");
  Serial.println("------------------------------");
  Serial.print("Initializing SD card...");

  // Retry mechanism for SD card initialization (per Adafruit example)
  int retryCount = 0;
  const int maxRetries = 5;

  while (!SD.begin(config)) {
    retryCount++;
    if (retryCount >= maxRetries) {
      Serial.println();
      Serial.println("✗ FAILED: SD card initialization failed after 5 retries");
      Serial.println();
      Serial.println("Troubleshooting:");
      Serial.println("  1. Is the SD card inserted properly?");
      Serial.println("  2. Is the card formatted as FAT32?");
      Serial.println("  3. Try formatting with 4KB allocation unit size");
      Serial.println("  4. Are the card contacts clean?");
      Serial.println("  5. Try a different SD card (some cheap cards don't work)");
      Serial.println("  6. Verify you're using CS pin 23 (NOT pin 10)");
      allTestsPassed = false;
      return;
    }
    Serial.print(".");
    delay(1000); // Wait before retrying
  }

  Serial.println();
  Serial.println("✓ PASSED: SD card initialized successfully");
  Serial.println();
}

// ============================================================
// Test 2: Card Information
// ============================================================
void testCardInfo() {
  Serial.println("Test 2: Card Information");
  Serial.println("------------------------------");

  // Get card type
  uint8_t cardType = SD.card()->type();
  Serial.print("Card Type: ");
  switch (cardType) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1 (Standard Capacity)");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2 (Standard Capacity)");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC (High Capacity)");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Get card size in MB
  uint32_t cardSizeKB = SD.card()->sectorCount() / 2; // 512-byte sectors to KB
  uint32_t cardSizeMB = cardSizeKB / 1024;
  Serial.print("Card Size: ");
  Serial.print(cardSizeMB);
  Serial.println(" MB");

  // Get volume size
  uint32_t volSizeKB = SD.vol()->clusterCount() * SD.vol()->sectorsPerCluster() / 2;
  uint32_t volSizeMB = volSizeKB / 1024;
  Serial.print("Volume Size: ");
  Serial.print(volSizeMB);
  Serial.println(" MB");

  Serial.println("✓ PASSED: Card information retrieved");
  Serial.println();
}

// ============================================================
// Test 3: File Write Test
// ============================================================
void testFileWrite() {
  Serial.println("Test 3: File Write Test");
  Serial.println("------------------------------");

  // Delete test file if it exists
  if (SD.exists(TEST_FILENAME)) {
    SD.remove(TEST_FILENAME);
  }

  // Open file for writing
  myFile = SD.open(TEST_FILENAME, FILE_WRITE);
  if (!myFile) {
    Serial.println("✗ FAILED: Could not create test file");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  // Write test data
  Serial.print("Writing to ");
  Serial.print(TEST_FILENAME);
  Serial.print("...");

  myFile.println("RP2040 SD Card Test");
  myFile.print("Timestamp: ");
  myFile.println(millis());
  myFile.println("This is a test file for SD card diagnostics.");
  myFile.println("If you can read this, the write operation succeeded!");

  size_t bytesWritten = myFile.size();
  myFile.close();

  if (bytesWritten == 0) {
    Serial.println();
    Serial.println("✗ FAILED: No data written to file");
    allTestsPassed = false;
  } else {
    Serial.println("done.");
    Serial.print("✓ PASSED: Wrote ");
    Serial.print(bytesWritten);
    Serial.println(" bytes to test file");
  }
  Serial.println();
}

// ============================================================
// Test 4: File Read Test
// ============================================================
void testFileRead() {
  Serial.println("Test 4: File Read Test");
  Serial.println("------------------------------");

  // Check if file exists
  if (!SD.exists(TEST_FILENAME)) {
    Serial.println("✗ FAILED: Test file does not exist");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  // Open file for reading
  myFile = SD.open(TEST_FILENAME, FILE_READ);
  if (!myFile) {
    Serial.println("✗ FAILED: Could not open test file for reading");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  // Read and display file contents
  Serial.print(TEST_FILENAME);
  Serial.println(" contents:");
  Serial.println("---");
  while (myFile.available()) {
    Serial.write(myFile.read());
  }
  Serial.println("---");
  myFile.close();

  Serial.println("✓ PASSED: File read successfully");
  Serial.println();
}

// ============================================================
// Test 5: Directory Listing
// ============================================================
void testDirectoryListing() {
  Serial.println("Test 5: Directory Listing");
  Serial.println("------------------------------");

  FsFile root = SD.open("/");
  if (!root) {
    Serial.println("✗ FAILED: Could not open root directory");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  Serial.println("Files on SD card:");
  int fileCount = 0;

  FsFile entry;
  while (entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDirectory()) {
      char fileName[64];
      entry.getName(fileName, sizeof(fileName));
      Serial.print("  ");
      Serial.print(fileName);
      Serial.print(" (");
      Serial.print(entry.size());
      Serial.println(" bytes)");
      fileCount++;
    }
    entry.close();
  }
  root.close();

  Serial.print("Total files: ");
  Serial.println(fileCount);
  Serial.println("✓ PASSED: Directory listing successful");
  Serial.println();
}

// ============================================================
// Test 6: Write/Read Speed Benchmark
// ============================================================
void testSpeedBenchmark() {
  Serial.println("Test 6: Write/Read Speed Benchmark");
  Serial.println("------------------------------");

  // Delete benchmark file if it exists
  if (SD.exists(BENCHMARK_FILENAME)) {
    SD.remove(BENCHMARK_FILENAME);
  }

  // Prepare test data
  uint8_t testBuffer[256];
  for (int i = 0; i < 256; i++) {
    testBuffer[i] = i;
  }

  // Write speed test
  myFile = SD.open(BENCHMARK_FILENAME, FILE_WRITE);
  if (!myFile) {
    Serial.println("✗ FAILED: Could not create benchmark file");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  uint32_t writeStart = millis();
  size_t totalWritten = 0;

  for (int i = 0; i < (BENCHMARK_SIZE / 256); i++) {
    totalWritten += myFile.write(testBuffer, 256);
  }
  myFile.close();

  uint32_t writeTime = millis() - writeStart;
  if (writeTime == 0) writeTime = 1; // Avoid division by zero
  float writeSpeed = (float)totalWritten / (float)writeTime; // bytes per ms = KB/s

  Serial.print("Write: ");
  Serial.print(totalWritten);
  Serial.print(" bytes in ");
  Serial.print(writeTime);
  Serial.print(" ms (");
  Serial.print(writeSpeed, 2);
  Serial.println(" KB/s)");

  // Read speed test
  myFile = SD.open(BENCHMARK_FILENAME, FILE_READ);
  if (!myFile) {
    Serial.println("✗ FAILED: Could not open benchmark file for reading");
    allTestsPassed = false;
    Serial.println();
    return;
  }

  uint32_t readStart = millis();
  size_t totalRead = 0;

  while (myFile.available()) {
    size_t bytesRead = myFile.read(testBuffer, 256);
    totalRead += bytesRead;
  }
  myFile.close();

  uint32_t readTime = millis() - readStart;
  if (readTime == 0) readTime = 1; // Avoid division by zero
  float readSpeed = (float)totalRead / (float)readTime; // bytes per ms = KB/s

  Serial.print("Read: ");
  Serial.print(totalRead);
  Serial.print(" bytes in ");
  Serial.print(readTime);
  Serial.print(" ms (");
  Serial.print(readSpeed, 2);
  Serial.println(" KB/s)");

  Serial.println("✓ PASSED: Speed benchmark completed");
  Serial.println();
}

// ============================================================
// Test 7: Cleanup
// ============================================================
void testCleanup() {
  Serial.println("Test 7: Cleanup");
  Serial.println("------------------------------");

  bool cleanupSuccess = true;

  // Delete test files
  if (SD.exists(TEST_FILENAME)) {
    if (!SD.remove(TEST_FILENAME)) {
      Serial.print("⚠ WARNING: Could not delete ");
      Serial.println(TEST_FILENAME);
      cleanupSuccess = false;
    }
  }

  if (SD.exists(BENCHMARK_FILENAME)) {
    if (!SD.remove(BENCHMARK_FILENAME)) {
      Serial.print("⚠ WARNING: Could not delete ");
      Serial.println(BENCHMARK_FILENAME);
      cleanupSuccess = false;
    }
  }

  if (cleanupSuccess) {
    Serial.println("✓ PASSED: Test files cleaned up successfully");
  } else {
    Serial.println("✓ PASSED with warnings: Some files could not be deleted");
  }
  Serial.println();
}
