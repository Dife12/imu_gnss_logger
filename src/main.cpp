#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <stdarg.h>
#include <string.h>
#include <Wire.h>
#include <time.h>

#include <pcf8563.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SdFat.h>

#include "imu_gnss_logger_config.h"
#include "imu_gnss_logger_csv.h"
#include "imu_gnss_logger_types.h"

PCF8563_Class rtc;
SFE_UBLOX_GNSS_SERIAL gnss;
HardwareSerial gnssSerial(1);
BMI270 imu;
SdFat SD;
File logFile;

QueueHandle_t logQueue = nullptr;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
GnssFixState latestGnssFix = {};

bool rtcPresent = false;
bool rtcTimeAvailable = false;
bool rtcSyncedFromGnss = false;
bool gnssOnline = false;
bool sdHealthy = true;

uint64_t rtcBaseEpochMs = 0;
uint32_t rtcBaseMillis = 0;
uint32_t droppedLogRecords = 0;
uint32_t writtenLogLines = 0;
uint32_t imuReadErrors = 0;
uint32_t sdWriteErrors = 0;
uint32_t skippedGnssFixes = 0;

char activeLogFilename[16] = { 0 };

void debugPrint(const char *message);
void debugPrintf(const char *format, ...);
void fatalErrorLoop(const char *message);
bool probeI2cDevice(uint8_t address);
bool isDateTimeSane(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
uint64_t buildEpochMsUtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond);
bool createNextLogFilename(char *filename, size_t length);
bool initSdCardAndLogFile();
bool createMetadataFilename(const char *logFilename, char *metadataFilename, size_t length);
bool writeSessionMetadataFile();
bool initImu();
bool initGnssModule();
bool initRtc();
void refreshRtcBaseFromChip();
void updateRtcBaseFromEpoch(uint64_t epochMs);
void syncRtcFromGnss(uint64_t epochMs);
bool isGnssFixUsable(uint8_t fixType, uint8_t satellites);
bool readGnssPvtAndUpdateState();
bool appendCsvLineToLog(const char *line);
bool flushLogFile();
void printStartupSummary();
void imuTask(void *pvParameters);
void gnssTask(void *pvParameters);
void loggerTask(void *pvParameters);

void debugPrint(const char *message) {
  if (AppConfig::kEnableDebugPrint) {
    Serial.println(message);
  }
}

void debugPrintf(const char *format, ...) {
  if (!AppConfig::kEnableDebugPrint) {
    return;
  }

  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.println(buffer);
}

void fatalErrorLoop(const char *message) {
  Serial.print("FATAL: ");
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

bool probeI2cDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool isDateTimeSane(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  return year >= 2020 && year <= 2099 &&
         month >= 1 && month <= 12 &&
         day >= 1 && day <= 31 &&
         hour <= 23 &&
         minute <= 59 &&
         second <= 60;
}

uint64_t buildEpochMsUtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond) {
  if (!isDateTimeSane(year, month, day, hour, minute, second)) {
    return 0;
  }

  struct tm timeInfo = {};
  timeInfo.tm_year = static_cast<int>(year) - 1900;
  timeInfo.tm_mon = static_cast<int>(month) - 1;
  timeInfo.tm_mday = day;
  timeInfo.tm_hour = hour;
  timeInfo.tm_min = minute;
  timeInfo.tm_sec = second;

  time_t epochSeconds = mktime(&timeInfo);
  if (epochSeconds < 0) {
    return 0;
  }

  return (static_cast<uint64_t>(epochSeconds) * 1000ULL) + millisecond;
}

bool createNextLogFilename(char *filename, size_t length) {
  for (uint16_t index = 1; index <= 9999; ++index) {
    snprintf(filename, length, "LOG_%04u.CSV", index);
    if (!SD.exists(filename)) {
      return true;
    }
  }
  return false;
}

bool createMetadataFilename(const char *logFilename, char *metadataFilename, size_t length) {
  const char *extension = strrchr(logFilename, '.');
  if (extension == nullptr) {
    return false;
  }

  size_t stemLength = static_cast<size_t>(extension - logFilename);
  if (stemLength + 5 >= length) {
    return false;
  }

  memcpy(metadataFilename, logFilename, stemLength);
  metadataFilename[stemLength] = '\0';
  strncat(metadataFilename, ".TXT", length - strlen(metadataFilename) - 1);
  return true;
}

bool initSdCardAndLogFile() {
  debugPrintf("Initializing SD card on CS pin %u", AppConfig::kSdCsPin);
  if (!SD.begin(AppConfig::kSdCsPin)) {
    return false;
  }

  if (!createNextLogFilename(activeLogFilename, sizeof(activeLogFilename))) {
    debugPrint("No available LOG_XXXX.CSV filename");
    return false;
  }

  logFile = SD.open(activeLogFilename, O_WRITE | O_CREAT | O_AT_END);
  if (!logFile) {
    debugPrintf("Failed to open %s", activeLogFilename);
    return false;
  }

  logFile.println(csvHeader());
  if (!logFile.sync()) {
    debugPrintf("Initial sync failed for %s", activeLogFilename);
    return false;
  }
  debugPrintf("Logging to %s", activeLogFilename);
  return true;
}

bool writeSessionMetadataFile() {
  SessionMetadata metadata = {};
  strncpy(metadata.logFilename, activeLogFilename, sizeof(metadata.logFilename) - 1);
  if (!createMetadataFilename(activeLogFilename, metadata.metadataFilename, sizeof(metadata.metadataFilename))) {
    debugPrint("Failed to create metadata filename");
    return false;
  }

  metadata.imuSampleRateHz = AppConfig::kImuSampleRateHz;
  metadata.gnssMeasurementPeriodMs = AppConfig::kGnssMeasurementPeriodMs;
  metadata.gnssBaudrate = AppConfig::kGnssBaudrate;
  metadata.sdCsPin = AppConfig::kSdCsPin;
  metadata.gnssRxPin = AppConfig::kGnssRxPin;
  metadata.gnssTxPin = AppConfig::kGnssTxPin;
  metadata.rtcEnabled = AppConfig::kEnableRtc;
  metadata.rtcDetected = rtcPresent;
  metadata.gnssOnlineAtBoot = gnssOnline;

  File metadataFile = SD.open(metadata.metadataFilename, O_WRITE | O_CREAT | O_TRUNC);
  if (!metadataFile) {
    debugPrintf("Failed to open metadata file %s", metadata.metadataFilename);
    return false;
  }

  metadataFile.println("imu-gnss-logger session metadata");
  metadataFile.printf("log_filename=%s\r\n", metadata.logFilename);
  metadataFile.printf("imu_sample_rate_hz=%u\r\n", metadata.imuSampleRateHz);
  metadataFile.printf("gnss_measurement_period_ms=%lu\r\n", static_cast<unsigned long>(metadata.gnssMeasurementPeriodMs));
  metadataFile.printf("gnss_baudrate=%lu\r\n", static_cast<unsigned long>(metadata.gnssBaudrate));
  metadataFile.printf("sd_cs_pin=%u\r\n", metadata.sdCsPin);
  metadataFile.printf("gnss_rx_pin=%u\r\n", metadata.gnssRxPin);
  metadataFile.printf("gnss_tx_pin=%u\r\n", metadata.gnssTxPin);
  metadataFile.printf("rtc_enabled=%s\r\n", metadata.rtcEnabled ? "true" : "false");
  metadataFile.printf("rtc_detected=%s\r\n", metadata.rtcDetected ? "true" : "false");
  metadataFile.printf("gnss_online_at_boot=%s\r\n", metadata.gnssOnlineAtBoot ? "true" : "false");
  metadataFile.printf("min_gnss_fix_type=%u\r\n", AppConfig::kMinGnssFixType);
  metadataFile.printf("min_gnss_satellites=%u\r\n", AppConfig::kMinGnssSatellites);
  metadataFile.printf("csv_header=%s\r\n", csvHeader());

  bool synced = metadataFile.sync();
  metadataFile.close();
  return synced;
}

bool initImu() {
  int imuResult = imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire);
  if (imuResult != BMI2_OK) {
    debugPrintf("BMI270 init failed: %d", imuResult);
    return false;
  }

  imu.setAccelODR(BMI2_ACC_ODR_200HZ);
  imu.setGyroODR(BMI2_GYR_ODR_200HZ);
  debugPrint("BMI270 connected");
  return true;
}

bool initGnssModule() {
  gnssSerial.begin(AppConfig::kGnssBaudrate, SERIAL_8N1, AppConfig::kGnssRxPin, AppConfig::kGnssTxPin);
  if (!gnss.begin(gnssSerial)) {
    debugPrint("GNSS module not detected");
    return false;
  }

  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  gnss.setMeasurementRate(AppConfig::kGnssMeasurementPeriodMs);

  if (AppConfig::kEnableGnssL5Tweaks) {
    gnss.setVal8(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1);
    gnss.setGPSL5HealthOverride(true);
    gnss.setLNAMode(SFE_UBLOX_LNA_MODE_NORMAL);
  }

  debugPrint("GNSS module configured");
  return true;
}

bool initRtc() {
  if (!AppConfig::kEnableRtc) {
    return false;
  }

  if (!probeI2cDevice(AppConfig::kRtcI2cAddress)) {
    debugPrint("RTC not detected on I2C bus");
    return false;
  }

  if (!rtc.begin(Wire, AppConfig::kRtcI2cAddress)) {
    debugPrint("RTC begin failed");
    return false;
  }

  rtcPresent = true;
  refreshRtcBaseFromChip();
  debugPrint("RTC detected");
  return true;
}

void refreshRtcBaseFromChip() {
  if (!rtcPresent) {
    return;
  }

  if (!rtc.isVaild()) {
    return;
  }

  RTC_Date rtcTime = rtc.getDateTime();
  uint64_t epochMs = buildEpochMsUtc(rtcTime.year, rtcTime.month, rtcTime.day, rtcTime.hour, rtcTime.minute, rtcTime.second, 0);
  if (epochMs == 0) {
    return;
  }

  portENTER_CRITICAL(&stateMux);
  rtcBaseEpochMs = epochMs;
  rtcBaseMillis = millis();
  rtcTimeAvailable = true;
  portEXIT_CRITICAL(&stateMux);
}

void updateRtcBaseFromEpoch(uint64_t epochMs) {
  portENTER_CRITICAL(&stateMux);
  rtcBaseEpochMs = epochMs;
  rtcBaseMillis = millis();
  rtcTimeAvailable = true;
  portEXIT_CRITICAL(&stateMux);
}

void syncRtcFromGnss(uint64_t epochMs) {
  if (!rtcPresent || epochMs == 0) {
    return;
  }

  time_t epochSeconds = static_cast<time_t>(epochMs / 1000ULL);
  struct tm utcTime = {};
  gmtime_r(&epochSeconds, &utcTime);

  rtc.setDateTime(static_cast<uint16_t>(utcTime.tm_year + 1900),
                  static_cast<uint8_t>(utcTime.tm_mon + 1),
                  static_cast<uint8_t>(utcTime.tm_mday),
                  static_cast<uint8_t>(utcTime.tm_hour),
                  static_cast<uint8_t>(utcTime.tm_min),
                  static_cast<uint8_t>(utcTime.tm_sec));

  updateRtcBaseFromEpoch(epochMs);
  rtcSyncedFromGnss = true;
  debugPrint("RTC synchronized from GNSS UTC");
}

bool isGnssFixUsable(uint8_t fixType, uint8_t satellites) {
  return fixType >= AppConfig::kMinGnssFixType &&
         satellites >= AppConfig::kMinGnssSatellites;
}

bool readGnssPvtAndUpdateState() {
  if (!gnss.getPVT()) {
    return false;
  }

  uint16_t year = gnss.getYear();
  uint8_t month = gnss.getMonth();
  uint8_t day = gnss.getDay();
  uint8_t hour = gnss.getHour();
  uint8_t minute = gnss.getMinute();
  uint8_t second = gnss.getSecond();
  uint16_t millisecond = gnss.getMillisecond();

  bool hasUtc = isDateTimeSane(year, month, day, hour, minute, second);
  uint64_t epochMs = hasUtc ? buildEpochMsUtc(year, month, day, hour, minute, second, millisecond) : 0;

  if (hasUtc && rtcPresent && !rtcSyncedFromGnss) {
    syncRtcFromGnss(epochMs);
  }

  uint8_t fixQuality = gnss.getFixType();
  uint8_t satellites = gnss.getSIV();
  if (!isGnssFixUsable(fixQuality, satellites)) {
    portENTER_CRITICAL(&stateMux);
    latestGnssFix.hasValidFix = false;
    latestGnssFix.hasUtc = false;
    portEXIT_CRITICAL(&stateMux);
    skippedGnssFixes++;
    return true;
  }

  portENTER_CRITICAL(&stateMux);
  latestGnssFix.hasValidFix = true;
  latestGnssFix.fixId++;
  latestGnssFix.lastUpdateMs = millis();
  latestGnssFix.hasUtc = hasUtc;
  latestGnssFix.utcEpochMs = epochMs;
  latestGnssFix.latitudeE7 = gnss.getLatitude();
  latestGnssFix.longitudeE7 = gnss.getLongitude();
  latestGnssFix.altitudeMm = gnss.getAltitude();
  latestGnssFix.speedMmPerSec = gnss.getGroundSpeed();
  latestGnssFix.headingDegE5 = gnss.getHeading();
  latestGnssFix.fixQuality = fixQuality;
  latestGnssFix.satellites = satellites;
  portEXIT_CRITICAL(&stateMux);

  return true;
}

bool appendCsvLineToLog(const char *line) {
  if (!logFile) {
    sdWriteErrors++;
    sdHealthy = false;
    return false;
  }

  size_t bytesWritten = logFile.println(line);
  if (bytesWritten == 0) {
    sdWriteErrors++;
    sdHealthy = false;
    return false;
  }

  sdHealthy = true;
  return true;
}

bool flushLogFile() {
  if (!logFile) {
    sdWriteErrors++;
    sdHealthy = false;
    return false;
  }

  bool synced = logFile.sync();
  if (!synced) {
    sdWriteErrors++;
    sdHealthy = false;
  } else {
    sdHealthy = true;
  }
  return synced;
}

void printStartupSummary() {
  debugPrint("imu-gnss-logger startup summary");
  debugPrintf("Board profile: ESP32-S3 / XIAO-compatible");
  debugPrintf("IMU sample rate: %u Hz", AppConfig::kImuSampleRateHz);
  debugPrintf("GNSS measurement period: %lu ms", static_cast<unsigned long>(AppConfig::kGnssMeasurementPeriodMs));
  debugPrintf("Log flush interval: %lu ms", static_cast<unsigned long>(AppConfig::kLogFlushIntervalMs));
  debugPrintf("Active log file: %s", activeLogFilename);
  debugPrintf("RTC present: %s", rtcPresent ? "yes" : "no");
  debugPrintf("GNSS online at boot: %s", gnssOnline ? "yes" : "no");
}

void imuTask(void *pvParameters) {
  const TickType_t periodTicks = pdMS_TO_TICKS(1000 / AppConfig::kImuSampleRateHz);
  TickType_t lastWakeTime = xTaskGetTickCount();
  uint32_t imuSampleId = 0;
  uint32_t lastSampleUs = micros();
  uint32_t lastImuErrorPrintMs = 0;

  while (true) {
    int8_t imuResult = imu.getSensorData();
    if (imuResult != BMI2_OK) {
      imuReadErrors++;
      uint32_t nowMs = millis();
      if (AppConfig::kEnableDebugPrint &&
          (lastImuErrorPrintMs == 0 || nowMs - lastImuErrorPrintMs >= AppConfig::kImuErrorPrintIntervalMs)) {
        debugPrintf("IMU read failed: %d (total=%lu)", imuResult, static_cast<unsigned long>(imuReadErrors));
        lastImuErrorPrintMs = nowMs;
      }
      vTaskDelayUntil(&lastWakeTime, periodTicks);
      continue;
    }

    uint32_t nowMs = millis();
    uint32_t nowUs = micros();
    float imuDtMs = (imuSampleId == 0) ? (1000.0f / AppConfig::kImuSampleRateHz) : ((nowUs - lastSampleUs) / 1000.0f);
    lastSampleUs = nowUs;
    imuSampleId++;

    GnssFixState gnssSnapshot = {};
    bool localRtcTimeAvailable = false;
    uint64_t localRtcBaseEpochMs = 0;
    uint32_t localRtcBaseMillis = 0;

    portENTER_CRITICAL(&stateMux);
    gnssSnapshot = latestGnssFix;
    localRtcTimeAvailable = rtcTimeAvailable;
    localRtcBaseEpochMs = rtcBaseEpochMs;
    localRtcBaseMillis = rtcBaseMillis;
    portEXIT_CRITICAL(&stateMux);

    LoggerRecord record = {};
    record.timestampMs = nowMs;
    record.imuSampleId = imuSampleId;
    record.ax = imu.data.accelX;
    record.ay = imu.data.accelY;
    record.az = imu.data.accelZ;
    record.gx = imu.data.gyroX;
    record.gy = imu.data.gyroY;
    record.gz = imu.data.gyroZ;
    record.imuDtMs = imuDtMs;

    bool useGnssFix = gnssSnapshot.hasValidFix &&
                      ((nowMs - gnssSnapshot.lastUpdateMs) <= AppConfig::kMaxGnssReuseAgeMs);

    if (useGnssFix) {
      // Each IMU row is stamped with the latest valid GNSS fix and its age.
      record.hasGnss = true;
      record.gnssFixId = gnssSnapshot.fixId;
      record.gnssAgeMs = nowMs - gnssSnapshot.lastUpdateMs;
      record.latitudeE7 = gnssSnapshot.latitudeE7;
      record.longitudeE7 = gnssSnapshot.longitudeE7;
      record.altitudeMm = gnssSnapshot.altitudeMm;
      record.speedMmPerSec = gnssSnapshot.speedMmPerSec;
      record.headingDegE5 = gnssSnapshot.headingDegE5;
      record.gnssFixQuality = gnssSnapshot.fixQuality;
      record.gnssSatellites = gnssSnapshot.satellites;

      if (gnssSnapshot.hasUtc && gnssSnapshot.utcEpochMs != 0) {
        record.hasUtc = true;
        record.utcEpochMs = gnssSnapshot.utcEpochMs + static_cast<uint64_t>(record.gnssAgeMs);
      } else if (localRtcTimeAvailable) {
        record.hasUtc = true;
        record.utcEpochMs = localRtcBaseEpochMs + static_cast<uint64_t>(nowMs - localRtcBaseMillis);
      }
    } else if (localRtcTimeAvailable) {
      record.hasUtc = true;
      record.utcEpochMs = localRtcBaseEpochMs + static_cast<uint64_t>(nowMs - localRtcBaseMillis);
    }

    if (xQueueSend(logQueue, &record, 0) != pdTRUE) {
      droppedLogRecords++;
    }

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void gnssTask(void *pvParameters) {
  uint32_t lastInitAttemptMs = 0;
  uint32_t lastRtcRefreshMs = millis();

  while (true) {
    if (!gnssOnline) {
      uint32_t nowMs = millis();
      if (nowMs - lastInitAttemptMs >= AppConfig::kGnssInitRetryMs) {
        lastInitAttemptMs = nowMs;
        gnssOnline = initGnssModule();
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    readGnssPvtAndUpdateState();

    uint32_t nowMs = millis();
    if (rtcPresent && !rtcSyncedFromGnss && nowMs - lastRtcRefreshMs >= AppConfig::kRtcRefreshIntervalMs) {
      refreshRtcBaseFromChip();
      lastRtcRefreshMs = nowMs;
    }

    vTaskDelay(pdMS_TO_TICKS(AppConfig::kGnssPollPeriodMs));
  }
}

void loggerTask(void *pvParameters) {
  char csvLine[AppConfig::kCsvLineBufferSize];
  uint32_t lastFlushMs = millis();
  uint32_t lastStatusMs = millis();
  uint32_t lastSdErrorPrintMs = 0;
  uint32_t linesSinceFlush = 0;
  LoggerRecord record = {};

  while (true) {
    if (xQueueReceive(logQueue, &record, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (formatCsvRecord(record, csvLine, sizeof(csvLine))) {
        // Flush in batches to keep SD latency away from the IMU sampling path.
        if (appendCsvLineToLog(csvLine)) {
          writtenLogLines++;
          linesSinceFlush++;
        }
      }
    }

    uint32_t nowMs = millis();
    if (linesSinceFlush > 0 &&
        (nowMs - lastFlushMs >= AppConfig::kLogFlushIntervalMs ||
         linesSinceFlush >= AppConfig::kLogFlushLineInterval)) {
      if (flushLogFile()) {
        lastFlushMs = nowMs;
        linesSinceFlush = 0;
      }
    }

    if (!sdHealthy &&
        AppConfig::kEnableDebugPrint &&
        (lastSdErrorPrintMs == 0 || nowMs - lastSdErrorPrintMs >= AppConfig::kSdErrorPrintIntervalMs)) {
      debugPrintf("SD write/sync failure detected (errors=%lu)", static_cast<unsigned long>(sdWriteErrors));
      lastSdErrorPrintMs = nowMs;
    }

    if (AppConfig::kEnableDebugPrint && nowMs - lastStatusMs >= AppConfig::kStatusPrintIntervalMs) {
      uint32_t latestFixId = 0;
      portENTER_CRITICAL(&stateMux);
      latestFixId = latestGnssFix.fixId;
      portEXIT_CRITICAL(&stateMux);

      debugPrintf("queue=%u dropped=%lu written=%lu latest_fix=%lu",
                  static_cast<unsigned int>(uxQueueMessagesWaiting(logQueue)),
                  static_cast<unsigned long>(droppedLogRecords),
                  static_cast<unsigned long>(writtenLogLines),
                  static_cast<unsigned long>(latestFixId));
      debugPrintf("imu_errors=%lu sd_errors=%lu skipped_gnss=%lu sd_ok=%s",
                  static_cast<unsigned long>(imuReadErrors),
                  static_cast<unsigned long>(sdWriteErrors),
                  static_cast<unsigned long>(skippedGnssFixes),
                  sdHealthy ? "true" : "false");
      lastStatusMs = nowMs;
    }
  }
}

void setup() {
  Serial.begin(AppConfig::kDebugBaudrate);
  delay(200);

  setenv("TZ", "UTC0", 1);
  tzset();

  if (AppConfig::kUseBoardDefaultI2cPins) {
    Wire.begin();
  } else {
    Wire.begin(AppConfig::kI2cSdaPin, AppConfig::kI2cSclPin);
  }

  if (!initSdCardAndLogFile()) {
    fatalErrorLoop("SD card initialization or CSV file creation failed");
  }

  if (!initImu()) {
    fatalErrorLoop("BMI270 initialization failed");
  }

  gnssOnline = initGnssModule();
  rtcPresent = initRtc();

  if (gnssOnline) {
    uint32_t startMs = millis();
    while (millis() - startMs < AppConfig::kGnssBootPrimeTimeoutMs) {
      if (readGnssPvtAndUpdateState()) {
        break;
      }
      delay(20);
    }
  }

  if (!writeSessionMetadataFile()) {
    debugPrint("Failed to write session metadata file");
  }

  logQueue = xQueueCreate(AppConfig::kLoggerQueueLength, sizeof(LoggerRecord));
  if (logQueue == nullptr) {
    fatalErrorLoop("Failed to create logger queue");
  }

  printStartupSummary();

  xTaskCreateUniversal(imuTask, "imuTask", 4096, nullptr, 3, nullptr, 1);
  xTaskCreateUniversal(gnssTask, "gnssTask", 6144, nullptr, 2, nullptr, 1);
  xTaskCreateUniversal(loggerTask, "loggerTask", 6144, nullptr, 1, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
