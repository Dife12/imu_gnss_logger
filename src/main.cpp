#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <stdarg.h>
#include <Wire.h>
#include <time.h>

#include <PCF8563.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SdFat.h>

#include "imu_gnss_logger_config.h"
#include "imu_gnss_logger_csv.h"
#include "imu_gnss_logger_types.h"

PCF8563 rtc;
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

uint64_t rtcBaseEpochMs = 0;
uint32_t rtcBaseMillis = 0;
uint32_t droppedLogRecords = 0;
uint32_t writtenLogLines = 0;

char activeLogFilename[16] = { 0 };

void debugPrint(const char *message);
void debugPrintf(const char *format, ...);
void fatalErrorLoop(const char *message);
bool probeI2cDevice(uint8_t address);
bool isDateTimeSane(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
uint64_t buildEpochMsUtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond);
bool createNextLogFilename(char *filename, size_t length);
bool initSdCardAndLogFile();
bool initImu();
bool initGnssModule();
bool initRtc();
void refreshRtcBaseFromChip();
void updateRtcBaseFromEpoch(uint64_t epochMs);
void syncRtcFromGnss(uint64_t epochMs);
bool readGnssPvtAndUpdateState();
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
  logFile.flush();
  debugPrintf("Logging to %s", activeLogFilename);
  return true;
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

  rtc.init();
  rtcPresent = true;
  refreshRtcBaseFromChip();
  debugPrint("RTC detected");
  return true;
}

void refreshRtcBaseFromChip() {
  if (!rtcPresent) {
    return;
  }

  Time rtcTime = rtc.getTime();
  uint16_t year = static_cast<uint16_t>(2000 + rtcTime.year);
  uint64_t epochMs = buildEpochMsUtc(year, rtcTime.month, rtcTime.day, rtcTime.hour, rtcTime.minute, rtcTime.second, 0);
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

  rtc.stopClock();
  rtc.setYear((utcTime.tm_year + 1900) % 100);
  rtc.setMonth(utcTime.tm_mon + 1);
  rtc.setDay(utcTime.tm_mday);
  rtc.setHour(utcTime.tm_hour);
  rtc.setMinut(utcTime.tm_min);
  rtc.setSecond(utcTime.tm_sec);
  rtc.startClock();

  updateRtcBaseFromEpoch(epochMs);
  rtcSyncedFromGnss = true;
  debugPrint("RTC synchronized from GNSS UTC");
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

  bool hasUtc = isDateTimeSane(year, month, day, hour, minute, second);
  uint64_t epochMs = hasUtc ? buildEpochMsUtc(year, month, day, hour, minute, second, 0) : 0;

  if (hasUtc && rtcPresent && !rtcSyncedFromGnss) {
    syncRtcFromGnss(epochMs);
  }

  uint8_t fixQuality = gnss.getFixType();
  if (fixQuality < 2) {
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
  latestGnssFix.satellites = gnss.getSIV();
  portEXIT_CRITICAL(&stateMux);

  return true;
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

  while (true) {
    imu.getSensorData();

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

    if (gnssSnapshot.hasValidFix) {
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
  uint32_t linesSinceFlush = 0;
  LoggerRecord record = {};

  while (true) {
    if (xQueueReceive(logQueue, &record, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (formatCsvRecord(record, csvLine, sizeof(csvLine))) {
        // Flush in batches to keep SD latency away from the IMU sampling path.
        logFile.println(csvLine);
        writtenLogLines++;
        linesSinceFlush++;
      }
    }

    uint32_t nowMs = millis();
    if (linesSinceFlush > 0 &&
        (nowMs - lastFlushMs >= AppConfig::kLogFlushIntervalMs ||
         linesSinceFlush >= AppConfig::kLogFlushLineInterval)) {
      logFile.flush();
      lastFlushMs = nowMs;
      linesSinceFlush = 0;
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
