#pragma once

#include <Arduino.h>

namespace AppConfig {

constexpr bool kEnableDebugPrint = true;
constexpr bool kEnableRtc = true;
constexpr bool kEnableGnssL5Tweaks = true;

// Keep the board default I2C pins unless the hardware wiring is explicitly known.
constexpr bool kUseBoardDefaultI2cPins = true;
constexpr int8_t kI2cSdaPin = -1;
constexpr int8_t kI2cSclPin = -1;

constexpr uint32_t kDebugBaudrate = 115200;
constexpr uint8_t kSdCsPin = D2;
constexpr uint8_t kGnssRxPin = D0;
constexpr uint8_t kGnssTxPin = D1;

constexpr uint32_t kGnssBaudrate = 38400;
constexpr uint16_t kImuSampleRateHz = 200;
constexpr uint32_t kGnssMeasurementPeriodMs = 50;
constexpr uint32_t kGnssPollPeriodMs = 50;

constexpr uint32_t kGnssBootPrimeTimeoutMs = 1500;
constexpr uint32_t kGnssInitRetryMs = 2000;
constexpr uint32_t kRtcRefreshIntervalMs = 1000;

constexpr size_t kLoggerQueueLength = 256;
constexpr uint32_t kLogFlushIntervalMs = 1000;
constexpr uint32_t kLogFlushLineInterval = 100;
constexpr uint32_t kStatusPrintIntervalMs = 5000;

constexpr uint8_t kRtcI2cAddress = 0x51;
constexpr size_t kCsvLineBufferSize = 256;

}  // namespace AppConfig
