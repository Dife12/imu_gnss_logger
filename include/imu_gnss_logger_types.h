#pragma once

#include <Arduino.h>
#include <stdint.h>

struct GnssFixState {
  bool hasValidFix = false;
  bool hasUtc = false;
  uint32_t fixId = 0;
  uint32_t lastUpdateMs = 0;
  uint64_t utcEpochMs = 0;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMm = 0;
  int32_t speedMmPerSec = 0;
  int32_t headingDegE5 = 0;
  uint8_t fixQuality = 0;
  uint8_t satellites = 0;
};

struct LoggerRecord {
  uint32_t timestampMs = 0;
  uint32_t imuSampleId = 0;
  uint32_t gnssFixId = 0;
  uint32_t gnssAgeMs = 0;
  uint64_t utcEpochMs = 0;
  bool hasUtc = false;
  bool hasGnss = false;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMm = 0;
  int32_t speedMmPerSec = 0;
  int32_t headingDegE5 = 0;
  uint8_t gnssFixQuality = 0;
  uint8_t gnssSatellites = 0;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float imuDtMs = 0.0f;
};
