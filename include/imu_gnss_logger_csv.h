#pragma once

#include <Arduino.h>

#include "imu_gnss_logger_types.h"

const char *csvHeader();
bool formatCsvRecord(const LoggerRecord &record, char *buffer, size_t bufferLength);
