#include "imu_gnss_logger_csv.h"

#include <stdio.h>
#include <time.h>

namespace {

void formatEpochMillisUtc(uint64_t epochMs, char *target, size_t length) {
  time_t epochSeconds = static_cast<time_t>(epochMs / 1000ULL);
  struct tm utcTime = {};
  gmtime_r(&epochSeconds, &utcTime);
  uint16_t milliseconds = static_cast<uint16_t>(epochMs % 1000ULL);
  snprintf(target, length, "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
           utcTime.tm_year + 1900,
           utcTime.tm_mon + 1,
           utcTime.tm_mday,
           utcTime.tm_hour,
           utcTime.tm_min,
           utcTime.tm_sec,
           milliseconds);
}

}  // namespace

const char *csvHeader() {
  return "timestamp_ms,datetime_utc,imu_sample_id,gnss_fix_id,lat,lon,alt_m,speed_mps,heading_deg,gnss_fix_quality,gnss_satellites,ax,ay,az,gx,gy,gz,imu_dt_ms,gnss_age_ms";
}

bool formatCsvRecord(const LoggerRecord &record, char *buffer, size_t bufferLength) {
  char datetimeUtc[32];
  char lat[20];
  char lon[20];
  char altM[16];
  char speedMps[16];
  char headingDeg[16];
  char fixQuality[8];
  char satellites[8];
  char gnssAgeMs[16];

  if (record.hasUtc) {
    formatEpochMillisUtc(record.utcEpochMs, datetimeUtc, sizeof(datetimeUtc));
  } else {
    snprintf(datetimeUtc, sizeof(datetimeUtc), "NA");
  }

  if (record.hasGnss) {
    snprintf(lat, sizeof(lat), "%.7f", static_cast<double>(record.latitudeE7) / 10000000.0);
    snprintf(lon, sizeof(lon), "%.7f", static_cast<double>(record.longitudeE7) / 10000000.0);
    snprintf(altM, sizeof(altM), "%.3f", static_cast<double>(record.altitudeMm) / 1000.0);
    snprintf(speedMps, sizeof(speedMps), "%.3f", static_cast<double>(record.speedMmPerSec) / 1000.0);
    snprintf(headingDeg, sizeof(headingDeg), "%.5f", static_cast<double>(record.headingDegE5) / 100000.0);
    snprintf(fixQuality, sizeof(fixQuality), "%u", record.gnssFixQuality);
    snprintf(satellites, sizeof(satellites), "%u", record.gnssSatellites);
    snprintf(gnssAgeMs, sizeof(gnssAgeMs), "%lu", static_cast<unsigned long>(record.gnssAgeMs));
  } else {
    snprintf(lat, sizeof(lat), "NA");
    snprintf(lon, sizeof(lon), "NA");
    snprintf(altM, sizeof(altM), "NA");
    snprintf(speedMps, sizeof(speedMps), "NA");
    snprintf(headingDeg, sizeof(headingDeg), "NA");
    snprintf(fixQuality, sizeof(fixQuality), "NA");
    snprintf(satellites, sizeof(satellites), "NA");
    snprintf(gnssAgeMs, sizeof(gnssAgeMs), "NA");
  }

  int written = snprintf(buffer, bufferLength,
                         "%lu,%s,%lu,%lu,%s,%s,%s,%s,%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%s",
                         static_cast<unsigned long>(record.timestampMs),
                         datetimeUtc,
                         static_cast<unsigned long>(record.imuSampleId),
                         static_cast<unsigned long>(record.gnssFixId),
                         lat,
                         lon,
                         altM,
                         speedMps,
                         headingDeg,
                         fixQuality,
                         satellites,
                         static_cast<double>(record.ax),
                         static_cast<double>(record.ay),
                         static_cast<double>(record.az),
                         static_cast<double>(record.gx),
                         static_cast<double>(record.gy),
                         static_cast<double>(record.gz),
                         static_cast<double>(record.imuDtMs),
                         gnssAgeMs);

  return written > 0 && static_cast<size_t>(written) < bufferLength;
}
