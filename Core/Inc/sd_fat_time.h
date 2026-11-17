#ifndef SD_FAT_TIME_H
#define SD_FAT_TIME_H
#include <stdbool.h>
#include <stdint.h>
#include "sd_calendar.h"
/* FAT directory stamps (created, modified, accessed) carry no time zone, and
 * Windows, macOS and Linux all show them as local time. The SD writer stamps
 * the zone below; file names, directories (/UTC, /NO_UTC) and every CSV row
 * stay UTC. Build-time zone, standard offset east of UTC in minutes plus the
 * US daylight rule (second Sunday of March 02:00 standard time to first Sunday
 * of November 02:00 daylight time). Default America/New_York; for UTC stamps
 * build with -DDAQ_SD_FAT_UTC_OFFSET_MIN=0 -DDAQ_SD_FAT_US_DST=0. */
#ifndef DAQ_SD_FAT_UTC_OFFSET_MIN
#define DAQ_SD_FAT_UTC_OFFSET_MIN (-300)
#endif
#ifndef DAQ_SD_FAT_US_DST
#define DAQ_SD_FAT_US_DST 1
#endif
_Static_assert(DAQ_SD_FAT_UTC_OFFSET_MIN >= -720 && DAQ_SD_FAT_UTC_OFFSET_MIN <= 840,
               "FAT stamp zone offset must be a real UTC offset");

/* Days since 1970-01-01 of a proleptic Gregorian date. */
static inline int64_t sd_fat_days_from_civil(int64_t y, unsigned m, unsigned d)
{
  y -= m <= 2U;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153U * (m > 2U ? m - 3U : m + 9U) + 2U) / 5U + d - 1U;
  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

/* Day (since 1970) of the n-th Sunday on or after the first of a month. */
static inline int64_t sd_fat_sunday(unsigned year, unsigned month, unsigned n)
{
  const int64_t first = sd_fat_days_from_civil(year, month, 1U);
  return first + (7 - (first + 4) % 7) % 7 + 7 * (int64_t)(n - 1U); /* 1970-01-01: Thursday */
}

/* Offset in seconds of the stamp zone at a UTC instant. */
static inline int32_t sd_fat_zone_offset_s(uint64_t utc_ms)
{
  const int32_t standard = DAQ_SD_FAT_UTC_OFFSET_MIN * 60;
#if DAQ_SD_FAT_US_DST
  sd_calendar_t local;
  if (!sd_calendar_from_unix_ms(utc_ms + (uint64_t)(int64_t)standard * 1000U, &local)) return standard;
  const int64_t utc_s = (int64_t)(utc_ms / 1000U);
  const int64_t begins = sd_fat_sunday(local.year, 3U, 2U) * 86400 + 7200 - standard;
  const int64_t ends = sd_fat_sunday(local.year, 11U, 1U) * 86400 + 7200 - (standard + 3600);
  if (utc_s >= begins && utc_s < ends) return standard + 3600;
#else
  (void)utc_ms;
#endif
  return standard;
}

/* Local calendar for a FAT stamp; false when UTC is unknown or out of range. */
static inline bool sd_fat_local_calendar(uint64_t utc_ms, sd_calendar_t *out)
{
  if (!sd_calendar_from_unix_ms(utc_ms, out)) return false;
  return sd_calendar_from_unix_ms(utc_ms + (uint64_t)((int64_t)sd_fat_zone_offset_s(utc_ms) * 1000), out);
}

/* FAT packed date and time (2-second resolution). */
static inline uint16_t sd_fat_pack_date(const sd_calendar_t *c)
{
  return (uint16_t)(((c->year - 1980U) << 9) | (c->month << 5) | c->day);
}
static inline uint16_t sd_fat_pack_time(const sd_calendar_t *c)
{
  return (uint16_t)((c->hour << 11) | (c->minute << 5) | (c->second / 2U));
}
#endif
