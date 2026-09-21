#ifndef SD_CALENDAR_H
#define SD_CALENDAR_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { unsigned year, month, day, hour, minute, second; } sd_calendar_t;
/* FAT stores no timezone and represents years 1980..2107. We write UTC. */
static inline bool sd_calendar_from_unix_ms(uint64_t ms, sd_calendar_t *out)
{
  if (out == 0 || ms < 315532800000ULL || ms >= 4354819200000ULL) return false;
  uint64_t seconds = ms / 1000U;
  unsigned days = (unsigned)(seconds / 86400U), year = 1970U;
  for (;;) {
    const unsigned leap = year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U);
    if (days < 365U + leap) break;
    days -= 365U + leap; ++year;
  }
  const unsigned leap = year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U);
  const unsigned lengths[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  unsigned month = 0;
  while (days >= lengths[month] + (month == 1U ? leap : 0U)) {
    days -= lengths[month] + (month == 1U ? leap : 0U); ++month;
  }
  *out = (sd_calendar_t){year,month+1U,days+1U,
      (unsigned)(seconds % 86400U / 3600U), (unsigned)(seconds % 3600U / 60U),
      (unsigned)(seconds % 60U)};
  return true;
}
#endif
