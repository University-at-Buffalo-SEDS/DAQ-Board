#ifndef SD_LOG_PATH_H
#define SD_LOG_PATH_H
#include <stdint.h>
#include <string.h>
#include "sd_calendar.h"
/* Directory names are short-name safe. Category describes file creation,
 * never a substituted clock for queued captured rows. Launch wins over UTC. */
static inline const char *sd_log_directory(const char *prefix, uint64_t utc_ms)
{
    sd_calendar_t calendar;
    if (strncmp(prefix, "DAQ_LAUNCH", sizeof("DAQ_LAUNCH") - 1U) == 0)
        return "/LAUNCH";
    return sd_calendar_from_unix_ms(utc_ms, &calendar) ? "/UTC" : "/NO_UTC";
}
#endif
