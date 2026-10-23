#ifndef SD_RECORD_FORMAT_H
#define SD_RECORD_FORMAT_H
#include "sd_float_format.h"
#include "board_uptime.h"
#include <stddef.h>
#include <stdio.h>
#define SD_CSV_HEADER "timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence\r\n"
/* Sensor names are internal schema labels, not caller-controlled CSV text.
 * All message/value text is quoted with RFC 4180 escaping. */
static inline int sd_format_csv_record(char *dst, size_t capacity, uint64_t utc_ms,
                                       uint64_t uptime, const char *sensor,
                                       const char *value, uint64_t sequence)
{
    if (!dst || !capacity || !sensor || !value) return -1;
    char stamp[21], ticks[21], seq[21];
    sd_format_u64(stamp, utc_ms ? utc_ms : uptime);
    sd_format_u64(ticks, uptime); sd_format_u64(seq, sequence);
    int length = snprintf(dst, capacity, "%s,%s,%s,\"", stamp, ticks, sensor);
    if (length < 0 || (size_t)length >= capacity) { dst[0] = 0; return -1; }
    size_t used = (size_t)length;
    for (const char *p = value; *p; ++p) {
        size_t need = *p == '"' ? 2U : 1U;
        if (need >= capacity - used) { dst[0] = 0; return -1; }
        dst[used++] = *p; if (*p == '"') dst[used++] = '"';
    }
    length = snprintf(dst + used, capacity - used, "\",,,,%s,,,%s,%u,%s\r\n",
                      utc_ms ? "network" : "local", ticks, BOARD_UPTIME_FREQUENCY_HZ, seq);
    if (length < 0 || (size_t)length >= capacity - used) { dst[0] = 0; return -1; }
    return (int)(used + (size_t)length);
}
/* Acquisition values are formatter-generated numeric vectors, containing no
 * CSV metacharacters. Raw codes and converted values belong to one sample. */
static inline int sd_format_csv_sensor_record(char *dst, size_t capacity,
    uint64_t utc_ms, uint64_t uptime, const char *sensor,
    const char *raw, const char *converted, uint64_t sequence)
{
    if (!dst || !capacity || !sensor || !raw || !converted) return -1;
    char stamp[21], ticks[21], seq[21];
    sd_format_u64(stamp, utc_ms ? utc_ms : uptime);
    sd_format_u64(ticks, uptime); sd_format_u64(seq, sequence);
    int n = snprintf(dst, capacity,
        "%s,%s,%s,\"%s\",\"%s\",\"%s\",\"%s\",%s,,,%s,%u,%s\r\n",
        stamp, ticks, sensor, converted, raw, raw, converted,
        utc_ms ? "network" : "local", ticks, BOARD_UPTIME_FREQUENCY_HZ, seq);
    if (n < 0 || (size_t)n >= capacity) { dst[0] = 0; return -1; }
    return n;
}

#endif
