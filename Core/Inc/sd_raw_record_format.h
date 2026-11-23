#ifndef SD_RAW_RECORD_FORMAT_H
#define SD_RAW_RECORD_FORMAT_H
#include "sd_float_format.h"
#include "board_uptime.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Writer-owned cache: the die temperature changes at about 14 Hz while raw
 * samples arrive 3000 times/second. A row repeats the temperature columns
 * only when the value changed, on the first row after the cache is reset
 * (the writer resets it when it switches logs), and at least every
 * SD_RAW_TEMPERATURE_REPEAT rows; otherwise
 * both columns are empty, meaning "unchanged". Never caches a measurement
 * or a time. */
#define SD_RAW_TEMPERATURE_REPEAT 256U
typedef struct {
    uint32_t temperature_bits;
    int32_t temperature_code;
    uint16_t rows_since;
    uint8_t temperature_valid;
    char temperature_text[24];
    const void *owner;
} sd_raw_format_cache_t;

static inline void sd_format_i32(char output[12], int32_t value)
{
    char reverse[10];
    unsigned count = 0U;
    uint32_t magnitude = (uint32_t)value;
    if (value < 0) { *output++ = '-'; magnitude = 0U - magnitude; }
    do { reverse[count++] = (char)('0' + magnitude % 10U); magnitude /= 10U; } while (magnitude);
    while (count) *output++ = reverse[--count];
    *output = '\0';
}

/* Thirteen-column raw CSV, without a general varargs formatter in the
 * acquisition-rate path. Keep the nine-digit binary32 representation.
 * monotonic_ms is the millisecond uptime; uptime_ticks/uptime_frequency_hz
 * carry the record's own clock: microseconds (1000000) for MCP3564R
 * conversions, milliseconds (1000) for the ADC1 supply records. On failure
 * the destination is empty and never overrun. */
static inline int sd_format_csv_raw_record(char *dst, size_t capacity,
    uint64_t network_ms, uint64_t uptime, uint64_t uptime_ticks, uint32_t uptime_hz,
    const char *sensor, int32_t raw_code,
    float raw_value, float calibrated_value, float temperature, int32_t temperature_code,
    uint64_t sequence, sd_raw_format_cache_t *cache)
{
    if (!dst || !capacity || !sensor || !cache) return -1;
    char stamp[21], ticks[21], fine_ticks[21], seq[21], raw[24], converted[24];
    char raw_code_text[12], temperature_code_text[12], frequency[21];
    sd_format_u64(stamp, network_ms ? network_ms : uptime);
    sd_format_u64(ticks, uptime);
    sd_format_u64(seq, sequence);
    sd_format_u64(fine_ticks, uptime_ticks);
    sd_format_u64(frequency, uptime_hz);
    sd_format_i32(raw_code_text, raw_code);
    sd_format_i32(temperature_code_text, temperature_code);
    sd_format_float(raw, raw_value);
    uint32_t raw_bits, converted_bits, temperature_bits;
    memcpy(&raw_bits, &raw_value, sizeof(raw_bits));
    memcpy(&converted_bits, &calibrated_value, sizeof(converted_bits));
    memcpy(&temperature_bits, &temperature, sizeof(temperature_bits));
    const char *calibrated = raw;
    if (raw_bits != converted_bits) {
        sd_format_float(converted, calibrated_value);
        calibrated = converted;
    }
    const int repeat = !cache->temperature_valid || temperature_bits != cache->temperature_bits ||
        temperature_code != cache->temperature_code || cache->rows_since + 1U >= SD_RAW_TEMPERATURE_REPEAT;
    if (repeat) {
        sd_format_float(cache->temperature_text, temperature);
        cache->temperature_bits = temperature_bits;
        cache->temperature_code = temperature_code;
        cache->temperature_valid = 1U;
        cache->rows_since = 0U;
    } else {
        ++cache->rows_since;
    }
    const char *fields[] = {stamp, ticks, sensor, "", raw_code_text, raw, calibrated,
        network_ms ? "network" : "local", repeat ? cache->temperature_text : "",
        repeat ? temperature_code_text : "", fine_ticks, frequency, seq};
    size_t used = 0U;
    for (unsigned i = 0U; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        for (const char *p = fields[i]; *p; ++p) {
            if (used + 1U >= capacity) { dst[0] = '\0'; return -1; }
            dst[used++] = *p;
        }
        if (i + 1U < sizeof(fields) / sizeof(fields[0])) {
            if (used + 1U >= capacity) { dst[0] = '\0'; return -1; }
            dst[used++] = ',';
        }
    }
    if (capacity - used < 3U) { dst[0] = '\0'; return -1; }
    dst[used++] = '\r'; dst[used++] = '\n'; dst[used] = '\0';
    return (int)used;
}
#endif
