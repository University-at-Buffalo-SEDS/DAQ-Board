#ifndef SD_RAW_RECORD_FORMAT_H
#define SD_RAW_RECORD_FORMAT_H
#include "sd_float_format.h"
#include "board_uptime.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Writer-owned cache: temperature changes at 10Hz while raw samples arrive
 * thousands of times/second. Cache its text, never any measurement or time. */
typedef struct {
    uint32_t temperature_bits;
    uint8_t temperature_valid;
    char temperature_text[24];
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

/* Byte-identical thirteen-column raw CSV, without a general varargs formatter
 * in the acquisition-rate path. Keep the nine-digit binary32 representation.
 * The empty value column, signed codes, both clocks, frequency and sequence
 * are unchanged. On failure the destination is empty and never overrun. */
static inline int sd_format_csv_raw_record(char *dst, size_t capacity,
    uint64_t network_ms, uint64_t uptime, const char *sensor, int32_t raw_code,
    float raw_value, float calibrated_value, float temperature, int32_t temperature_code,
    uint64_t sequence, sd_raw_format_cache_t *cache)
{
    if (!dst || !capacity || !sensor || !cache) return -1;
    char stamp[21], ticks[21], seq[21], raw[24], converted[24];
    char raw_code_text[12], temperature_code_text[12], frequency[21];
    sd_format_u64(stamp, network_ms ? network_ms : uptime);
    sd_format_u64(ticks, uptime);
    sd_format_u64(seq, sequence);
    sd_format_u64(frequency, BOARD_UPTIME_FREQUENCY_HZ);
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
    if (!cache->temperature_valid || temperature_bits != cache->temperature_bits) {
        sd_format_float(cache->temperature_text, temperature);
        cache->temperature_bits = temperature_bits;
        cache->temperature_valid = 1U;
    }
    const char *fields[] = {stamp, ticks, sensor, "", raw_code_text, raw, calibrated,
        network_ms ? "network" : "local", cache->temperature_text,
        temperature_code_text, ticks, frequency, seq};
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
