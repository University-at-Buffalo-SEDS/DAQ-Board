#ifndef DAQ_NETWORK_SCHEDULE_H
#define DAQ_NETWORK_SCHEDULE_H
#include "daq_rates.h"
#include "daq_network_config.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum {
    DAQ_NETWORK_KG50, DAQ_NETWORK_KG1000, DAQ_NETWORK_TEMPERATURE,
    DAQ_NETWORK_SDADC, DAQ_NETWORK_SAR, DAQ_NETWORK_CURRENT, DAQ_NETWORK_POWER,
    DAQ_NETWORK_STREAMS
};
typedef struct {
    uint64_t captured_ms;
    uint32_t type;
    float values[8];
    uint8_t count, pending;
} daq_network_sample_t;
typedef struct {
    daq_network_sample_t latest[DAQ_NETWORK_STREAMS];
    uint32_t last_ms;
    uint16_t credit;
    int32_t current[DAQ_NETWORK_STREAMS];
    uint8_t started;
} daq_network_schedule_t;

/* Caller serializes producer/consumer access. No allocation or SDK call here.
 * Return 1 when replacing an unsent NETWORK sample; SD ownership is separate. */
static inline int daq_network_schedule_offer(daq_network_schedule_t *s,
    unsigned stream, uint32_t type, const float *values, size_t count, uint64_t captured_ms)
{
    if (stream >= DAQ_NETWORK_STREAMS || values == NULL || count == 0U || count > 8U) return -1;
    daq_network_sample_t *slot = &s->latest[stream];
    const int replaced = slot->pending != 0U;
    slot->type = type;
    slot->captured_ms = captured_ms;
    slot->count = (uint8_t)count;
    memcpy(slot->values, values, count * sizeof(float));
    slot->pending = 1U;
    return replaced;
}
static inline int daq_network_schedule_take(daq_network_schedule_t *s,
    uint32_t now_ms, unsigned *stream, daq_network_sample_t *sample)
{
    if (!s->started) {
        s->started = 1U;
        s->last_ms = now_ms;
    } else {
        const uint32_t elapsed = now_ms - s->last_ms;
        s->last_ms = now_ms;
        const uint32_t credit = elapsed >= 1000U ? 2000U :
            (uint32_t)s->credit + elapsed * DAQ_NETWORK_RATE_HZ;
        if (credit < 1000U) { s->credit = (uint16_t)credit; return 0; }
        /* A stalled worker gets one slot, never a catch-up burst. */
        s->credit = credit >= 2000U ? 0U : (uint16_t)(credit - 1000U);
    }
    /* Smooth weighted interleave. Empty scheduled slots stay empty: another
     * stream cannot steal bandwidth or exceed the aggregate cap. */
    static const uint16_t weights[DAQ_NETWORK_STREAMS] = {
        DAQ_NETWORK_WEIGHT_KG50, DAQ_NETWORK_WEIGHT_KG1000, DAQ_NETWORK_WEIGHT_TEMPERATURE,
        DAQ_NETWORK_WEIGHT_SDADC, DAQ_NETWORK_WEIGHT_SAR, DAQ_NETWORK_WEIGHT_CURRENT,
        DAQ_NETWORK_WEIGHT_POWER
    };
    unsigned selected = DAQ_NETWORK_STREAMS;
    for (unsigned i = 0; i < DAQ_NETWORK_STREAMS; ++i) {
        if (weights[i] == 0U) continue;
        s->current[i] += weights[i];
        if (selected == DAQ_NETWORK_STREAMS || s->current[i] > s->current[selected]) selected = i;
    }
    s->current[selected] -= (int32_t)DAQ_NETWORK_WEIGHT_TOTAL;
    *stream = selected;
    daq_network_sample_t *slot = &s->latest[*stream];
    if (!slot->pending) return 0;
    *sample = *slot;
    slot->pending = 0U;
    return 1;
}
#endif
