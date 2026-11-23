#ifndef DAQ_NETWORK_CONFIG_H
#define DAQ_NETWORK_CONFIG_H
/* Aggregate live SENSOR packet cap (slots/s). ADC/report windows and SD are
 * independent.
 * Heartbeats, acknowledgements, commands and discovery do not use this cap. */
#ifndef DAQ_NETWORK_RATE_HZ
#define DAQ_NETWORK_RATE_HZ 100U
#endif
#if DAQ_NETWORK_RATE_HZ < 1 || DAQ_NETWORK_RATE_HZ > 1000U
#error "DAQ_NETWORK_RATE_HZ must be from 1 to 1000"
#endif

/* Slot classes, in order. Each slot (DAQ_NETWORK_RATE_HZ per second) goes to
 * the fast pair (KG1000 + chamber, 50 Hz) when one of its 20 ms bins is
 * waiting, else to the selected KG50 (20 Hz) when one of its 50 ms bins is
 * waiting, else to the next auxiliary stream in the weighted interleave
 * below. The two load-cell streams are offered once per bin, so they never
 * exceed 70 slots/s and the auxiliary streams keep at least 30 slots/s.
 * Every load-cell bin is sent, at uniform bin times: the old weighted
 * interleave sent the latest 4 ms bin at uneven slots repeating every 250 ms,
 * which put a 4 Hz line into the received stream.
 *
 * Weights of the auxiliary streams (KG50/KG1000 weights are unused: those
 * streams have their own slot classes). Zero disables only that stream's
 * NETWORK output; acquisition, SD logging and control traffic are
 * unaffected. */
#ifndef DAQ_NETWORK_WEIGHT_KG50
#define DAQ_NETWORK_WEIGHT_KG50 0U
#endif
#ifndef DAQ_NETWORK_WEIGHT_KG1000
#define DAQ_NETWORK_WEIGHT_KG1000 0U
#endif
#ifndef DAQ_NETWORK_WEIGHT_TEMPERATURE
#define DAQ_NETWORK_WEIGHT_TEMPERATURE 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_SDADC
#define DAQ_NETWORK_WEIGHT_SDADC 2U
#endif
#ifndef DAQ_NETWORK_WEIGHT_SAR
#define DAQ_NETWORK_WEIGHT_SAR 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_CURRENT
#define DAQ_NETWORK_WEIGHT_CURRENT 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_POWER
#define DAQ_NETWORK_WEIGHT_POWER 2U
#endif
#define DAQ_NETWORK_WEIGHT_TOTAL (DAQ_NETWORK_WEIGHT_TEMPERATURE + DAQ_NETWORK_WEIGHT_SDADC + DAQ_NETWORK_WEIGHT_SAR + \
    DAQ_NETWORK_WEIGHT_CURRENT + DAQ_NETWORK_WEIGHT_POWER)
#if DAQ_NETWORK_WEIGHT_KG50 < 0 || DAQ_NETWORK_WEIGHT_KG50 > 1000U || \
    DAQ_NETWORK_WEIGHT_KG1000 < 0 || DAQ_NETWORK_WEIGHT_KG1000 > 1000U || \
    DAQ_NETWORK_WEIGHT_TEMPERATURE < 0 || DAQ_NETWORK_WEIGHT_TEMPERATURE > 1000U || \
    DAQ_NETWORK_WEIGHT_SDADC < 0 || DAQ_NETWORK_WEIGHT_SDADC > 1000U || \
    DAQ_NETWORK_WEIGHT_SAR < 0 || DAQ_NETWORK_WEIGHT_SAR > 1000U || \
    DAQ_NETWORK_WEIGHT_CURRENT < 0 || DAQ_NETWORK_WEIGHT_CURRENT > 1000U || \
    DAQ_NETWORK_WEIGHT_POWER < 0 || DAQ_NETWORK_WEIGHT_POWER > 1000U
#error "DAQ network stream weights must be from 0 to 1000"
#endif
#if DAQ_NETWORK_WEIGHT_TOTAL == 0U
#error "At least one auxiliary DAQ network stream weight must be positive"
#endif
/* The load-cell slot classes use 1e6/DAQ_FAST_REPORT_US + 1e6/DAQ_KG50_REPORT_US
 * slots/s (70); keep at least 10 slots/s for the auxiliary streams. */
#define DAQ_NETWORK_LOADCELL_SLOTS_HZ 70U
#if DAQ_NETWORK_RATE_HZ < DAQ_NETWORK_LOADCELL_SLOTS_HZ + 10U
#error "DAQ_NETWORK_RATE_HZ must leave room for every load-cell bin and the auxiliary streams (>= 80)"
#endif
#endif
