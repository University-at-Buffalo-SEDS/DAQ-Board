#ifndef DAQ_RATES_H
#define DAQ_RATES_H

/* Board-local settings. No network variables; rebuild/reflash to change.
 * MCLK is the physical CubeMX MCO output and must match the board clock. */
#ifndef DAQ_ADC_OSR
#define DAQ_ADC_OSR 1024U
#endif
#ifndef DAQ_ADC_READ_RATE_HZ
#define DAQ_ADC_READ_RATE_HZ 3700U
#endif
#ifndef DAQ_ACQUISITION_PERIOD_MS
#define DAQ_ACQUISITION_PERIOD_MS 20U
#endif
#ifndef DAQ_BROADCAST_RATE_HZ
#define DAQ_BROADCAST_RATE_HZ 50U
#endif
#define DAQ_ADC_MCLK_HZ 16000000U
#define DAQ_RAW_BATCH_CAPACITY 96U
/* Bounded SD scheduling/startup reserve, independent of network cadence. */
#ifndef DAQ_SD_RAW_BUFFER_MS
#define DAQ_SD_RAW_BUFFER_MS 200U
#endif
#define DAQ_SD_RAW_QUEUE_DEPTH ((DAQ_SD_RAW_BUFFER_MS + DAQ_ACQUISITION_PERIOD_MS - 1U) / DAQ_ACQUISITION_PERIOD_MS + 1U)
#if DAQ_SD_RAW_QUEUE_DEPTH < 2 || DAQ_SD_RAW_QUEUE_DEPTH > 64
#error "SD raw reserve must fit between 2 and 64 batches"
#endif

#if DAQ_ADC_OSR == 32
#define DAQ_ADC_OSR_BITS 0U
#elif DAQ_ADC_OSR == 64
#define DAQ_ADC_OSR_BITS 1U
#elif DAQ_ADC_OSR == 128
#define DAQ_ADC_OSR_BITS 2U
#elif DAQ_ADC_OSR == 256
#define DAQ_ADC_OSR_BITS 3U
#elif DAQ_ADC_OSR == 512
#define DAQ_ADC_OSR_BITS 4U
#elif DAQ_ADC_OSR == 1024
#define DAQ_ADC_OSR_BITS 5U
#elif DAQ_ADC_OSR == 2048
#define DAQ_ADC_OSR_BITS 6U
#elif DAQ_ADC_OSR == 4096
#define DAQ_ADC_OSR_BITS 7U
#else
#error "Unsupported DAQ_ADC_OSR (choose a power of two from 32 to 4096)"
#endif
#if DAQ_ADC_READ_RATE_HZ < 1 || DAQ_ADC_READ_RATE_HZ > 12000
#error "DAQ_ADC_READ_RATE_HZ must be from 1 to 12000"
#endif
#if DAQ_ADC_READ_RATE_HZ > DAQ_ADC_MCLK_HZ / (4U * DAQ_ADC_OSR)
#error "ADC read rate exceeds conversion rate; reduce DAQ_ADC_OSR or read rate"
#endif
#if DAQ_ACQUISITION_PERIOD_MS < 1 || DAQ_ACQUISITION_PERIOD_MS > 1000
#error "DAQ_ACQUISITION_PERIOD_MS must be from 1 to 1000"
#endif
#if (DAQ_ADC_READ_RATE_HZ * DAQ_ACQUISITION_PERIOD_MS + 999U) / 1000U > DAQ_RAW_BATCH_CAPACITY
#error "ADC drain batch too small; reduce DAQ_ACQUISITION_PERIOD_MS"
#endif
#if DAQ_BROADCAST_RATE_HZ < 1 || DAQ_BROADCAST_RATE_HZ > 1000 / DAQ_ACQUISITION_PERIOD_MS
#error "DAQ_BROADCAST_RATE_HZ must not exceed the acquisition service rate"
#endif
#define DAQ_ADC_READ_INTERVAL_US ((1000000U + DAQ_ADC_READ_RATE_HZ - 1U) / DAQ_ADC_READ_RATE_HZ)
/* Datasheet Table 5-15: initial SINC3/SINC1 conversion settling. */
#define DAQ_ADC_SETTLE_CLOCKS ((DAQ_ADC_OSR <= 512U) ? (3U * DAQ_ADC_OSR) : (DAQ_ADC_OSR + 1024U))
#define DAQ_ADC_FIRST_CONVERSION_US ((DAQ_ADC_SETTLE_CLOCKS * 1000000ULL * 4U + DAQ_ADC_MCLK_HZ - 1U) / DAQ_ADC_MCLK_HZ + 32U)
#define DAQ_BROADCAST_PERIOD_MS ((1000U + DAQ_BROADCAST_RATE_HZ - 1U) / DAQ_BROADCAST_RATE_HZ)
#endif
