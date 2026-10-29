#ifndef DAQ_NETWORK_CONFIG_H
#define DAQ_NETWORK_CONFIG_H
/* Aggregate live SENSOR packet cap. ADC/report windows and SD are independent.
 * Heartbeats, acknowledgements, commands and discovery do not use this cap. */
#ifndef DAQ_NETWORK_RATE_HZ
#define DAQ_NETWORK_RATE_HZ 100U
#endif
#if DAQ_NETWORK_RATE_HZ < 1 || DAQ_NETWORK_RATE_HZ > 1000U
#error "DAQ_NETWORK_RATE_HZ must be from 1 to 1000"
#endif

/* Relative shares of the aggregate DAQ_NETWORK_RATE_HZ sensor budget.
 * Defaults at100 packets/s: selected50kg40,1000kg40, each auxiliary4.
 * Change these and rebuild. Zero disables only that stream's NETWORK output;
 * acquisition, SD logging and control/heartbeat/ACK traffic are unaffected. */
#ifndef DAQ_NETWORK_WEIGHT_KG50
#define DAQ_NETWORK_WEIGHT_KG50 10U
#endif
#ifndef DAQ_NETWORK_WEIGHT_KG1000
#define DAQ_NETWORK_WEIGHT_KG1000 10U
#endif
#ifndef DAQ_NETWORK_WEIGHT_TEMPERATURE
#define DAQ_NETWORK_WEIGHT_TEMPERATURE 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_SDADC
#define DAQ_NETWORK_WEIGHT_SDADC 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_SAR
#define DAQ_NETWORK_WEIGHT_SAR 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_CURRENT
#define DAQ_NETWORK_WEIGHT_CURRENT 1U
#endif
#ifndef DAQ_NETWORK_WEIGHT_POWER
#define DAQ_NETWORK_WEIGHT_POWER 1U
#endif
#define DAQ_NETWORK_WEIGHT_TOTAL (DAQ_NETWORK_WEIGHT_KG50 + DAQ_NETWORK_WEIGHT_KG1000 + \
    DAQ_NETWORK_WEIGHT_TEMPERATURE + DAQ_NETWORK_WEIGHT_SDADC + DAQ_NETWORK_WEIGHT_SAR + \
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
#error "At least one DAQ network stream weight must be positive"
#endif
#endif
