#ifndef PRESSURE_SD_WIRE_H
#define PRESSURE_SD_WIRE_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
/* VALVE_PRESSURE_SD_V1: four alternating conversions from one half of the
 * 500 Hz ADC DMA ring. Only VALVE_PRESSURE_LOG is a destination. No native struct padding
 * crosses the wire. Sequence counts acquired samples, including queue drops. */
#define PRESSURE_SD_SAMPLES 4U
#define PRESSURE_SD_BYTES 44U
#define PRESSURE_SD_PERIOD_MS 4U
#define PRESSURE_SD_CLOCK_HZ 1000U
typedef struct {
    uint64_t uptime_ms;
    uint64_t unix_ms;
    uint32_t first_sequence;
    uint16_t raw[PRESSURE_SD_SAMPLES];
    float psi[PRESSURE_SD_SAMPLES];
} pressure_sd_batch_t;
static inline void pressure_sd_put(uint8_t *dst, uint64_t value, unsigned bytes) {
    for (unsigned i=0;i<bytes;++i) dst[i]=(uint8_t)(value>>(8U*i));
}
static inline uint64_t pressure_sd_get(const uint8_t *src, unsigned bytes) {
    uint64_t value=0;for(unsigned i=0;i<bytes;++i)value|=(uint64_t)src[i]<<(8U*i);return value;
}
static inline int pressure_sd_encode(uint8_t *dst,size_t size,const pressure_sd_batch_t *batch) {
    _Static_assert(sizeof(float)==4,"pressure wire format requires binary32");
    if(!dst || size<PRESSURE_SD_BYTES || !batch || batch->uptime_ms>UINT64_MAX-12U ||
       (batch->unix_ms && batch->unix_ms>UINT64_MAX-12U))return -1;
    for(unsigned i=0;i<PRESSURE_SD_SAMPLES;++i)
        if(batch->raw[i]>4095U || !isfinite(batch->psi[i]) || batch->psi[i]<0 || batch->psi[i]>2000)return -1;
    pressure_sd_put(dst,batch->uptime_ms,8);pressure_sd_put(dst+8,batch->unix_ms,8);
    pressure_sd_put(dst+16,batch->first_sequence,4);
    for(unsigned i=0;i<PRESSURE_SD_SAMPLES;++i){
        uint32_t bits;memcpy(&bits,&batch->psi[i],4);
        pressure_sd_put(dst+20+i*6,batch->raw[i],2);pressure_sd_put(dst+22+i*6,bits,4);
    }
    return PRESSURE_SD_BYTES;
}
static inline int pressure_sd_decode(pressure_sd_batch_t *batch,const uint8_t *src,size_t size) {
    if(!batch || !src || size!=PRESSURE_SD_BYTES)return -1;
    pressure_sd_batch_t decoded={0};
    decoded.uptime_ms=pressure_sd_get(src,8);decoded.unix_ms=pressure_sd_get(src+8,8);
    decoded.first_sequence=(uint32_t)pressure_sd_get(src+16,4);
    if(decoded.uptime_ms>UINT64_MAX-12U || (decoded.unix_ms && decoded.unix_ms>UINT64_MAX-12U))return -1;
    for(unsigned i=0;i<PRESSURE_SD_SAMPLES;++i){
        decoded.raw[i]=(uint16_t)pressure_sd_get(src+20+i*6,2);
        uint32_t bits=(uint32_t)pressure_sd_get(src+22+i*6,4);memcpy(&decoded.psi[i],&bits,4);
        if(decoded.raw[i]>4095U || !isfinite(decoded.psi[i]) || decoded.psi[i]<0 || decoded.psi[i]>2000)return -1;
    }
    *batch=decoded;return 0;
}
#endif
