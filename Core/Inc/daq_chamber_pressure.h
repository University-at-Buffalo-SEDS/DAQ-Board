#ifndef DAQ_CHAMBER_PRESSURE_H
#define DAQ_CHAMBER_PRESSURE_H
#include <math.h>
#include <stdint.h>
/* Schematic sheet9: P4/3 HVAIN1 -> R27/R31 divider -> U8B -> U7 CH6.
 * The installed PX309-1KG5V uses HVAIN1. Explicit -1 disables this input;
 * P4/4 HVAIN2 remains available as an explicit CH7 override. */
#ifndef DAQ_CHAMBER_ADC_CHANNEL
#define DAQ_CHAMBER_ADC_CHANNEL 6
#endif
#if DAQ_CHAMBER_ADC_CHANNEL != -1 && DAQ_CHAMBER_ADC_CHANNEL != 6 && DAQ_CHAMBER_ADC_CHANNEL != 7
#error "Select -1 (unassigned), 6 (P4/3), or 7 (P4/4)"
#endif
#if DAQ_CHAMBER_ADC_CHANNEL >= 0
#define DAQ_CHAMBER_SCAN_MASK (1U << DAQ_CHAMBER_ADC_CHANNEL)
#else
#define DAQ_CHAMBER_SCAN_MASK 0U
#endif
/* OMEGA PX309-1KG5V:0..1000psi gauge,0..5V output,9..30V supply.
 * OfficialOMEGA PX309 datasheet page4. Preserve out-of-range voltage-derived
 * pressure instead of clamping a possible overrange reading to fullscale. */
static inline float daq_chamber_pressure_psi(float connector_v)
{
    return isfinite(connector_v) ? connector_v * 200.0f : NAN;
}
static inline int daq_chamber_is_channel(unsigned channel)
{
    return DAQ_CHAMBER_SCAN_MASK != 0U && channel == (unsigned)DAQ_CHAMBER_ADC_CHANNEL;
}
#endif
