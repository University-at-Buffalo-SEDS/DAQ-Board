#ifndef DAQ_VDDA_H
#define DAQ_VDDA_H
/* STM32 VDDA (= VREF+, one pin on this package, = the +3V3A rail) from the
 * internal reference VREFINT. Pure: no HAL.
 *
 * Every STM32 ADC1/ADC4 reading is code x VREF+ / full scale. Firmware used
 * to assume VREF+ = 3.3 V, so a lifted +3V3A read as a low VMAIN (see
 * daq_input_fault_config.h). VREFINT is a fixed ~1.21 V bandgap; the factory
 * value VREFINT_CAL (system memory 0x0BFA07A5, RM0456 "Internal voltage
 * reference"; VREFINT_CAL_ADDR in the LL ADC header) is its 14-bit reading
 * taken at VREF+ = 3.0 V +/-10 mV and 30 C +/-5 C. ADC1 runs at 14 bits, the
 * calibration resolution, so
 *     VDDA = 3.0 V x VREFINT_CAL / VREFINT_reading
 * (__LL_ADC_CALC_VREFANALOG_VOLTAGE). */
#include <math.h>
#include <stdint.h>

#define DAQ_VDDA_CAL_V 3.0f
#define DAQ_VDDA_NOMINAL_V 3.3f

/* NaN for a missing or implausible reading (outside 1.6-5.0 V: below the
 * U5's 1.62 V VDDA minimum, or a reading no live board can produce). */
static inline float daq_vdda_from_vrefint(uint32_t vrefint_code, uint16_t vrefint_cal)
{
  if (vrefint_code == 0U || vrefint_code >= 16383U || vrefint_cal == 0U || vrefint_cal == UINT16_MAX)
    return NAN;
  const float vdda = DAQ_VDDA_CAL_V * (float)vrefint_cal / (float)vrefint_code;
  return vdda >= 1.6f && vdda <= 5.0f ? vdda : NAN;
}

/* The reference to convert with: the measured VDDA, else the nominal rail. */
static inline float daq_vdda_reference(float measured_vdda)
{
  return isfinite(measured_vdda) ? measured_vdda : DAQ_VDDA_NOMINAL_V;
}
#endif
