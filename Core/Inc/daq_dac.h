#ifndef DAQ_DAC_H
#define DAQ_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include <stddef.h>
#include <stdint.h>

/* DAC1 signal generator for ADC loopback validation (see docs/dac-validation.md).
 * OUT1 = PA4 (U19C, P10 pin 2), OUT2 = PA5 (U19B, P10 pin 5). Off at boot; only
 * GroundStation commands drive it, only in Startup/Idle, and only while the
 * station keeps its lease alive. */

typedef struct
{
  float analog_outputs_v[2]; /* DAC data output register, pin volts; NaN when off. */
} daq_dac_sample_t;

/* Puts both channels in the safe off state (output disabled, no trigger/DMA). */
UINT daq_dac_init(void);
/* Acquisition thread. With read_back, also converts each active channel on
 * the ADC4 internal DAC channel (at most two conversions, 100 ms cadence). */
UINT daq_dac_sample(daq_dac_sample_t *sample, uint8_t read_back);

/* Router thread: copies one DAQ_DAC_COMMAND payload for the service. */
void daq_dac_submit_command(const uint8_t *payload, size_t len);
/* Any thread: immediately stops both outputs (abort, flight-state change). */
void daq_dac_emergency_off(uint8_t reason);
/* Periodic owner (live network thread): applies commands, enforces the state
 * gate and lease, publishes DAQ_DAC_STATUS and logs SD rows. Rate-limited. */
void daq_dac_service(uint32_t now_ms);

extern volatile uint32_t g_daq_dac_active_mask;

#ifdef __cplusplus
}
#endif

#endif /* DAQ_DAC_H */
