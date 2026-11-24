#ifndef MCP3564R_H
#define MCP3564R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "tx_api.h"
#include "mcp3564r_board_config.h"
#include "daq_acq_schedule.h"

typedef struct
{
  uint8_t channel; /* 0: KG1000, 1: KG50 AMP2, 2..7: auxiliary; 9: P6 differential at x16. */
  uint8_t sample_valid;
  uint8_t dma_busy;
  uint8_t queued_samples;
  uint8_t profile;  /* DAQ_ACQ_PROFILE_* of the frame that took it */
  uint8_t slot;     /* 0: fast scan A, 1: slot scan B */
  uint64_t monotonic_ms;  /* HAL millisecond base: sample_us / 1000 */
  uint64_t sample_us;     /* conversion midpoint, HAL microsecond base */
  uint32_t frame;         /* frame counter: gaps show skipped frames */
  int32_t code;
  float voltage_v; /* ADC-pin voltage, using the nominal internal reference. */
  float raw_value; /* Calibration input: ADC-pin volts on every channel (CH9 divided by PGA gain). */
  float temperature_c;
  int32_t temperature_code; /* Uncorrected TEMP conversion, valid when temperature_c is finite. */
  uint32_t overrun_count;
} mcp3564r_sample_t;

/* Frame engine diagnostics (published in DAQ_INPUT_HEALTH v2 and readable by
 * a debugger). Counters only ever increase. */
typedef struct
{
  volatile uint32_t frames;          /* frame ticks */
  volatile uint32_t frames_skipped;  /* tick found the previous frame still busy */
  volatile uint32_t slots_skipped;   /* slot item would not have ended in time */
  volatile uint32_t scans_timed_out; /* a scan's DRDY never arrived; restarted */
  volatile uint32_t status_faults;   /* invalid STATUS byte: device recovery */
  volatile uint32_t unexpected;      /* result for a channel not in the scan */
  volatile uint32_t recoveries;      /* thread-level device re-initialisations */
  volatile uint32_t fast_done_max_us; /* latest scan A end seen, from the frame start */
  volatile uint32_t slot_done_max_us; /* latest slot end seen, from the frame start */
  volatile uint32_t start_late_max_us; /* worst START latency after its compare */
  volatile uint32_t temperature_over_range; /* TEMP conversions over range (daq_over_range.h) */
  volatile int8_t temperature_over_range_last; /* latest TEMP conversion: -1/0/+1 */
} mcp3564r_diag_t;

extern mcp3564r_diag_t g_mcp3564r_diag;
/* GPIO enable-level readback only, NOT a measurement of the +12VA rail.
 * UINT32_MAX before initialization, then0/1. Init status4 means stuck low. */
extern volatile uint32_t g_mcp3564r_12v_enable_readback;
/* Explicit stalled-DMA recoveries; never a synthetic conversion/sample. */
extern volatile uint32_t g_mcp3564r_dma_timeout_count;

/* 0: AMP2/CH1; 1: P6 differential CH2-CH3 (scan channel 9). Applied at the
 * next frame. */
void mcp3564r_request_kg50_input(uint8_t input);
uint8_t mcp3564r_kg50_input(void);
/* DAQ_ACQ_PROFILE_FILL or DAQ_ACQ_PROFILE_LAUNCH, applied at the next frame. */
void mcp3564r_request_profile(uint8_t profile);
uint8_t mcp3564r_profile(void);

UINT mcp3564r_init(SPI_HandleTypeDef *spi);
/* Acquisition-thread service, once per cycle: DMA-stall detection, device
 * recovery and a missed-DRDY read. Never blocks on a running frame. */
HAL_StatusTypeDef mcp3564r_start_dma(void);
void mcp3564r_dma_complete(void);
void mcp3564r_dma_error(void);
void mcp3564r_data_ready_callback(uint16_t pin);
/* TIM2 interrupt: frame tick (CC1) and slot start (CC2). */
void mcp3564r_timer_irq(void);
UINT mcp3564r_get_sample(mcp3564r_sample_t *sample);
uint8_t mcp3564r_pending_samples(void);
/* HAL-base microseconds now (thread context). */
uint64_t mcp3564r_now_us(void);

#ifdef __cplusplus
}
#endif

#endif /* MCP3564R_H */
