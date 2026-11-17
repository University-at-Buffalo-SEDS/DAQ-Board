#ifndef DAQ_DAC_CONTROL_H
#define DAQ_DAC_CONTROL_H

/* Hardware-independent DAC signal-generator logic: wire decode/encode, request
 * clamping, waveform tables, timer planning and the safety state machine.
 * No HAL, RTOS or SEDSnet dependency, so host tests compile it directly. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAQ_DAC_CHANNELS 2U
/* Samples per period of a periodic wave and the staircase step limit. Each
 * entry is one 32-bit DHR12Rx word, so both channels cost 256 bytes of RAM. */
#define DAQ_DAC_TABLE_LEN 32U
#define DAQ_DAC_CODE_MAX 4095U
/* LQFP64 has no VREF+ pin: VREF+ is VDDA (+3V3A). With the output buffer on,
 * the datasheet linear output range is 0.2 V .. VREF+ - 0.2 V. */
#define DAQ_DAC_VREF_V 3.3f
#define DAQ_DAC_MIN_V 0.20f
#define DAQ_DAC_MAX_V 3.10f
#define DAQ_DAC_FREQ_MIN_HZ 0.01f
#define DAQ_DAC_FREQ_MAX_HZ 1000.0f
#define DAQ_DAC_STEPS_MIN 2U
#define DAQ_DAC_STEPS_MAX DAQ_DAC_TABLE_LEN
#define DAQ_DAC_DWELL_MIN_MS 10U
#define DAQ_DAC_DWELL_MAX_MS 20000U
#define DAQ_DAC_LEASE_DEFAULT_MS 3000U
#define DAQ_DAC_LEASE_MIN_MS 1000U
#define DAQ_DAC_LEASE_MAX_MS 10000U

#define DAQ_DAC_COMMAND_VERSION 1U
#define DAQ_DAC_COMMAND_BYTES 32U
#define DAQ_DAC_STATUS_VERSION 1U
#define DAQ_DAC_STATUS_CHANNEL_BYTES 36U
#define DAQ_DAC_STATUS_BYTES (8U + DAQ_DAC_CHANNELS * DAQ_DAC_STATUS_CHANNEL_BYTES)
#define DAQ_DAC_CHANNEL_ALL 0xFFU

/* Network FLIGHT_STATE values in which output is permitted. */
#define DAQ_DAC_STATE_STARTUP 0U
#define DAQ_DAC_STATE_IDLE 1U
#define DAQ_DAC_STATE_UNKNOWN 0xFFU

typedef enum {
  DAQ_DAC_WAVE_DC = 0,
  DAQ_DAC_WAVE_SQUARE = 1,
  DAQ_DAC_WAVE_SINE = 2,
  DAQ_DAC_WAVE_TRIANGLE = 3,
  DAQ_DAC_WAVE_STAIRCASE = 4,
  DAQ_DAC_WAVE_COUNT = 5
} daq_dac_waveform_t;

typedef enum {
  DAQ_DAC_RESULT_OK = 0,
  DAQ_DAC_RESULT_REFUSED_STATE = 1,
  DAQ_DAC_RESULT_REFUSED_UNKNOWN_STATE = 2,
  DAQ_DAC_RESULT_BAD_REQUEST = 3,
  DAQ_DAC_RESULT_HARDWARE_ERROR = 4,
  DAQ_DAC_RESULT_NOT_READY = 5
} daq_dac_result_t;

typedef enum {
  DAQ_DAC_OFF_NONE = 0,       /* Off since boot. */
  DAQ_DAC_OFF_COMMANDED = 1,
  DAQ_DAC_OFF_FLIGHT_STATE = 2,
  DAQ_DAC_OFF_ABORT = 3,
  DAQ_DAC_OFF_LEASE = 4,      /* No GS keepalive: link loss. */
  DAQ_DAC_OFF_HARDWARE = 5
} daq_dac_off_reason_t;

#define DAQ_DAC_CLAMP_OFFSET 0x01U
#define DAQ_DAC_CLAMP_AMPLITUDE 0x02U
#define DAQ_DAC_CLAMP_FREQUENCY 0x04U
#define DAQ_DAC_CLAMP_STEPS 0x08U
#define DAQ_DAC_CLAMP_DWELL 0x10U
#define DAQ_DAC_CLAMP_LEASE 0x20U

typedef struct {
  uint8_t channel;
  uint8_t enable;
  uint8_t waveform;
  float amplitude_v;
  float offset_v;
  float frequency_hz;
  uint16_t steps;
  uint16_t lease_ms;
  uint32_t dwell_ms;
  uint32_t sequence;
} daq_dac_command_t;

/* An applied (clamped) configuration, in DAC-pin volts. */
typedef struct {
  uint8_t waveform;
  uint8_t clamp_flags;
  uint16_t steps;
  float amplitude_v;
  float offset_v;
  float frequency_hz; /* Requested (clamped); the actual one is in the plan. */
  uint32_t dwell_ms;
  uint16_t lease_ms;
} daq_dac_config_t;

/* Timer-paced sample plan: update rate = clock / ((psc + 1) * (arr + 1)). */
typedef struct {
  uint16_t psc;
  uint16_t arr;
  uint16_t samples;          /* Table entries used: 1 for DC. */
  float update_hz;           /* Actual sample rate (0 for DC). */
  float actual_frequency_hz; /* Periodic waves; 0 for DC and staircase. */
  uint32_t actual_dwell_ms;  /* Staircase; 0 otherwise. */
} daq_dac_plan_t;

typedef enum {
  DAQ_DAC_ACTION_NONE = 0,
  DAQ_DAC_ACTION_START = 1,   /* (Re)start the hardware with ch[c].config. */
  DAQ_DAC_ACTION_STOP = 2,    /* Disable the output (hi-Z, pulled to 0 V). */
} daq_dac_action_t;

typedef struct {
  uint8_t active;
  uint8_t last_result;
  uint8_t off_reason;
  uint8_t configured; /* config holds a last-applied configuration. */
  daq_dac_config_t config;   /* Clamped request; identical requests refresh. */
  float actual_frequency_hz;
  uint32_t actual_dwell_ms;
  uint32_t sequence;
  uint32_t lease_deadline_ms;
} daq_dac_channel_state_t;

typedef struct {
  daq_dac_channel_state_t ch[DAQ_DAC_CHANNELS];
  /* Bit c: channel c changed (activity, configuration or result) since the
   * last daq_dac_take_changes(). Drives status and SD event logging. */
  uint8_t changed;
} daq_dac_ctrl_t;

/* Gate: DAQ_DAC_RESULT_OK only in Startup or Idle with a known state. */
daq_dac_result_t daq_dac_state_gate(bool has_state, uint8_t state);

int daq_dac_command_decode(const uint8_t *payload, size_t len, daq_dac_command_t *out);
/* Encodes exactly DAQ_DAC_COMMAND_BYTES (host tests and the simulator). */
void daq_dac_command_encode(const daq_dac_command_t *cmd, uint8_t out[DAQ_DAC_COMMAND_BYTES]);

/* Clamps a request to the output range; returns the clamp flags. */
uint8_t daq_dac_clamp(const daq_dac_command_t *cmd, daq_dac_config_t *cfg);

uint16_t daq_dac_volts_to_code(float volts);
float daq_dac_code_to_volts(uint32_t code);

/* Plans the trigger timer (actual frequency/dwell after quantisation) and
 * fills the table with DHR12R words. Returns 0, or -1 if unplannable. */
int daq_dac_plan(const daq_dac_config_t *cfg, uint32_t timer_clock_hz, daq_dac_plan_t *plan,
                 uint32_t table[DAQ_DAC_TABLE_LEN]);

void daq_dac_ctrl_init(daq_dac_ctrl_t *ctrl);
/* Applies one decoded command; fills actions[c]. Returns the result code for
 * the addressed channel (all channels share it for DAQ_DAC_CHANNEL_ALL). */
daq_dac_result_t daq_dac_ctrl_command(daq_dac_ctrl_t *ctrl, const daq_dac_command_t *cmd,
                                      bool has_state, uint8_t state, uint32_t now_ms,
                                      uint8_t actions[DAQ_DAC_CHANNELS]);
/* Periodic safety check: flight state and lease expiry. */
void daq_dac_ctrl_tick(daq_dac_ctrl_t *ctrl, bool has_state, uint8_t state, uint32_t now_ms,
                       uint8_t actions[DAQ_DAC_CHANNELS]);
/* Unconditional off for every active channel (abort, state change, fault). */
void daq_dac_ctrl_force_off(daq_dac_ctrl_t *ctrl, uint8_t reason,
                            uint8_t actions[DAQ_DAC_CHANNELS]);
/* Records the hardware outcome of a START action. A failed start leaves the
 * channel off with a hardware reason. */
void daq_dac_ctrl_started(daq_dac_ctrl_t *ctrl, unsigned channel, const daq_dac_plan_t *plan,
                          bool ok);
uint8_t daq_dac_ctrl_active_mask(const daq_dac_ctrl_t *ctrl);
uint8_t daq_dac_take_changes(daq_dac_ctrl_t *ctrl);

void daq_dac_status_encode(const daq_dac_ctrl_t *ctrl, uint8_t flight_state, uint8_t gate,
                           uint8_t flags, uint32_t uptime_ms,
                           const float output_v[DAQ_DAC_CHANNELS],
                           const float readback_v[DAQ_DAC_CHANNELS],
                           uint8_t out[DAQ_DAC_STATUS_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_DAC_CONTROL_H */
