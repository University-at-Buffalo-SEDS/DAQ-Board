#ifndef DAQ_DAC_CONTROL_H
#define DAQ_DAC_CONTROL_H

/* Hardware-independent DAC signal-generator logic: wire decode/encode, request
 * clamping, waveform tables, timer planning and the safety state machine.
 * No HAL, RTOS or SEDSnet dependency, so host tests compile it directly. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "daq_dac_wave.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAQ_DAC_CHANNELS 2U
#define DAQ_DAC_LEASE_DEFAULT_MS 3000U
#define DAQ_DAC_LEASE_MIN_MS 1000U
#define DAQ_DAC_LEASE_MAX_MS 10000U
/* Square duty is held inside 1 %..99 %. */
#define DAQ_DAC_DUTY_MIN_Q16 656U
#define DAQ_DAC_DUTY_MAX_Q16 64880U

/* Wire layouts (little endian); docs/dac-waveforms.md has the byte tables.
 * v2 replaced v1 when the whole stack moved together. */
#define DAQ_DAC_COMMAND_VERSION 2U
#define DAQ_DAC_COMMAND_BYTES 48U
#define DAQ_DAC_STATUS_VERSION 2U
#define DAQ_DAC_STATUS_CHANNEL_BYTES 52U
#define DAQ_DAC_STATUS_TABLE_BYTES 24U
#define DAQ_DAC_STATUS_BYTES (8U + DAQ_DAC_CHANNELS * DAQ_DAC_STATUS_CHANNEL_BYTES + DAQ_DAC_STATUS_TABLE_BYTES)
#define DAQ_DAC_CHANNEL_ALL 0xFFU

/* DAQ_DAC_TABLE: chunked upload of an arbitrary table, GS -> SD_CARD. */
#define DAQ_DAC_TABLE_VERSION 1U
#define DAQ_DAC_TABLE_CHUNK_SAMPLES 32U
#define DAQ_DAC_TABLE_BYTES (12U + 2U * DAQ_DAC_TABLE_CHUNK_SAMPLES)
#define DAQ_DAC_TABLE_CHUNKS_MAX (DAQ_DAC_ARB_LEN_MAX / DAQ_DAC_TABLE_CHUNK_SAMPLES)

/* Network FLIGHT_STATE values in which output is permitted. */
#define DAQ_DAC_STATE_STARTUP 0U
#define DAQ_DAC_STATE_IDLE 1U
#define DAQ_DAC_STATE_UNKNOWN 0xFFU

typedef enum {
  DAQ_DAC_RESULT_OK = 0,
  DAQ_DAC_RESULT_REFUSED_STATE = 1,
  DAQ_DAC_RESULT_REFUSED_UNKNOWN_STATE = 2,
  DAQ_DAC_RESULT_BAD_REQUEST = 3,
  DAQ_DAC_RESULT_HARDWARE_ERROR = 4,
  DAQ_DAC_RESULT_NOT_READY = 5,
  /* Arbitrary: no verified table with the commanded length and CRC. */
  DAQ_DAC_RESULT_TABLE_MISMATCH = 6
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
#define DAQ_DAC_CLAMP_DUTY 0x40U
#define DAQ_DAC_CLAMP_SWEEP 0x80U

typedef struct {
  uint8_t channel;
  uint8_t enable;
  uint8_t waveform;
  uint8_t flags;
  float amplitude_v;
  float offset_v;
  float frequency_hz;      /* Chirp: start. Noise: sample rate. Arbitrary: table repeats/s. */
  float frequency_stop_hz; /* Chirp. */
  uint16_t steps;
  uint16_t lease_ms;
  uint16_t phase_q16;
  uint16_t duty_q16;
  uint16_t table_len;      /* Arbitrary. */
  uint32_t dwell_ms;
  uint32_t sweep_ms;       /* Chirp. */
  uint32_t table_crc;      /* Arbitrary. */
  uint32_t sequence;
} daq_dac_command_t;

/* An applied (clamped) configuration, in DAC-pin volts. Fields a waveform
 * does not use are zero. */
typedef struct {
  uint8_t waveform;
  uint8_t clamp_flags;
  uint8_t flags;
  uint16_t steps;
  uint16_t phase_q16;
  uint16_t duty_q16;
  uint16_t table_len;
  uint16_t lease_ms;
  float amplitude_v;
  float offset_v;
  float frequency_hz; /* Requested (clamped); the actual one is in the plan. */
  float frequency_stop_hz;
  uint32_t dwell_ms;
  uint32_t sweep_ms;
  uint32_t table_crc;
} daq_dac_config_t;

/* Timer-paced sample plan: update rate = clock / ((psc + 1) * (arr + 1)). */
typedef struct {
  uint16_t psc;
  uint16_t arr;
  uint16_t samples;          /* Table entries used: 1 for DC. */
  float update_hz;           /* Actual sample rate (0 for DC). */
  float actual_frequency_hz; /* Periodic/chirp/arbitrary: period rate. Noise: sample rate. */
  uint32_t actual_dwell_ms;  /* Staircase; 0 otherwise. */
  uint32_t table_crc;        /* CRC-32 of the table the DMA plays. */
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
  float update_hz;
  uint16_t samples;
  uint32_t actual_dwell_ms;
  uint32_t table_crc;
  uint32_t sequence;
  uint32_t lease_deadline_ms;
} daq_dac_channel_state_t;

typedef struct {
  daq_dac_channel_state_t ch[DAQ_DAC_CHANNELS];
  /* Bit c: channel c changed (activity, configuration or result) since the
   * last daq_dac_take_changes(). Drives status and SD event logging. */
  uint8_t changed;
} daq_dac_ctrl_t;

typedef enum {
  DAQ_DAC_TABLE_EMPTY = 0,
  DAQ_DAC_TABLE_LOADING = 1,   /* Accepting chunks. */
  DAQ_DAC_TABLE_READY = 2,     /* Complete and CRC verified: playable. */
  DAQ_DAC_TABLE_CRC_ERROR = 3,
  DAQ_DAC_TABLE_BAD_SAMPLE = 4, /* A shape sample of -32768 (outside -1..1). */
  DAQ_DAC_TABLE_BAD_LENGTH = 5,
  DAQ_DAC_TABLE_VERIFYING = 6  /* Transient: commit computing the CRC. */
} daq_dac_table_state_t;

typedef enum {
  DAQ_DAC_TABLE_OP_BEGIN = 1,
  DAQ_DAC_TABLE_OP_DATA = 2,
  DAQ_DAC_TABLE_OP_COMMIT = 3,
  DAQ_DAC_TABLE_OP_CLEAR = 4
} daq_dac_table_op_t;

typedef struct {
  uint8_t op;
  uint16_t index;     /* BEGIN/COMMIT: table length. DATA: first sample. */
  uint32_t upload_id;
  uint32_t value;     /* BEGIN/COMMIT: expected CRC. DATA: sample count. */
  int16_t samples[DAQ_DAC_TABLE_CHUNK_SAMPLES];
} daq_dac_table_msg_t;

/* The uploaded shape. samples points at DAQ_DAC_ARB_LEN_MAX Q15 values. */
typedef struct {
  uint8_t state;
  uint16_t len;
  uint32_t upload_id;
  uint32_t expected_crc;
  uint32_t crc;
  uint64_t chunks; /* Bit k: samples [32k, 32k + 32) received. */
  int16_t *samples;
} daq_dac_table_t;

/* Gate: DAQ_DAC_RESULT_OK only in Startup or Idle with a known state. */
daq_dac_result_t daq_dac_state_gate(bool has_state, uint8_t state);

int daq_dac_command_decode(const uint8_t *payload, size_t len, daq_dac_command_t *out);
/* Encodes exactly DAQ_DAC_COMMAND_BYTES (host tests and the simulator). */
void daq_dac_command_encode(const daq_dac_command_t *cmd, uint8_t out[DAQ_DAC_COMMAND_BYTES]);

/* Clamps a request to the output range; returns the clamp flags. */
uint8_t daq_dac_clamp(const daq_dac_command_t *cmd, daq_dac_config_t *cfg);

uint16_t daq_dac_volts_to_code(float volts);
float daq_dac_code_to_volts(uint32_t code);
/* Table-generation inputs of an applied configuration. */
void daq_dac_wave_params(const daq_dac_config_t *cfg, daq_dac_wave_params_t *p);

/* Plans the trigger timer (actual frequency/dwell after quantisation) and
 * fills the channel's play table (capacity DAQ_DAC_ARB_LEN_MAX codes). An
 * arbitrary wave scales the verified uploaded shape (shape must be READY
 * with the configured length and CRC). Returns 0, or -1 if unplannable. */
int daq_dac_plan(const daq_dac_config_t *cfg, uint32_t timer_clock_hz, daq_dac_plan_t *plan,
                 uint16_t table[DAQ_DAC_ARB_LEN_MAX], const daq_dac_table_t *shape);
/* Chirp retune elapsed_ms after start, keeping the table length. Returns 0
 * with the new timer split and actual frequency, or -1. */
int daq_dac_chirp_retune(const daq_dac_config_t *cfg, uint16_t samples, uint32_t timer_clock_hz,
                         uint32_t elapsed_ms, uint16_t *psc, uint16_t *arr, float *actual_hz);

int daq_dac_table_decode(const uint8_t *payload, size_t len, daq_dac_table_msg_t *out);
void daq_dac_table_encode(const daq_dac_table_msg_t *msg, uint8_t out[DAQ_DAC_TABLE_BYTES]);
void daq_dac_table_init(daq_dac_table_t *table, int16_t *samples);
/* BEGIN/CLEAR. A table length outside DAQ_DAC_ARB_LEN_MIN..MAX or odd gives
 * DAQ_DAC_TABLE_BAD_LENGTH. */
void daq_dac_table_begin(daq_dac_table_t *table, uint32_t upload_id, uint16_t len, uint32_t crc);
void daq_dac_table_clear(daq_dac_table_t *table);
/* DATA. Returns 0 when stored; -1 when ignored (wrong id/state/offset) or
 * when a code is out of range (the table becomes BAD_SAMPLE). */
int daq_dac_table_data(daq_dac_table_t *table, const daq_dac_table_msg_t *msg);
/* COMMIT, in two halves so the DAQ can stop chunk writes before hashing.
 * Begin returns 1 and moves to VERIFYING when the upload is complete; an
 * incomplete upload stays LOADING so the station resends missing chunks. */
int daq_dac_table_commit_begin(daq_dac_table_t *table, uint32_t upload_id, uint16_t len, uint32_t crc);
void daq_dac_table_commit_finish(daq_dac_table_t *table);
uint64_t daq_dac_table_full_mask(uint16_t len);

void daq_dac_ctrl_init(daq_dac_ctrl_t *ctrl);
/* Applies one decoded command; fills actions[c]. Returns the result code for
 * the addressed channel (all channels share it for DAQ_DAC_CHANNEL_ALL). */
daq_dac_result_t daq_dac_ctrl_command(daq_dac_ctrl_t *ctrl, const daq_dac_command_t *cmd,
                                      const daq_dac_table_t *table, bool has_state, uint8_t state,
                                      uint32_t now_ms, uint8_t actions[DAQ_DAC_CHANNELS]);

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

void daq_dac_status_encode(const daq_dac_ctrl_t *ctrl, const daq_dac_table_t *table,
                           uint8_t flight_state, uint8_t gate,
                           uint8_t flags, uint32_t uptime_ms,
                           const float output_v[DAQ_DAC_CHANNELS],
                           const float readback_v[DAQ_DAC_CHANNELS],
                           uint8_t out[DAQ_DAC_STATUS_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_DAC_CONTROL_H */
