#ifndef DAQ_DAC_WAVE_H
#define DAQ_DAC_WAVE_H

/* DAC waveform specification: sample generation, code conversion, table CRC
 * and trigger-rate selection. docs/dac-waveforms.md is the normative text;
 * this file is its reference implementation. It has no HAL, RTOS or SEDSnet
 * dependency, so the firmware, the host tests and the firmware simulator
 * (virtual DAC->ADC patching) all compile the same code.
 *
 * Determinism: every table value is computed with IEEE-754 double +, -, *, /
 * and floor only (no libm transcendental, no FMA contraction), so the
 * GroundStation can regenerate a table bit for bit and compare its CRC with
 * the one the DAQ reports in DAQ_DAC_STATUS. */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DAQ_DAC_WAVE_DC = 0,
  DAQ_DAC_WAVE_SQUARE = 1,
  DAQ_DAC_WAVE_SINE = 2,
  DAQ_DAC_WAVE_TRIANGLE = 3,
  DAQ_DAC_WAVE_STAIRCASE = 4,
  DAQ_DAC_WAVE_SAWTOOTH = 5,  /* Ramp up; DAQ_DAC_FLAG_RAMP_DOWN for down. */
  DAQ_DAC_WAVE_CHIRP = 6,     /* Sine whose frequency the DAQ retunes. */
  DAQ_DAC_WAVE_NOISE = 7,     /* Repeating 256-sample xorshift32 sequence. */
  DAQ_DAC_WAVE_ARBITRARY = 8, /* GroundStation-uploaded normalized shape. */
  DAQ_DAC_WAVE_COUNT = 9
} daq_dac_waveform_t;

/* Wave flags (command byte 46, status channel byte 5). */
#define DAQ_DAC_FLAG_RAMP_DOWN 0x01U /* Sawtooth falls instead of rising. */
#define DAQ_DAC_FLAG_LOG_SWEEP 0x02U /* Chirp sweeps logarithmically. */
#define DAQ_DAC_FLAGS_KNOWN (DAQ_DAC_FLAG_RAMP_DOWN | DAQ_DAC_FLAG_LOG_SWEEP)

#define DAQ_DAC_CODE_MAX 4095U
/* LQFP64 has no VREF+ pin: VREF+ is VDDA (+3V3A). With the output buffer on,
 * the datasheet linear output range is 0.2 V .. VREF+ - 0.2 V. */
#define DAQ_DAC_VREF_V 3.3
#define DAQ_DAC_MIN_V 0.20
#define DAQ_DAC_MAX_V 3.10
/* daq_dac_wave_volts_to_code(DAQ_DAC_MIN_V / DAQ_DAC_MAX_V): the only codes
 * an uploaded table may contain. */
#define DAQ_DAC_CODE_LO 248U
#define DAQ_DAC_CODE_HI 3847U

/* Trigger timers (TIM7 on APB1, TIM15 on APB2) run at 160 MHz on this board. */
#define DAQ_DAC_TIMER_CLOCK_HZ 160000000U
/* Highest DAC update rate per channel. The buffered DAC settles a full-scale
 * step in a few microseconds; 200 kS/s leaves margin and keeps GPDMA load
 * negligible next to the ADC/SPI channels. */
#define DAQ_DAC_UPDATE_MAX_HZ 200000.0
/* Lowest planned update rate: total timer counts per sample stay below 2^32. */
#define DAQ_DAC_UPDATE_MIN_HZ 0.05

/* Built-in periodic tables: an even sample count in this range is chosen to
 * minimise frequency error (see daq_dac_wave_choose). */
#define DAQ_DAC_PERIODIC_LEN_MIN 16U
#define DAQ_DAC_PERIODIC_LEN_MAX 256U
#define DAQ_DAC_NOISE_LEN 256U
#define DAQ_DAC_NOISE_SEED 0x2545F491U
/* Uploaded shapes: int16 Q15 samples, -32767..32767 = -1..1 (512 x 2 B = 1 KiB).
 * Each channel's play table holds up to this many codes as well. */
#define DAQ_DAC_ARB_LEN_MIN 16U
#define DAQ_DAC_ARB_LEN_MAX 512U
#define DAQ_DAC_SHAPE_MAX 32767

#define DAQ_DAC_FREQ_MIN_HZ 0.01
#define DAQ_DAC_FREQ_MAX_HZ 10000.0
#define DAQ_DAC_NOISE_RATE_MIN_HZ 1.0
#define DAQ_DAC_SWEEP_MIN_MS 100U
#define DAQ_DAC_SWEEP_MAX_MS 600000U
/* The DAQ retunes a chirp at each 50 ms service tick. */
#define DAQ_DAC_CHIRP_STEP_MS 50U
#define DAQ_DAC_STEPS_MIN 2U
#define DAQ_DAC_STEPS_MAX DAQ_DAC_PERIODIC_LEN_MAX
#define DAQ_DAC_DWELL_MIN_MS 10U
#define DAQ_DAC_DWELL_MAX_MS 20000U
#define DAQ_DAC_DUTY_DEFAULT_Q16 32768U

/* Inputs of table generation, exactly as the DAQ applied them (status
 * reports every one of them). */
typedef struct {
  uint8_t waveform;
  uint8_t flags;
  uint16_t steps;     /* Staircase. */
  uint16_t phase_q16; /* Start phase, turns x 65536. */
  uint16_t duty_q16;  /* Square high fraction x 65536. */
  float amplitude_v;  /* DAC-pin volts (peak). */
  float offset_v;     /* DAC-pin volts. */
} daq_dac_wave_params_t;

/* sin(2*pi*x) for any finite x, using only basic double arithmetic. */
double daq_dac_wave_sin_turns(double x);
/* Unit shape (-1..1) of a periodic waveform at phase x in [0, 1). */
double daq_dac_wave_unit(uint8_t waveform, uint8_t flags, uint16_t duty_q16, double x);
/* floor(v * 4095 / 3.3 + 0.5) saturated to 0..4095 (NaN and v <= 0 give 0). */
uint16_t daq_dac_wave_volts_to_code(double volts);
double daq_dac_wave_code_to_volts(uint32_t code);

/* Fills n codes for a built-in waveform. n is the table length chosen by
 * daq_dac_wave_choose (periodic), steps (staircase), DAQ_DAC_NOISE_LEN
 * (noise) or 1 (DC). Returns 0, or -1 for an arbitrary waveform or bad n. */
int daq_dac_wave_fill(const daq_dac_wave_params_t *p, uint16_t n, uint16_t *out);

/* Codes for an uploaded shape s[0..n-1], scaled like a built-in wave:
 * v_i = o + a * (s[j] / 32767) with j = (i + rot) mod n and
 * rot = ((phase_q16 * n + 32768) >> 16) mod n. Returns 0 or -1. */
int daq_dac_wave_fill_shape(const daq_dac_wave_params_t *p, const int16_t *shape, uint16_t n,
                            uint16_t *out);

/* CRC-32 (IEEE 802.3, reflected 0xEDB88320, init and xorout 0xFFFFFFFF) of
 * the codes as little-endian uint16 bytes. */
uint32_t daq_dac_wave_crc32(const uint16_t *codes, size_t n);
uint32_t daq_dac_crc32_update(uint32_t crc, const uint8_t *bytes, size_t len);

/* Prescaler candidates tried from the smallest one that fits ARR: a lone
 * split (staircase, noise, chirp, uploaded table) tries 64, the table-length
 * search 3 per length (it has 121 lengths to vary instead). */
#define DAQ_DAC_SPLIT_TRIES 64U
#define DAQ_DAC_CHOOSE_SPLIT_TRIES 3U

/* Splits clock/rate into 16-bit PSC and ARR register values (counts per
 * sample = (psc + 1) * (arr + 1)), trying DAQ_DAC_SPLIT_TRIES prescalers for
 * the smallest rate error (ties keep the smaller prescaler). Returns the
 * achieved rate, or 0 when the rate is outside
 * DAQ_DAC_UPDATE_MIN_HZ..DAQ_DAC_UPDATE_MAX_HZ. */
double daq_dac_wave_split(uint32_t clock_hz, double rate_hz, uint16_t *psc, uint16_t *arr);

typedef struct {
  uint16_t samples;
  uint16_t psc;
  uint16_t arr;
  double update_hz;
  double frequency_hz; /* update_hz / samples */
} daq_dac_rate_t;

/* Relative frequency error accepted without shortening the table: below
 * the timer crystal's own tolerance. */
#define DAQ_DAC_FREQ_TOLERANCE 1e-5

/* Chooses an even table length in [n_min, n_max] and its timer split: the
 * longest length whose frequency error is within DAQ_DAC_FREQ_TOLERANCE,
 * else the length with the least error (ties keep the longer table).
 * Update rate stays within the limits above. Returns 0 or -1. */
int daq_dac_wave_choose(uint32_t clock_hz, double frequency_hz, uint16_t n_min, uint16_t n_max,
                        daq_dac_rate_t *out);

/* Instantaneous chirp frequency elapsed_ms into a sweep that repeats every
 * sweep_ms: linear, or logarithmic with DAQ_DAC_FLAG_LOG_SWEEP. */
double daq_dac_wave_chirp_hz(double start_hz, double stop_hz, uint32_t sweep_ms, uint8_t flags,
                             uint32_t elapsed_ms);
/* Table length for a chirp: the longest even length whose update rate stays
 * within the limit at the sweep's highest frequency. */
uint16_t daq_dac_wave_chirp_len(double start_hz, double stop_hz);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_DAC_WAVE_H */
