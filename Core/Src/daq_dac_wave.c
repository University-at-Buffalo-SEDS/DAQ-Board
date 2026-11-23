#include "daq_dac_wave.h"

#include <math.h>

/* The table maths must not be contracted into fused multiply-adds: the spec
 * is plain IEEE-754 double arithmetic (see daq_dac_wave.h). */
#if defined(__clang__)
#pragma STDC FP_CONTRACT OFF
#elif defined(__GNUC__)
#pragma GCC optimize("fp-contract=off")
#endif

/* Taylor coefficients of sin(t), t in [0, pi/2]: truncation error < 1e-11,
 * far below one DAC code (2.4e-4 of full scale). Each is a correctly rounded
 * double quotient, so every IEEE-754 implementation gets the same bits. */
static const double k_half_pi = 1.5707963267948966;
static const double k_c3 = -1.0 / 6.0;
static const double k_c5 = 1.0 / 120.0;
static const double k_c7 = -1.0 / 5040.0;
static const double k_c9 = 1.0 / 362880.0;
static const double k_c11 = -1.0 / 39916800.0;
static const double k_c13 = 1.0 / 6227020800.0;
static const double k_c15 = -1.0 / 1307674368000.0;

double daq_dac_wave_sin_turns(double x)
{
  x -= floor(x);
  const double y = 4.0 * x;
  const double q = floor(y);
  double r = y - q;
  const int quadrant = (int)q & 3;
  if (quadrant & 1) r = 1.0 - r;
  const double t = r * k_half_pi;
  const double t2 = t * t;
  double s = k_c15;
  s = k_c13 + t2 * s;
  s = k_c11 + t2 * s;
  s = k_c9 + t2 * s;
  s = k_c7 + t2 * s;
  s = k_c5 + t2 * s;
  s = k_c3 + t2 * s;
  s = 1.0 + t2 * s;
  s = t * s;
  return quadrant >= 2 ? -s : s;
}

double daq_dac_wave_unit(uint8_t waveform, uint8_t flags, uint16_t duty_q16, double x)
{
  switch (waveform) {
    case DAQ_DAC_WAVE_SINE:
    case DAQ_DAC_WAVE_CHIRP: return daq_dac_wave_sin_turns(x);
    case DAQ_DAC_WAVE_SQUARE: return x < (double)duty_q16 / 65536.0 ? 1.0 : -1.0;
    case DAQ_DAC_WAVE_TRIANGLE: return x < 0.5 ? -1.0 + 4.0 * x : 3.0 - 4.0 * x;
    case DAQ_DAC_WAVE_SAWTOOTH:
      return (flags & DAQ_DAC_FLAG_RAMP_DOWN) ? 1.0 - 2.0 * x : -1.0 + 2.0 * x;
    default: return 0.0;
  }
}

uint16_t daq_dac_wave_volts_to_code(double volts)
{
  if (!(volts > 0.0)) return 0U; /* Also NaN. */
  const double code = floor(volts * 4095.0 / DAQ_DAC_VREF_V + 0.5);
  return code >= (double)DAQ_DAC_CODE_MAX ? (uint16_t)DAQ_DAC_CODE_MAX : (uint16_t)code;
}

double daq_dac_wave_code_to_volts(uint32_t code)
{
  if (code > DAQ_DAC_CODE_MAX) code = DAQ_DAC_CODE_MAX;
  return (double)code * DAQ_DAC_VREF_V / 4095.0;
}

int daq_dac_wave_fill(const daq_dac_wave_params_t *p, uint16_t n, uint16_t *out)
{
  if (!p || !out || n == 0U) return -1;
  const double a = (double)p->amplitude_v;
  const double o = (double)p->offset_v;
  switch (p->waveform) {
    case DAQ_DAC_WAVE_DC:
      for (unsigned i = 0U; i < n; ++i) out[i] = daq_dac_wave_volts_to_code(o);
      return 0;
    case DAQ_DAC_WAVE_STAIRCASE:
      if (n < 2U) return -1;
      for (unsigned i = 0U; i < n; ++i)
        out[i] = daq_dac_wave_volts_to_code(o - a + 2.0 * a * (double)i / (double)(n - 1U));
      return 0;
    case DAQ_DAC_WAVE_NOISE: {
      uint32_t s = DAQ_DAC_NOISE_SEED;
      for (unsigned i = 0U; i < n; ++i) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        const double u = (double)(s >> 8) / 8388608.0 - 1.0;
        out[i] = daq_dac_wave_volts_to_code(o + a * u);
      }
      return 0;
    }
    case DAQ_DAC_WAVE_SQUARE:
    case DAQ_DAC_WAVE_SINE:
    case DAQ_DAC_WAVE_TRIANGLE:
    case DAQ_DAC_WAVE_SAWTOOTH:
    case DAQ_DAC_WAVE_CHIRP: {
      const double phase = (double)p->phase_q16 / 65536.0;
      for (unsigned i = 0U; i < n; ++i) {
        double x = (double)i / (double)n + phase;
        x -= floor(x);
        out[i] = daq_dac_wave_volts_to_code(o + a * daq_dac_wave_unit(p->waveform, p->flags, p->duty_q16, x));
      }
      return 0;
    }
    default: return -1;
  }
}

int daq_dac_wave_fill_shape(const daq_dac_wave_params_t *p, const int16_t *shape, uint16_t n,
                            uint16_t *out)
{
  if (!p || !shape || !out || n == 0U) return -1;
  const double a = (double)p->amplitude_v;
  const double o = (double)p->offset_v;
  const uint32_t rot = (((uint32_t)p->phase_q16 * n + 32768U) >> 16) % n;
  for (unsigned i = 0U; i < n; ++i) {
    const double u = (double)shape[(i + rot) % n] / 32767.0;
    out[i] = daq_dac_wave_volts_to_code(o + a * u);
  }
  return 0;
}

uint32_t daq_dac_crc32_update(uint32_t crc, const uint8_t *bytes, size_t len)
{
  crc = ~crc;
  for (size_t i = 0U; i < len; ++i) {
    crc ^= bytes[i];
    for (unsigned b = 0U; b < 8U; ++b) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

uint32_t daq_dac_wave_crc32(const uint16_t *codes, size_t n)
{
  uint32_t crc = 0U;
  for (size_t i = 0U; i < n; ++i) {
    const uint8_t le[2] = {(uint8_t)codes[i], (uint8_t)(codes[i] >> 8U)};
    crc = daq_dac_crc32_update(crc, le, 2U);
  }
  return crc;
}

static double split_tries(uint32_t clock_hz, double rate_hz, double tries, uint16_t *psc, uint16_t *arr)
{
  if (clock_hz == 0U || !(rate_hz >= DAQ_DAC_UPDATE_MIN_HZ) || !(rate_hz <= DAQ_DAC_UPDATE_MAX_HZ))
    return 0.0;
  const double clock = (double)clock_hz;
  const double total = clock / rate_hz;
  if (!(total >= 2.0)) return 0.0;
  const double first = floor((total - 1.0) / 65536.0);
  double best_rate = 0.0, best_err = 0.0;
  for (double p = first; p < first + tries && p <= 65535.0; p += 1.0) {
    const double div = p + 1.0;
    double a = floor(total / div + 0.5);
    if (a < 2.0) a = 2.0;
    if (a > 65536.0) a = 65536.0;
    const double achieved = clock / (div * a);
    const double err = fabs(achieved - rate_hz);
    if (best_rate == 0.0 || err < best_err) {
      best_rate = achieved; best_err = err;
      if (psc) *psc = (uint16_t)p;
      if (arr) *arr = (uint16_t)(a - 1.0);
    }
  }
  return best_rate;
}

double daq_dac_wave_split(uint32_t clock_hz, double rate_hz, uint16_t *psc, uint16_t *arr)
{
  return split_tries(clock_hz, rate_hz, (double)DAQ_DAC_SPLIT_TRIES, psc, arr);
}

int daq_dac_wave_choose(uint32_t clock_hz, double frequency_hz, uint16_t n_min, uint16_t n_max,
                        daq_dac_rate_t *out)
{
  if (!out || !(frequency_hz > 0.0) || n_min < 2U || n_max < n_min) return -1;
  int found = 0;
  double best_err = 0.0;
  for (unsigned n = n_max & ~1U; n >= n_min && n >= 2U; n -= 2U) {
    const double rate = frequency_hz * (double)n;
    if (rate > DAQ_DAC_UPDATE_MAX_HZ || rate < DAQ_DAC_UPDATE_MIN_HZ) continue;
    uint16_t psc = 0U, arr = 0U;
    const double achieved = split_tries(clock_hz, rate, (double)DAQ_DAC_CHOOSE_SPLIT_TRIES, &psc, &arr);
    if (achieved == 0.0) continue;
    const double f = achieved / (double)n;
    const double err = fabs(f - frequency_hz);
    if (!found || err < best_err) {
      found = 1; best_err = err;
      out->samples = (uint16_t)n; out->psc = psc; out->arr = arr;
      out->update_hz = achieved; out->frequency_hz = f;
    }
    /* Lengths are tried longest first: stop at the first one inside the
     * tolerance, since a longer table has less quantisation distortion. */
    if (err <= frequency_hz * DAQ_DAC_FREQ_TOLERANCE) break;
  }
  return found ? 0 : -1;
}

double daq_dac_wave_chirp_hz(double start_hz, double stop_hz, uint32_t sweep_ms, uint8_t flags,
                             uint32_t elapsed_ms)
{
  if (sweep_ms == 0U) return start_hz;
  const double frac = (double)(elapsed_ms % sweep_ms) / (double)sweep_ms;
  if ((flags & DAQ_DAC_FLAG_LOG_SWEEP) && start_hz > 0.0 && stop_hz > 0.0)
    return start_hz * (double)powf((float)(stop_hz / start_hz), (float)frac);
  return start_hz + (stop_hz - start_hz) * frac;
}

uint16_t daq_dac_wave_chirp_len(double start_hz, double stop_hz)
{
  const double top = start_hz > stop_hz ? start_hz : stop_hz;
  if (!(top > 0.0)) return DAQ_DAC_PERIODIC_LEN_MIN;
  const double fit = floor(DAQ_DAC_UPDATE_MAX_HZ / top);
  unsigned n = fit >= (double)DAQ_DAC_PERIODIC_LEN_MAX ? DAQ_DAC_PERIODIC_LEN_MAX : (unsigned)fit;
  n &= ~1U;
  return (uint16_t)(n < DAQ_DAC_PERIODIC_LEN_MIN ? DAQ_DAC_PERIODIC_LEN_MIN : n);
}
