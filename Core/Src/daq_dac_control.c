#include "daq_dac_control.h"

#include <math.h>
#include <string.h>

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8U)); }
static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}
static float rdf(const uint8_t *p) { const uint32_t w = rd32(p); float f; memcpy(&f, &w, 4U); return f; }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U); }
static void wr32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U); p[2] = (uint8_t)(v >> 16U); p[3] = (uint8_t)(v >> 24U);
}
static void wrf(uint8_t *p, float f) { uint32_t w; memcpy(&w, &f, 4U); wr32(p, w); }

daq_dac_result_t daq_dac_state_gate(bool has_state, uint8_t state)
{
  if (!has_state || state == DAQ_DAC_STATE_UNKNOWN) return DAQ_DAC_RESULT_REFUSED_UNKNOWN_STATE;
  return (state == DAQ_DAC_STATE_STARTUP || state == DAQ_DAC_STATE_IDLE)
      ? DAQ_DAC_RESULT_OK : DAQ_DAC_RESULT_REFUSED_STATE;
}

static void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32U)); }

int daq_dac_command_decode(const uint8_t *p, size_t len, daq_dac_command_t *out)
{
  if (!p || !out || len != DAQ_DAC_COMMAND_BYTES || p[0] != DAQ_DAC_COMMAND_VERSION) return -1;
  memset(out, 0, sizeof(*out));
  out->channel = p[1];
  out->enable = p[2];
  out->waveform = p[3];
  out->amplitude_v = rdf(p + 4);
  out->offset_v = rdf(p + 8);
  out->frequency_hz = rdf(p + 12);
  out->steps = rd16(p + 16);
  out->lease_ms = rd16(p + 18);
  out->dwell_ms = rd32(p + 20);
  out->sequence = rd32(p + 24);
  out->phase_q16 = rd16(p + 28);
  out->duty_q16 = rd16(p + 30);
  out->frequency_stop_hz = rdf(p + 32);
  out->sweep_ms = rd32(p + 36);
  out->table_crc = rd32(p + 40);
  out->table_len = rd16(p + 44);
  out->flags = p[46];
  if (out->enable > 1U) return -1;
  if (out->channel >= DAQ_DAC_CHANNELS && out->channel != DAQ_DAC_CHANNEL_ALL) return -1;
  /* "All" exists only to turn everything off. */
  if (out->channel == DAQ_DAC_CHANNEL_ALL && out->enable) return -1;
  return 0;
}

void daq_dac_command_encode(const daq_dac_command_t *c, uint8_t out[DAQ_DAC_COMMAND_BYTES])
{
  memset(out, 0, DAQ_DAC_COMMAND_BYTES);
  out[0] = DAQ_DAC_COMMAND_VERSION; out[1] = c->channel; out[2] = c->enable; out[3] = c->waveform;
  wrf(out + 4, c->amplitude_v); wrf(out + 8, c->offset_v); wrf(out + 12, c->frequency_hz);
  wr16(out + 16, c->steps); wr16(out + 18, c->lease_ms);
  wr32(out + 20, c->dwell_ms); wr32(out + 24, c->sequence);
  wr16(out + 28, c->phase_q16); wr16(out + 30, c->duty_q16);
  wrf(out + 32, c->frequency_stop_hz); wr32(out + 36, c->sweep_ms);
  wr32(out + 40, c->table_crc); wr16(out + 44, c->table_len); out[46] = c->flags;
}

static bool is_periodic(uint8_t w)
{
  return w == DAQ_DAC_WAVE_SQUARE || w == DAQ_DAC_WAVE_SINE || w == DAQ_DAC_WAVE_TRIANGLE ||
         w == DAQ_DAC_WAVE_SAWTOOTH;
}

static float clampf_flag(float v, float lo, float hi, uint8_t bit, uint8_t *flags)
{
  if (v < lo) { *flags |= bit; return lo; }
  if (v > hi) { *flags |= bit; return hi; }
  return v;
}

uint8_t daq_dac_clamp(const daq_dac_command_t *cmd, daq_dac_config_t *cfg)
{
  uint8_t flags = 0U;
  memset(cfg, 0, sizeof(*cfg));
  const uint8_t w = cmd->waveform;
  cfg->waveform = w;

  cfg->offset_v = clampf_flag(cmd->offset_v, (float)DAQ_DAC_MIN_V, (float)DAQ_DAC_MAX_V,
                              DAQ_DAC_CLAMP_OFFSET, &flags);
  if (w != DAQ_DAC_WAVE_DC) {
    const float o = cfg->offset_v;
    const float headroom = fminf(o - (float)DAQ_DAC_MIN_V, (float)DAQ_DAC_MAX_V - o);
    cfg->amplitude_v = clampf_flag(cmd->amplitude_v, 0.0f, headroom, DAQ_DAC_CLAMP_AMPLITUDE, &flags);
  }
  if (w == DAQ_DAC_WAVE_ARBITRARY) {
    /* A normalized shape scaled like a built-in wave; the update rate
     * (frequency x length) stays within the DAC limit. */
    cfg->table_len = cmd->table_len;
    cfg->table_crc = cmd->table_crc;
    cfg->phase_q16 = cmd->phase_q16;
    const float top = cmd->table_len ? (float)(DAQ_DAC_UPDATE_MAX_HZ / (double)cmd->table_len)
                                     : (float)DAQ_DAC_FREQ_MAX_HZ;
    cfg->frequency_hz = clampf_flag(cmd->frequency_hz, (float)DAQ_DAC_FREQ_MIN_HZ,
                                    fminf(top, (float)DAQ_DAC_FREQ_MAX_HZ),
                                    DAQ_DAC_CLAMP_FREQUENCY, &flags);
  }

  if (is_periodic(w) || w == DAQ_DAC_WAVE_CHIRP) {
    cfg->frequency_hz = clampf_flag(cmd->frequency_hz, (float)DAQ_DAC_FREQ_MIN_HZ,
                                    (float)DAQ_DAC_FREQ_MAX_HZ, DAQ_DAC_CLAMP_FREQUENCY, &flags);
    cfg->phase_q16 = cmd->phase_q16;
  }
  if (w == DAQ_DAC_WAVE_SQUARE) {
    uint16_t duty = cmd->duty_q16;
    if (duty < DAQ_DAC_DUTY_MIN_Q16) { duty = DAQ_DAC_DUTY_MIN_Q16; flags |= DAQ_DAC_CLAMP_DUTY; }
    else if (duty > DAQ_DAC_DUTY_MAX_Q16) { duty = DAQ_DAC_DUTY_MAX_Q16; flags |= DAQ_DAC_CLAMP_DUTY; }
    cfg->duty_q16 = duty;
  }
  if (w == DAQ_DAC_WAVE_SAWTOOTH) cfg->flags = cmd->flags & DAQ_DAC_FLAG_RAMP_DOWN;
  if (w == DAQ_DAC_WAVE_CHIRP) {
    cfg->flags = cmd->flags & DAQ_DAC_FLAG_LOG_SWEEP;
    cfg->frequency_stop_hz = clampf_flag(cmd->frequency_stop_hz, (float)DAQ_DAC_FREQ_MIN_HZ,
                                         (float)DAQ_DAC_FREQ_MAX_HZ, DAQ_DAC_CLAMP_FREQUENCY, &flags);
    uint32_t sweep = cmd->sweep_ms;
    if (sweep < DAQ_DAC_SWEEP_MIN_MS) { sweep = DAQ_DAC_SWEEP_MIN_MS; flags |= DAQ_DAC_CLAMP_SWEEP; }
    else if (sweep > DAQ_DAC_SWEEP_MAX_MS) { sweep = DAQ_DAC_SWEEP_MAX_MS; flags |= DAQ_DAC_CLAMP_SWEEP; }
    cfg->sweep_ms = sweep;
  }
  if (w == DAQ_DAC_WAVE_NOISE)
    cfg->frequency_hz = clampf_flag(cmd->frequency_hz, (float)DAQ_DAC_NOISE_RATE_MIN_HZ,
                                    (float)DAQ_DAC_UPDATE_MAX_HZ, DAQ_DAC_CLAMP_FREQUENCY, &flags);

  if (w == DAQ_DAC_WAVE_STAIRCASE) {
    uint16_t steps = cmd->steps;
    if (steps < DAQ_DAC_STEPS_MIN) { steps = DAQ_DAC_STEPS_MIN; flags |= DAQ_DAC_CLAMP_STEPS; }
    else if (steps > DAQ_DAC_STEPS_MAX) { steps = DAQ_DAC_STEPS_MAX; flags |= DAQ_DAC_CLAMP_STEPS; }
    cfg->steps = steps;
    uint32_t dwell = cmd->dwell_ms;
    if (dwell < DAQ_DAC_DWELL_MIN_MS) { dwell = DAQ_DAC_DWELL_MIN_MS; flags |= DAQ_DAC_CLAMP_DWELL; }
    else if (dwell > DAQ_DAC_DWELL_MAX_MS) { dwell = DAQ_DAC_DWELL_MAX_MS; flags |= DAQ_DAC_CLAMP_DWELL; }
    cfg->dwell_ms = dwell;
  }

  uint16_t lease = cmd->lease_ms == 0U ? (uint16_t)DAQ_DAC_LEASE_DEFAULT_MS : cmd->lease_ms;
  if (lease < DAQ_DAC_LEASE_MIN_MS) { lease = (uint16_t)DAQ_DAC_LEASE_MIN_MS; flags |= DAQ_DAC_CLAMP_LEASE; }
  else if (lease > DAQ_DAC_LEASE_MAX_MS) { lease = (uint16_t)DAQ_DAC_LEASE_MAX_MS; flags |= DAQ_DAC_CLAMP_LEASE; }
  cfg->lease_ms = lease;
  cfg->clamp_flags = flags;
  return flags;
}

uint16_t daq_dac_volts_to_code(float volts)
{
  return daq_dac_wave_volts_to_code((double)volts);
}

float daq_dac_code_to_volts(uint32_t code)
{
  return (float)daq_dac_wave_code_to_volts(code);
}

void daq_dac_wave_params(const daq_dac_config_t *cfg, daq_dac_wave_params_t *p)
{
  memset(p, 0, sizeof(*p));
  p->waveform = cfg->waveform;
  p->flags = cfg->flags;
  p->steps = cfg->steps;
  p->phase_q16 = cfg->phase_q16;
  p->duty_q16 = cfg->duty_q16;
  p->amplitude_v = cfg->amplitude_v;
  p->offset_v = cfg->offset_v;
}

int daq_dac_plan(const daq_dac_config_t *cfg, uint32_t timer_clock_hz, daq_dac_plan_t *plan,
                 uint16_t table[DAQ_DAC_ARB_LEN_MAX], const daq_dac_table_t *shape)
{
  if (!cfg || !plan || !table || cfg->waveform >= DAQ_DAC_WAVE_COUNT || timer_clock_hz == 0U) return -1;
  memset(plan, 0, sizeof(*plan));
  const uint8_t w = cfg->waveform;
  uint16_t n = 0U;
  double rate = 0.0;
  daq_dac_rate_t chosen = {0};
  switch (w) {
    case DAQ_DAC_WAVE_DC: n = 1U; break;
    case DAQ_DAC_WAVE_STAIRCASE:
      n = cfg->steps;
      if (n < DAQ_DAC_STEPS_MIN || n > DAQ_DAC_STEPS_MAX || cfg->dwell_ms == 0U) return -1;
      rate = 1000.0 / (double)cfg->dwell_ms;
      break;
    case DAQ_DAC_WAVE_NOISE:
      n = DAQ_DAC_NOISE_LEN;
      rate = (double)cfg->frequency_hz;
      break;
    case DAQ_DAC_WAVE_CHIRP:
      n = daq_dac_wave_chirp_len((double)cfg->frequency_hz, (double)cfg->frequency_stop_hz);
      rate = (double)cfg->frequency_hz * (double)n;
      break;
    case DAQ_DAC_WAVE_ARBITRARY:
      n = cfg->table_len;
      if (n < DAQ_DAC_ARB_LEN_MIN || n > DAQ_DAC_ARB_LEN_MAX || (n & 1U)) return -1;
      rate = (double)cfg->frequency_hz * (double)n;
      break;
    default:
      if (!(cfg->frequency_hz > 0.0f)) return -1;
      if (daq_dac_wave_choose(timer_clock_hz, (double)cfg->frequency_hz, DAQ_DAC_PERIODIC_LEN_MIN,
                              DAQ_DAC_PERIODIC_LEN_MAX, &chosen) != 0)
        return -1;
      n = chosen.samples;
      break;
  }
  daq_dac_wave_params_t p;
  daq_dac_wave_params(cfg, &p);
  if (w == DAQ_DAC_WAVE_ARBITRARY) {
    if (!shape || !shape->samples || shape->state != DAQ_DAC_TABLE_READY || shape->len != n ||
        shape->crc != cfg->table_crc || daq_dac_wave_fill_shape(&p, shape->samples, n, table) != 0)
      return -1;
  } else if (daq_dac_wave_fill(&p, n, table) != 0) {
    return -1;
  }
  plan->table_crc = daq_dac_wave_crc32(table, n);
  plan->samples = n;
  if (n == 1U) return 0;

  if (is_periodic(w)) {
    plan->psc = chosen.psc;
    plan->arr = chosen.arr;
    plan->update_hz = (float)chosen.update_hz;
    plan->actual_frequency_hz = (float)chosen.frequency_hz;
    return 0;
  }
  const double achieved = daq_dac_wave_split(timer_clock_hz, rate, &plan->psc, &plan->arr);
  if (achieved == 0.0) return -1;
  plan->update_hz = (float)achieved;
  if (w == DAQ_DAC_WAVE_STAIRCASE) plan->actual_dwell_ms = (uint32_t)(1000.0 / achieved + 0.5);
  else if (w == DAQ_DAC_WAVE_NOISE) plan->actual_frequency_hz = (float)achieved;
  else plan->actual_frequency_hz = (float)(achieved / (double)n);
  return 0;
}

int daq_dac_chirp_retune(const daq_dac_config_t *cfg, uint16_t samples, uint32_t timer_clock_hz,
                         uint32_t elapsed_ms, uint16_t *psc, uint16_t *arr, float *actual_hz)
{
  if (!cfg || cfg->waveform != DAQ_DAC_WAVE_CHIRP || samples == 0U) return -1;
  const double f = daq_dac_wave_chirp_hz((double)cfg->frequency_hz, (double)cfg->frequency_stop_hz,
                                         cfg->sweep_ms, cfg->flags, elapsed_ms);
  const double achieved = daq_dac_wave_split(timer_clock_hz, f * (double)samples, psc, arr);
  if (achieved == 0.0) return -1;
  if (actual_hz) *actual_hz = (float)(achieved / (double)samples);
  return 0;
}

int daq_dac_table_decode(const uint8_t *p, size_t len, daq_dac_table_msg_t *out)
{
  if (!p || !out || len != DAQ_DAC_TABLE_BYTES || p[0] != DAQ_DAC_TABLE_VERSION) return -1;
  memset(out, 0, sizeof(*out));
  out->op = p[1];
  out->index = rd16(p + 2);
  out->upload_id = rd32(p + 4);
  out->value = rd32(p + 8);
  if (out->op < DAQ_DAC_TABLE_OP_BEGIN || out->op > DAQ_DAC_TABLE_OP_CLEAR) return -1;
  if (out->op == DAQ_DAC_TABLE_OP_DATA) {
    if (out->value == 0U || out->value > DAQ_DAC_TABLE_CHUNK_SAMPLES) return -1;
    for (unsigned i = 0U; i < out->value; ++i) out->samples[i] = (int16_t)rd16(p + 12U + 2U * i);
  }
  return 0;
}

void daq_dac_table_encode(const daq_dac_table_msg_t *m, uint8_t out[DAQ_DAC_TABLE_BYTES])
{
  memset(out, 0, DAQ_DAC_TABLE_BYTES);
  out[0] = DAQ_DAC_TABLE_VERSION; out[1] = m->op;
  wr16(out + 2, m->index); wr32(out + 4, m->upload_id); wr32(out + 8, m->value);
  if (m->op == DAQ_DAC_TABLE_OP_DATA)
    for (unsigned i = 0U; i < m->value && i < DAQ_DAC_TABLE_CHUNK_SAMPLES; ++i)
      wr16(out + 12U + 2U * i, (uint16_t)m->samples[i]);
}

void daq_dac_table_init(daq_dac_table_t *t, int16_t *samples)
{
  memset(t, 0, sizeof(*t));
  t->samples = samples;
}

uint64_t daq_dac_table_full_mask(uint16_t len)
{
  const unsigned chunks = (len + DAQ_DAC_TABLE_CHUNK_SAMPLES - 1U) / DAQ_DAC_TABLE_CHUNK_SAMPLES;
  return chunks >= 64U ? ~(uint64_t)0 : (((uint64_t)1 << chunks) - 1U);
}

void daq_dac_table_begin(daq_dac_table_t *t, uint32_t upload_id, uint16_t len, uint32_t crc)
{
  t->upload_id = upload_id;
  t->len = len;
  t->expected_crc = crc;
  t->crc = 0U;
  t->chunks = 0U;
  t->state = (len < DAQ_DAC_ARB_LEN_MIN || len > DAQ_DAC_ARB_LEN_MAX || (len & 1U))
      ? DAQ_DAC_TABLE_BAD_LENGTH : DAQ_DAC_TABLE_LOADING;
}

void daq_dac_table_clear(daq_dac_table_t *t)
{
  int16_t *samples = t->samples;
  daq_dac_table_init(t, samples);
}

int daq_dac_table_data(daq_dac_table_t *t, const daq_dac_table_msg_t *m)
{
  if (t->state != DAQ_DAC_TABLE_LOADING || m->op != DAQ_DAC_TABLE_OP_DATA ||
      m->upload_id != t->upload_id || !t->samples)
    return -1;
  const unsigned at = m->index;
  if (at % DAQ_DAC_TABLE_CHUNK_SAMPLES != 0U || at >= t->len) return -1;
  const unsigned want = t->len - at < DAQ_DAC_TABLE_CHUNK_SAMPLES ? t->len - at : DAQ_DAC_TABLE_CHUNK_SAMPLES;
  if (m->value != want) return -1;
  for (unsigned i = 0U; i < want; ++i) {
    if (m->samples[i] < -DAQ_DAC_SHAPE_MAX) {
      t->state = DAQ_DAC_TABLE_BAD_SAMPLE;
      return -1;
    }
  }
  memcpy(t->samples + at, m->samples, want * sizeof(uint16_t));
  t->chunks |= (uint64_t)1 << (at / DAQ_DAC_TABLE_CHUNK_SAMPLES);
  return 0;
}

int daq_dac_table_commit_begin(daq_dac_table_t *t, uint32_t upload_id, uint16_t len, uint32_t crc)
{
  if (t->state != DAQ_DAC_TABLE_LOADING || upload_id != t->upload_id || len != t->len ||
      crc != t->expected_crc || t->chunks != daq_dac_table_full_mask(t->len))
    return 0;
  t->state = DAQ_DAC_TABLE_VERIFYING;
  return 1;
}

void daq_dac_table_commit_finish(daq_dac_table_t *t)
{
  if (t->state != DAQ_DAC_TABLE_VERIFYING) return;
  t->crc = daq_dac_wave_crc32((const uint16_t *)t->samples, t->len);
  t->state = t->crc == t->expected_crc ? DAQ_DAC_TABLE_READY : DAQ_DAC_TABLE_CRC_ERROR;
}

void daq_dac_ctrl_init(daq_dac_ctrl_t *ctrl)
{
  memset(ctrl, 0, sizeof(*ctrl));
}

static void turn_off(daq_dac_ctrl_t *ctrl, unsigned c, uint8_t reason, uint8_t actions[DAQ_DAC_CHANNELS])
{
  if (!ctrl->ch[c].active) return;
  ctrl->ch[c].active = 0U;
  ctrl->ch[c].off_reason = reason;
  ctrl->changed |= (uint8_t)(1U << c);
  actions[c] = DAQ_DAC_ACTION_STOP;
}

static bool same_config(const daq_dac_config_t *a, const daq_dac_config_t *b)
{
  return a->waveform == b->waveform && a->steps == b->steps && a->amplitude_v == b->amplitude_v &&
         a->offset_v == b->offset_v && a->frequency_hz == b->frequency_hz && a->dwell_ms == b->dwell_ms &&
         a->flags == b->flags && a->phase_q16 == b->phase_q16 && a->duty_q16 == b->duty_q16 &&
         a->frequency_stop_hz == b->frequency_stop_hz && a->sweep_ms == b->sweep_ms &&
         a->table_len == b->table_len && a->table_crc == b->table_crc;
}

daq_dac_result_t daq_dac_ctrl_command(daq_dac_ctrl_t *ctrl, const daq_dac_command_t *cmd,
                                      const daq_dac_table_t *table, bool has_state, uint8_t state,
                                      uint32_t now_ms, uint8_t actions[DAQ_DAC_CHANNELS])
{
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) actions[c] = DAQ_DAC_ACTION_NONE;
  if (!ctrl || !cmd) return DAQ_DAC_RESULT_BAD_REQUEST;
  if (cmd->channel == DAQ_DAC_CHANNEL_ALL || !cmd->enable) {
    for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
      if (cmd->channel != DAQ_DAC_CHANNEL_ALL && cmd->channel != c) continue;
      turn_off(ctrl, c, DAQ_DAC_OFF_COMMANDED, actions);
      if (ctrl->ch[c].last_result != DAQ_DAC_RESULT_OK || ctrl->ch[c].sequence != cmd->sequence)
        ctrl->changed |= (uint8_t)(1U << c);
      ctrl->ch[c].last_result = DAQ_DAC_RESULT_OK;
      ctrl->ch[c].sequence = cmd->sequence;
    }
    return cmd->channel == DAQ_DAC_CHANNEL_ALL || cmd->channel < DAQ_DAC_CHANNELS
        ? DAQ_DAC_RESULT_OK : DAQ_DAC_RESULT_BAD_REQUEST;
  }
  if (cmd->channel >= DAQ_DAC_CHANNELS) return DAQ_DAC_RESULT_BAD_REQUEST;
  daq_dac_channel_state_t *ch = &ctrl->ch[cmd->channel];
  const uint8_t bit = (uint8_t)(1U << cmd->channel);
  ctrl->changed |= (uint8_t)(ch->sequence != cmd->sequence ? bit : 0U);
  ch->sequence = cmd->sequence;

  if (cmd->waveform >= DAQ_DAC_WAVE_COUNT || !isfinite(cmd->amplitude_v) ||
      !isfinite(cmd->offset_v) || !isfinite(cmd->frequency_hz) ||
      !isfinite(cmd->frequency_stop_hz) || (cmd->flags & ~DAQ_DAC_FLAGS_KNOWN) != 0U) {
    /* A malformed request changes nothing that is already running. */
    if (ch->last_result != DAQ_DAC_RESULT_BAD_REQUEST) ctrl->changed |= bit;
    ch->last_result = DAQ_DAC_RESULT_BAD_REQUEST;
    return DAQ_DAC_RESULT_BAD_REQUEST;
  }
  const daq_dac_result_t gate = daq_dac_state_gate(has_state, state);
  if (gate != DAQ_DAC_RESULT_OK) {
    if (ch->last_result != (uint8_t)gate) ctrl->changed |= bit;
    ch->last_result = (uint8_t)gate;
    turn_off(ctrl, cmd->channel, DAQ_DAC_OFF_FLIGHT_STATE, actions);
    return gate;
  }
  if (cmd->waveform == DAQ_DAC_WAVE_ARBITRARY &&
      (!table || table->state != DAQ_DAC_TABLE_READY || table->len != cmd->table_len ||
       table->crc != cmd->table_crc)) {
    /* Never play a table the station did not verify; a running output keeps
     * running unless it was the same channel's arbitrary output. */
    if (ch->last_result != DAQ_DAC_RESULT_TABLE_MISMATCH) ctrl->changed |= bit;
    ch->last_result = DAQ_DAC_RESULT_TABLE_MISMATCH;
    return DAQ_DAC_RESULT_TABLE_MISMATCH;
  }
  daq_dac_config_t cfg;
  (void)daq_dac_clamp(cmd, &cfg);
  if (ch->last_result != DAQ_DAC_RESULT_OK) ctrl->changed |= bit;
  ch->last_result = DAQ_DAC_RESULT_OK;
  ch->lease_deadline_ms = now_ms + cfg.lease_ms;
  if (ch->active && ch->configured && same_config(&ch->config, &cfg)) {
    /* Keepalive: the waveform keeps running with its phase. */
    if (ch->config.lease_ms != cfg.lease_ms || ch->config.clamp_flags != cfg.clamp_flags) ctrl->changed |= bit;
    ch->config.lease_ms = cfg.lease_ms;
    ch->config.clamp_flags = cfg.clamp_flags;
    return DAQ_DAC_RESULT_OK;
  }
  ch->config = cfg;
  ch->configured = 1U;
  ch->active = 1U;
  ch->off_reason = DAQ_DAC_OFF_NONE;
  ch->actual_frequency_hz = 0.0f;
  ch->actual_dwell_ms = 0U;
  ch->update_hz = 0.0f;
  ch->samples = 0U;
  ch->table_crc = 0U;
  ctrl->changed |= bit;
  actions[cmd->channel] = DAQ_DAC_ACTION_START;
  return DAQ_DAC_RESULT_OK;
}

void daq_dac_ctrl_tick(daq_dac_ctrl_t *ctrl, bool has_state, uint8_t state, uint32_t now_ms,
                       uint8_t actions[DAQ_DAC_CHANNELS])
{
  const bool allowed = daq_dac_state_gate(has_state, state) == DAQ_DAC_RESULT_OK;
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    actions[c] = DAQ_DAC_ACTION_NONE;
    if (!ctrl->ch[c].active) continue;
    if (!allowed) turn_off(ctrl, c, DAQ_DAC_OFF_FLIGHT_STATE, actions);
    else if ((int32_t)(now_ms - ctrl->ch[c].lease_deadline_ms) >= 0)
      turn_off(ctrl, c, DAQ_DAC_OFF_LEASE, actions);
  }
}

void daq_dac_ctrl_force_off(daq_dac_ctrl_t *ctrl, uint8_t reason, uint8_t actions[DAQ_DAC_CHANNELS])
{
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    actions[c] = DAQ_DAC_ACTION_NONE;
    turn_off(ctrl, c, reason, actions);
  }
}

void daq_dac_ctrl_started(daq_dac_ctrl_t *ctrl, unsigned c, const daq_dac_plan_t *plan, bool ok)
{
  if (c >= DAQ_DAC_CHANNELS) return;
  daq_dac_channel_state_t *ch = &ctrl->ch[c];
  ctrl->changed |= (uint8_t)(1U << c);
  if (!ok) {
    ch->active = 0U;
    ch->off_reason = DAQ_DAC_OFF_HARDWARE;
    ch->last_result = DAQ_DAC_RESULT_HARDWARE_ERROR;
    return;
  }
  ch->actual_frequency_hz = plan ? plan->actual_frequency_hz : 0.0f;
  ch->actual_dwell_ms = plan ? plan->actual_dwell_ms : 0U;
  ch->update_hz = plan ? plan->update_hz : 0.0f;
  ch->samples = plan ? plan->samples : 0U;
  ch->table_crc = plan ? plan->table_crc : 0U;
}

uint8_t daq_dac_ctrl_active_mask(const daq_dac_ctrl_t *ctrl)
{
  uint8_t mask = 0U;
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c)
    if (ctrl->ch[c].active) mask |= (uint8_t)(1U << c);
  return mask;
}

uint8_t daq_dac_take_changes(daq_dac_ctrl_t *ctrl)
{
  const uint8_t changed = ctrl->changed;
  ctrl->changed = 0U;
  return changed;
}

void daq_dac_status_encode(const daq_dac_ctrl_t *ctrl, const daq_dac_table_t *table,
                           uint8_t flight_state, uint8_t gate,
                           uint8_t flags, uint32_t uptime_ms,
                           const float output_v[DAQ_DAC_CHANNELS],
                           const float readback_v[DAQ_DAC_CHANNELS],
                           uint8_t out[DAQ_DAC_STATUS_BYTES])
{
  memset(out, 0, DAQ_DAC_STATUS_BYTES);
  out[0] = DAQ_DAC_STATUS_VERSION; out[1] = flight_state; out[2] = gate; out[3] = flags;
  wr32(out + 4, uptime_ms);
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    const daq_dac_channel_state_t *ch = &ctrl->ch[c];
    const daq_dac_config_t *cfg = &ch->config;
    uint8_t *p = out + 8U + c * DAQ_DAC_STATUS_CHANNEL_BYTES;
    p[0] = ch->active; p[1] = cfg->waveform; p[2] = ch->last_result;
    p[3] = cfg->clamp_flags; p[4] = ch->off_reason; p[5] = cfg->flags;
    wr16(p + 6, cfg->waveform == DAQ_DAC_WAVE_STAIRCASE ? cfg->steps : 0U);
    wrf(p + 8, cfg->amplitude_v);
    wrf(p + 12, cfg->offset_v);
    wrf(p + 16, ch->actual_frequency_hz);
    wr32(p + 20, ch->actual_dwell_ms);
    wr32(p + 24, ch->sequence);
    wrf(p + 28, ch->active && output_v ? output_v[c] : NAN);
    wrf(p + 32, ch->active && readback_v ? readback_v[c] : NAN);
    wrf(p + 36, ch->update_hz);
    wr16(p + 40, ch->samples);
    wr16(p + 42, cfg->duty_q16);
    wr16(p + 44, cfg->phase_q16);
    wr16(p + 46, cfg->table_len);
    wr32(p + 48, ch->table_crc);
  }
  if (table) {
    uint8_t *t = out + 8U + DAQ_DAC_CHANNELS * DAQ_DAC_STATUS_CHANNEL_BYTES;
    t[0] = table->state;
    wr16(t + 2, table->len);
    wr32(t + 4, table->upload_id);
    wr32(t + 8, table->expected_crc);
    wr32(t + 12, table->crc);
    wr64(t + 16, table->chunks);
  }
}
