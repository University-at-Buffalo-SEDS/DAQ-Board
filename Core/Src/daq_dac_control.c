#include "daq_dac_control.h"

#include <math.h>
#include <string.h>

#define DAQ_DAC_TWO_PI 6.28318530718f

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

int daq_dac_command_decode(const uint8_t *p, size_t len, daq_dac_command_t *out)
{
  if (!p || !out || len != DAQ_DAC_COMMAND_BYTES || p[0] != DAQ_DAC_COMMAND_VERSION) return -1;
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
}

uint8_t daq_dac_clamp(const daq_dac_command_t *cmd, daq_dac_config_t *cfg)
{
  uint8_t flags = 0U;
  memset(cfg, 0, sizeof(*cfg));
  cfg->waveform = cmd->waveform;

  float offset = cmd->offset_v;
  if (offset < DAQ_DAC_MIN_V) { offset = DAQ_DAC_MIN_V; flags |= DAQ_DAC_CLAMP_OFFSET; }
  else if (offset > DAQ_DAC_MAX_V) { offset = DAQ_DAC_MAX_V; flags |= DAQ_DAC_CLAMP_OFFSET; }
  cfg->offset_v = offset;

  if (cmd->waveform != DAQ_DAC_WAVE_DC) {
    const float headroom = fminf(offset - DAQ_DAC_MIN_V, DAQ_DAC_MAX_V - offset);
    float amplitude = cmd->amplitude_v;
    if (amplitude < 0.0f) { amplitude = 0.0f; flags |= DAQ_DAC_CLAMP_AMPLITUDE; }
    if (amplitude > headroom) { amplitude = headroom; flags |= DAQ_DAC_CLAMP_AMPLITUDE; }
    cfg->amplitude_v = amplitude;
  }

  if (cmd->waveform == DAQ_DAC_WAVE_SQUARE || cmd->waveform == DAQ_DAC_WAVE_SINE ||
      cmd->waveform == DAQ_DAC_WAVE_TRIANGLE) {
    float frequency = cmd->frequency_hz;
    if (frequency < DAQ_DAC_FREQ_MIN_HZ) { frequency = DAQ_DAC_FREQ_MIN_HZ; flags |= DAQ_DAC_CLAMP_FREQUENCY; }
    else if (frequency > DAQ_DAC_FREQ_MAX_HZ) { frequency = DAQ_DAC_FREQ_MAX_HZ; flags |= DAQ_DAC_CLAMP_FREQUENCY; }
    cfg->frequency_hz = frequency;
  }

  if (cmd->waveform == DAQ_DAC_WAVE_STAIRCASE) {
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
  if (!(volts > 0.0f)) return 0U; /* Also NaN. */
  const float code = volts * (float)DAQ_DAC_CODE_MAX / DAQ_DAC_VREF_V + 0.5f;
  return code >= (float)DAQ_DAC_CODE_MAX ? (uint16_t)DAQ_DAC_CODE_MAX : (uint16_t)code;
}

float daq_dac_code_to_volts(uint32_t code)
{
  if (code > DAQ_DAC_CODE_MAX) code = DAQ_DAC_CODE_MAX;
  return (float)code * DAQ_DAC_VREF_V / (float)DAQ_DAC_CODE_MAX;
}

static float wave_value(const daq_dac_config_t *cfg, unsigned i, unsigned n)
{
  const float a = cfg->amplitude_v, o = cfg->offset_v;
  switch (cfg->waveform) {
    case DAQ_DAC_WAVE_SQUARE: return i < n / 2U ? o + a : o - a;
    case DAQ_DAC_WAVE_SINE: return o + a * sinf(DAQ_DAC_TWO_PI * (float)i / (float)n);
    case DAQ_DAC_WAVE_TRIANGLE: {
      const float p = (float)i / (float)n;
      return o + a * (p < 0.5f ? -1.0f + 4.0f * p : 3.0f - 4.0f * p);
    }
    case DAQ_DAC_WAVE_STAIRCASE:
      return o - a + 2.0f * a * (float)i / (float)(n - 1U);
    default: return o;
  }
}

int daq_dac_plan(const daq_dac_config_t *cfg, uint32_t timer_clock_hz, daq_dac_plan_t *plan,
                 uint32_t table[DAQ_DAC_TABLE_LEN])
{
  if (!cfg || !plan || !table || cfg->waveform >= DAQ_DAC_WAVE_COUNT || timer_clock_hz == 0U) return -1;
  memset(plan, 0, sizeof(*plan));
  unsigned n;
  float rate;
  switch (cfg->waveform) {
    case DAQ_DAC_WAVE_DC: n = 1U; rate = 0.0f; break;
    case DAQ_DAC_WAVE_STAIRCASE:
      n = cfg->steps;
      if (n < DAQ_DAC_STEPS_MIN || n > DAQ_DAC_STEPS_MAX || cfg->dwell_ms == 0U) return -1;
      rate = 1000.0f / (float)cfg->dwell_ms;
      break;
    default:
      n = DAQ_DAC_TABLE_LEN;
      if (!(cfg->frequency_hz > 0.0f)) return -1;
      rate = cfg->frequency_hz * (float)n;
      break;
  }
  for (unsigned i = 0U; i < n; ++i) table[i] = daq_dac_volts_to_code(wave_value(cfg, i, n));
  plan->samples = (uint16_t)n;
  if (n == 1U) return 0;

  /* Counts per sample, split into a 16-bit prescaler and auto-reload. */
  const float total_f = (float)timer_clock_hz / rate;
  if (!(total_f >= 2.0f) || total_f > 4294967296.0f) return -1;
  uint64_t total = (uint64_t)(total_f + 0.5f);
  const uint64_t psc = (total - 1U) / 65536U;
  if (psc > 65535U) return -1;
  uint64_t arr = (total + (psc + 1U) / 2U) / (psc + 1U);
  if (arr < 2U) arr = 2U;
  if (arr > 65536U) arr = 65536U;
  plan->psc = (uint16_t)psc;
  plan->arr = (uint16_t)(arr - 1U);
  plan->update_hz = (float)timer_clock_hz / ((float)(psc + 1U) * (float)arr);
  if (cfg->waveform == DAQ_DAC_WAVE_STAIRCASE)
    plan->actual_dwell_ms = (uint32_t)(1000.0f / plan->update_hz + 0.5f);
  else plan->actual_frequency_hz = plan->update_hz / (float)n;
  return 0;
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
         a->offset_v == b->offset_v && a->frequency_hz == b->frequency_hz && a->dwell_ms == b->dwell_ms;
}

daq_dac_result_t daq_dac_ctrl_command(daq_dac_ctrl_t *ctrl, const daq_dac_command_t *cmd,
                                      bool has_state, uint8_t state, uint32_t now_ms,
                                      uint8_t actions[DAQ_DAC_CHANNELS])
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
      !isfinite(cmd->offset_v) || !isfinite(cmd->frequency_hz)) {
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

void daq_dac_status_encode(const daq_dac_ctrl_t *ctrl, uint8_t flight_state, uint8_t gate,
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
    uint8_t *p = out + 8U + c * DAQ_DAC_STATUS_CHANNEL_BYTES;
    p[0] = ch->active; p[1] = ch->config.waveform; p[2] = ch->last_result;
    p[3] = ch->config.clamp_flags; p[4] = ch->off_reason;
    wr16(p + 6, ch->config.waveform == DAQ_DAC_WAVE_STAIRCASE ? ch->config.steps : 0U);
    wrf(p + 8, ch->config.amplitude_v);
    wrf(p + 12, ch->config.offset_v);
    wrf(p + 16, ch->actual_frequency_hz);
    wr32(p + 20, ch->actual_dwell_ms);
    wr32(p + 24, ch->sequence);
    wrf(p + 28, ch->active && output_v ? output_v[c] : NAN);
    wrf(p + 32, ch->active && readback_v ? readback_v[c] : NAN);
  }
}
