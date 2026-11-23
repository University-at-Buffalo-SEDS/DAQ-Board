# DAC waveform specification

This is the normative definition of every signal the DAQ's DAC generator can
play: its parameters, how its samples are computed, how the trigger timer is
chosen, and the wire formats that carry it. Three implementations follow it
and must agree bit for bit:

- the firmware: `Core/Src/daq_dac_wave.c` (the reference) and
  `Core/Src/daq_dac_control.c` (clamping and planning);
- GroundStation26: `backend/src/daq_dac/waveform.rs` (preview, achieved
  frequency and the CRC it expects in `DAQ_DAC_STATUS`);
- the firmware simulator's virtual DAC->ADC patch. It can compile
  `daq_dac_wave.c` and `daq_dac_control.c` directly (no HAL, RTOS or SEDSnet
  dependency) or reimplement this text.

Hardware, safety gating and the ADC validation workflow are in
[dac-validation.md](dac-validation.md).

## Output model

Volts here are **DAC-pin volts**. P10 carries 2 x pin volts once the U19
buffer is reworked (see dac-validation.md). GroundStation converts.

- Code: 12 bits, `V = code * 3.3 / 4095`. One code is 0.806 mV.
- Linear range: 0.20 to 3.10 V, codes `CODE_LO = 248` to `CODE_HI = 3847`.
  Built-in waves are clamped into it (offset first, then amplitude to the
  nearer rail). An uploaded table may contain only codes in this range.
- Volts to code: `code = floor(v * 4095.0 / 3.3 + 0.5)`, evaluated left to
  right in IEEE-754 double. Saturate to 0..4095. NaN and `v <= 0` give 0.
  `v` is the single-precision value from the command or status, widened to
  double exactly.

### Determinism

Every table value uses only double `+ - * /` and `floor`, evaluated in the
order written here, with no fused multiply-add. Sine uses the polynomial
below, not libm. IEEE-754 rounds these operations exactly, so an MCU with soft
double, x86-64, Rust and a native simulator all produce identical tables. Only
the chirp's frequency (not its table) uses `powf`.

`sin_turns(x)` = sin(2 pi x):

```
x  = x - floor(x)
y  = 4x;  q = floor(y);  r = y - q;  quadrant = q mod 4
if quadrant is odd: r = 1 - r
t  = r * 1.5707963267948966;  t2 = t * t
s  = C15
s  = C13 + t2*s;  s = C11 + t2*s;  s = C9 + t2*s;  s = C7 + t2*s
s  = C5 + t2*s;   s = C3 + t2*s;   s = 1 + t2*s;   s = t*s
return quadrant >= 2 ? -s : s
C3 = -1/6, C5 = 1/120, C7 = -1/5040, C9 = 1/362880, C11 = -1/39916800,
C13 = 1/6227020800, C15 = -1/1307674368000   (each a double quotient)
```

It is within 1e-11 of the true sine; one DAC code is 2.4e-4 of full scale.

## Waveforms

`a` = amplitude (peak), `o` = offset, both pin volts as applied. `n` = table
length. Each sample `i` in `0..n-1` has value `v_i`, and the table entry is
`code(v_i)`. Periodic waves use the phase
`x_i = frac(i / n + phase_q16 / 65536)`, where `frac(z) = z - floor(z)`.

| # | Waveform | `v_i` | Parameters |
|---|---|---|---|
| 0 | DC | `o` (n = 1, no timer) | offset |
| 1 | Square | `o + a` if `x_i < duty_q16 / 65536`, else `o - a` | f, a, o, phase, duty |
| 2 | Sine | `o + a * sin_turns(x_i)` | f, a, o, phase |
| 3 | Triangle | `o + a * (x_i < 0.5 ? -1 + 4x_i : 3 - 4x_i)` | f, a, o, phase |
| 4 | Staircase | `o - a + 2a * i / (n - 1)`, n = steps | a, o, steps, dwell |
| 5 | Sawtooth / ramp | up: `o + a * (-1 + 2x_i)`; with RAMP_DOWN: `o + a * (1 - 2x_i)` | f, a, o, phase, flag |
| 6 | Chirp | as sine, n = chirp length; frequency retuned (below) | f_start, f_stop, sweep, a, o, phase, LOG flag |
| 7 | Noise | `o + a * ((s_i >> 8) / 8388608 - 1)`, n = 256 | sample rate, a, o |
| 8 | Arbitrary | `o + a * (s_j / 32767)`, `j = (i + rot) mod n` | f (repetitions/s), a, o, phase, shape length + CRC |

- **Noise**: `s` is xorshift32 seeded `0x2545F491`. For each sample:
  `s ^= s << 13; s ^= s >> 17; s ^= s << 5` (32-bit), then use `s_i = s`. It
  is uniform in about `o +/- a`, and the same 256 values repeat, so a capture
  can be compared sample for sample. Its `frequency_hz` is the **sample
  rate**.
- **Arbitrary**: the station uploads a **normalized shape** `s` of `n` signed
  Q15 samples (`-32767..32767` = -1..1). For example, it may be an equation
  `f(x)` sampled over one period with `x = i/n` in [0, 1), or a WAV clip.
  - Each channel scales the shape into its own play table exactly like a
    built-in wave: `rot = ((phase_q16 * n + 32768) >> 16) mod n`, all integer.
  - `frequency_hz` is the number of periods (whole shapes) per second, so the
    update rate is `frequency_hz * n`.
  - Changing amplitude, offset, phase or frequency re-scales or re-times the
    stored shape with no new upload. The station uploads again only when the
    shape itself changes, for example an equation of `t` at a new frequency,
    or a frequency that needs a different length.
  - Amplitude is clamped to the offset's headroom like every wave, so a shape
    within -1..1 can never leave the DAC range.
  - The `-32768` code is refused.
- **Duty** is clamped to 656..64880 (1 %..99 %). The achieved duty is
  `count(x_i < duty) / n`, so it is quantised to 1/n.
- **Phase** sets the start phase of the table, relative to its own start.
  The two channels start independently, so phase between channels is not
  defined.

## Rates and table lengths

Timers TIM7 (OUT1) and TIM15 (OUT2) run at `CLK = 160 MHz`. The update rate
is `CLK / ((psc + 1) * (arr + 1))` with 16-bit PSC and ARR, limited to
0.05 S/s..**200 kS/s per channel**.

`split(rate, tries)`: `total = CLK / rate` (must be >= 2);
`first = floor((total - 1) / 65536)`. For `p = first, first + 1, ...` (while
`p < first + tries` and `p <= 65535`): `a = clamp(floor(total / (p + 1) + 0.5), 2, 65536)`
and `achieved = CLK / ((p + 1) * a)`. Keep the smallest `|achieved - rate|`;
on a tie keep the earlier `p`. Then `psc = p` and `arr = a - 1`. A lone split
uses `tries = 64`.

`choose(f, n_min = 16, n_max = 256)`, for periodic waves: for even `n` from
`n_max` down to `n_min`, skip lengths whose `f * n` is outside the rate limits.
Split with `tries = 3` and compute `err = |achieved / n - f|`. Keep the
smallest error (strict `<`, so ties keep the longer table). Stop at the first
length with `err <= f * 1e-5`: 10 ppm is inside the crystal's own tolerance,
and a longer table has less quantisation distortion.

Measured over a log sweep from 0.01 Hz to 10 kHz:
- the error is within 10 ppm up to about 600 Hz;
- the worst error is 2.6e-4, at about 9.4 kHz, where the 200 kS/s limit allows
  only 20-24 samples per period.
- Examples: 1-100 Hz use 256 samples; 1 kHz uses 200 samples at 200 kS/s;
  10 kHz uses 20 samples.

| Waveform | Table length | Update rate | Frequency / timing range |
|---|---|---|---|
| Sine, square, triangle, sawtooth | `choose` (16-256) | `f * n` | 0.01 Hz-10 kHz |
| Chirp | `chirp_len`: `floor(200000 / max(f0, f1))`, at most 256, even, at least 16 | `f(t) * n` | 0.01 Hz-10 kHz each end; sweep 0.1-600 s |
| Noise | 256 | `f` (lone split) | 1 S/s-200 kS/s |
| Staircase | steps (2-256) | `1000 / dwell_ms` | dwell 10-20000 ms |
| Arbitrary | 16-512, even | `f * len` | 0.01 Hz to `min(10 kHz, 200000 / len)` |
| DC | 1 | none | none |

- **Chirp**:
  - `f(t) = f0 + (f1 - f0) * u` (linear) or `f0 * (f1 / f0)^u` (LOG flag), with
    `u = (t mod sweep_ms) / sweep_ms`. The sweep repeats; downward sweeps are
    allowed.
  - The DAQ re-splits `f(t) * n` at each 50 ms service tick. ARR and PSC are
    preloaded and land on the next update event, so the table index never
    jumps and the sine stays phase-continuous.
  - The frequency is stepped (20 steps/s), not swept continuously. Status
    reports the instantaneous frequency.
  - A PSC/ARR pair written across an update event can stretch one sample.
- **Status** reports the achieved values: `actual_frequency_hz`, `update_hz`,
  `samples` and the actual staircase dwell.

## Table CRC

`CRC-32/ISO-HDLC` (reflected polynomial 0xEDB88320, init 0xFFFFFFFF, final
xor 0xFFFFFFFF) is computed over the table codes, each as two little-endian
bytes. An uploaded shape is identified by the same CRC over its int16 samples,
also little-endian. `DAQ_DAC_STATUS` reports it for the table each channel's DMA is
playing, so GroundStation can regenerate the table from the applied
parameters in the same status and confirm the DAQ plays exactly what it
previewed.

Golden vectors (offset 1.6 V, amplitude 1.0 V unless noted, 160 MHz):

| Case | Parameters | n | psc | arr | CRC |
|---|---|---|---|---|---|
| sine10 | sine 10 Hz | 256 | 0 | 62499 | 81CDD413 |
| sine123p90 | sine 123.4 Hz, phase 16384 | 236 | 0 | 5493 | AF1D0167 |
| square50d25 | square 50 Hz, duty 16384 | 256 | 0 | 12499 | 4C829B77 |
| tri1 | triangle 1 Hz | 256 | 9 | 62499 | 8BF0BFDD |
| saw7777 | sawtooth up 7777 Hz | 18 | 0 | 1142 | F74D8A08 |
| sawdown2p5 | sawtooth down 2.5 Hz | 256 | 3 | 62499 | A444C715 |
| stair11 | staircase 11 steps x 500 ms | 11 | 1249 | 63999 | 43675E77 |
| noise1k | noise 1000 S/s, amplitude 0.5 V | 256 | 3 | 39999 | 806C5080 |
| chirp10to1k | chirp 10 to 1000 Hz linear, 10 s | 200 | 1 | 39999 | 70F8CF62 |
| dc1p25 | DC 1.25 V | 1 | none | none | 2F22AB05 |
| shape | arbitrary: 64-sample triangle shape `floor(32767 * (x<0.5 ? -1+4x : 3-4x) + 0.5)` (shape CRC 79119F9E), 100 Hz, phase 16384 | 64 | 0 | 24999 | A7B93781 |

The DAQ host tests (`tests/test_dac_waveforms.py`) and the GroundStation unit
tests assert these values.

## Wire formats (little endian)

### DAQ_DAC_COMMAND v2 (48 bytes, GS to SD_CARD, unreliable, lease keepalive)

| Bytes | Field |
|---|---|
| 0 | version = 2 |
| 1 | channel (0, 1; 0xFF = all, disable only) |
| 2 | enable (0/1) |
| 3 | waveform (table above) |
| 4-7 | f32 amplitude_v (pin) |
| 8-11 | f32 offset_v (pin) |
| 12-15 | f32 frequency_hz (chirp start; noise sample rate; arbitrary repetitions/s) |
| 16-17 | u16 steps |
| 18-19 | u16 lease_ms (0 = 3000) |
| 20-23 | u32 dwell_ms |
| 24-27 | u32 sequence |
| 28-29 | u16 phase_q16 |
| 30-31 | u16 duty_q16 |
| 32-35 | f32 frequency_stop_hz (chirp) |
| 36-39 | u32 sweep_ms (chirp) |
| 40-43 | u32 table_crc (arbitrary: CRC of the uploaded shape) |
| 44-45 | u16 table_len (arbitrary) |
| 46 | flags: bit 0 RAMP_DOWN, bit 1 LOG_SWEEP; other bits are refused |
| 47 | reserved (0) |

An arbitrary command is refused with `TABLE_MISMATCH` (result 6) unless the
uploaded table is READY with exactly that length and CRC.

### DAQ_DAC_STATUS v2 (136 bytes, to the GS)

Header (8 bytes):

| Bytes | Field |
|---|---|
| 0 | version = 2 |
| 1 | flight state |
| 2 | gate |
| 3 | flags (bit 0 hardware ready, bit 1 readback OK) |
| 4-7 | u32 uptime_ms |

Then two channel blocks of 52 bytes each (channel 0 at 8, channel 1 at 60):

| Offset | Field |
|---|---|
| 0 | active |
| 1 | waveform |
| 2 | last_result |
| 3 | clamp_flags (offset, amplitude, frequency, steps, dwell, lease, duty, sweep) |
| 4 | off_reason (0 none, 1 commanded, 2 flight state, 3 abort, 4 lease, 5 hardware) |
| 5 | wave flags |
| 6 | u16 steps |
| 8 | f32 amplitude_v |
| 12 | f32 offset_v |
| 16 | f32 actual_frequency_hz |
| 20 | u32 actual_dwell_ms |
| 24 | u32 sequence |
| 28 | f32 output_v |
| 32 | f32 readback_v |
| 36 | f32 update_hz |
| 40 | u16 samples (table length playing) |
| 42 | u16 duty_q16 (applied) |
| 44 | u16 phase_q16 |
| 46 | u16 table_len (arbitrary) |
| 48 | u32 table_crc (CRC of the codes the DMA plays) |

Then the table block (24 bytes, at 112):

| Offset | Field |
|---|---|
| 0 | state: 0 empty, 1 loading, 2 ready, 3 CRC error, 4 bad sample, 5 bad length, 6 verifying |
| 2 | u16 len |
| 4 | u32 upload_id |
| 8 | u32 expected CRC (from BEGIN) |
| 12 | u32 computed CRC (set at commit) |
| 16 | u64 chunk bitmap (bit k = samples 32k..32k+31 received) |

### DAQ_DAC_TABLE v1 (76 bytes, GS to SD_CARD, unreliable)

| Bytes | Field |
|---|---|
| 0 | version = 1 |
| 1 | op: 1 BEGIN, 2 DATA, 3 COMMIT, 4 CLEAR |
| 2-3 | u16 index (BEGIN/COMMIT: table length; DATA: first sample, a multiple of 32) |
| 4-7 | u32 upload_id |
| 8-11 | u32 value (BEGIN/COMMIT: CRC-32 of the whole table; DATA: sample count, 32 except the last chunk) |
| 12-75 | 32 x i16 shape samples (DATA) |

Upload protocol, as the station runs it:

1. Skip the upload when status already shows `ready` with the same length
   and shape CRC.
2. Send BEGIN, and repeat it until status shows `loading` with this
   `upload_id`. Running outputs keep playing their own scaled copies.
3. Send the DATA chunks, paced.
4. Read the bitmap from status and resend the missing chunks until the bitmap
   is full.
5. Send COMMIT. The DAQ refuses chunk writes, hashes the table and reports
   `ready` with the computed CRC, or `crc_error`. An incomplete COMMIT leaves
   the table `loading`.
6. Start the channel with an ARBITRARY command carrying the length, shape CRC,
   amplitude, offset, phase and frequency.

DATA is only accepted while the table is `loading`. A sample of -32768 makes
the upload `bad_sample`. RAM use:
- the shape: 512 entries, 1 KiB, shared;
- each channel's play table: 512 codes, 2 x 1 KiB. Built-in waves use up to
  256 of them.

Codes are stored as 16 bits, and the GPDMA pads each one to the 32-bit
DHR12Rx write.

## Not yet verified on silicon

- GPDMA half-word-to-word padding into DHR12Rx (`SDW = half-word`,
  `DDW = word`, `PAM = 0`). Check with a scope that every table sample
  appears, not every other one.
- The chirp's preloaded ARR/PSC retune while running.
- Settling at 200 kS/s through the reworked x2 buffer.
