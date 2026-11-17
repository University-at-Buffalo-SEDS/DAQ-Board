# DAC signal generator for ADC validation

GroundStation can drive the STM32U585's DAC1 to generate a known signal. A cable
from the DAC connector to an ADC input then checks that input's whole
measurement path. Output is off at boot and runs only while the station
allows it. GroundStation chooses the channel, waveform, amplitude, offset,
frequency and the ADC input under test.

## Hardware

Schematic rev A (`Schematic PDF_[No Variations] (2).pdf`), sheets 3 and 6.

| MCU | Schematic net | Buffer | P10 pin |
|---|---|---|---|
| DAC1 OUT1, PA4 | `DAC1` | U19C (OPA2994, +12VA) | pin 2, net `DAC2_OUT` |
| DAC1 OUT2, PA5 | `DAC2` | U19B | pin 5, net `DAC1_OUT` |

P10 is a JST-SH 6-pin connector: pins 1, 3, 4 and 6 are GND. The output net
names are crossed relative to the MCU channels: `DAC2_OUT` carries PA4.

- The LQFP64 STM32U585RIT6 has no VREF+ pin, so VREF+ is VDDA (+3V3A). Firmware
  runs the output buffer on, so its linear range is 0.20 to 3.10 V at the pin
  (`DAQ_DAC_MIN_V`/`DAQ_DAC_MAX_V`). Each step is 0.81 mV.
- Each pin has a 1 kOhm series resistor into the op-amp, a 1 MOhm pulldown,
  and a 10k/10k divider from the op-amp output.
- **As drawn, the buffer does not work.** The DAC drives the inverting input
  (U19B pin 2, U19C pin 6), and the divider tap feeds the non-inverting input
  (pins 3 and 5). That is positive feedback, so the output latches to a rail.
  The divider then holds the + input at 0 or about 6 V, which the DAC can never
  cross. **Minimal rework:** swap each half's input nets, so the DAC/R93 node
  goes to pin 3 (pin 5) and DAC1_FB (DAC2_FB) goes to pin 2 (pin 6). For
  example, lift the two pins and cross-wire them, or use a board spin. Each half
  then becomes a non-inverting x2 buffer: P10 = 2 x DAC, 0.4 to 6.2 V.
- Before the rework, use the internal readback only. Do not loop P10 to an
  input.

The CubeMX `.ioc` still declares DAC1 as internal-only and leaves PA4/PA5
unassigned, so they keep their reset analog mode. `daq_dac.c` reconfigures each
channel at run time: external pin plus on-chip connection, buffer on, and a
timer trigger. Regenerating CubeMX code therefore does not change this
behaviour.

## Reachable ADC inputs (V_in = 2 x V_dac x external attenuation)

| Input | Path | Reported value | Usable V_in |
|---|---|---|---|
| ADC4 internal `DAC1CH1/2_ADC4` | none (no cable) | `readback_v` in `DAQ_DAC_STATUS` | 0.20-3.10 V at the pin |
| SAR_IN1-8 (P9, P3, P2, P1 pins 3/4) | unity buffer to STM32 ADC4/ADC1 | V_in | 0-3.3 V |
| MCP CH2-CH5 (P6, P5 pins 3/4) | 47R + 47R, 10 nF, diode clamps | V_in | 0-2.4 V |
| HVAIN1/2 (P4 pins 3/4) | 10k:1k divider, unity buffer, MCP CH6/7; firmware x11 | V_in | 0-26.4 V (DAC reaches 6.2 V) |
| P6 differential (pin 3 - pin 4) | PGA x16, KG50 input = P6 | V_in | 0-0.15 V |
| P7 load cell (AMP1, CH0) | 0.218 V + 100.9 x (S- minus S+) | ADC-pin volts | -2.2 to +21.6 mV: needs about 1:100 |
| P8 load cell (AMP2, CH1) | 0.600 V + 100.9 x (S- minus S+) | ADC-pin volts | -5.9 to +17.8 mV: needs about 1:100 |

To loop into a load cell, drive S- (pin 4) through an external divider and
ground S+ (pin 3); driving S+ instead gives a gain of -100.9. The
instrumentation amplifier's common-mode range near 0 V must still be checked
against the TLV2888 datasheet. Without an attenuator, one DAC step is about
163 mV at the load-cell ADC.

GroundStation receives about 40 packets/s for load cells and about 4/s for the
auxiliary streams, so periodic waves must be slow; GroundStation warns about
this. The SD card keeps the full-rate raw ADC data for offline comparison.

## Firmware

- `daq_dac_control.c` is HAL-free. It decodes the command, clamps the request,
  builds the waveform table, plans the timer and runs the safety state machine.
  `daq_dac.c` drives the hardware.
- Waveforms: DC, square (50 %), sine, triangle and staircase. Periodic waves use
  32 samples per period at 0.01-1000 Hz. A staircase has 2-32 steps from
  offset-amplitude to offset+amplitude, each held for 10-20000 ms. TIM7 (OUT1)
  and TIM15 (OUT2) TRGO trigger the DAC. GPDMA1 channels 10 and 11 reload a
  self-linked list item, so the table repeats with no interrupts or CPU time,
  and load-cell acquisition is untouched. DC needs no timer or DMA. RAM cost is
  a 256-byte table plus about 250 bytes of state.
- Each request is clamped to 0.20-3.10 V, and the applied values and clamp
  flags appear in status. The timer quantises frequency and dwell, and status
  reports the actual values.
- Safety:
  - Output is off at boot, and nothing is persisted.
  - Enable is accepted only in Startup or Idle with a known FLIGHT_STATE.
  - Received FLIGHT_STATE values outside Startup/Idle stop the hardware
    immediately, through the `flight_state_cache_apply_network_update` hook, and
    a 50 ms poll backs this up.
  - ABORT (delivered to SD_CARD) stops both outputs immediately and discards
    any queued request.
  - The station must refresh each channel's lease, 3 s by default. Without that,
    for example on GS link loss, the channel turns off.
  - "Off" disables the DAC channel, its DMA and its timer. The pin goes
    high-impedance and the 1 MOhm pulldown takes it to 0 V.
- `DAQ_DAC_STATUS` is sent at 1 Hz while off, 5 Hz while active, and promptly on
  any change. It carries the gate, active state, applied configuration, last
  result, why the output last went off, the DAC output register, and an ADC4
  conversion of the DAC output. The ADC4 conversion runs in the existing 100 ms
  internal-ADC pass, and only while a channel is active.
- SD: each change writes `dac_chN_active`, `_waveform`, `_amplitude_v`,
  `_offset_v`, `_frequency_hz` (actual), `_steps`, `_dwell_ms`, `_sequence`,
  `_result` and `_off_reason` rows. While active, `dac_chN_output_v` and
  `dac_chN_readback_v` are written at 10 Hz. The existing `analog_out_N_v`
  snapshot rows now report the DAC output, or NaN when off.

The wire layouts are in `config/sedsnet.json` (`DAQ_DAC_COMMAND`, 32 bytes, GS to
SD_CARD, unreliable; `DAQ_DAC_STATUS`, 80 bytes, to GROUND_STATION). They are
also in `Core/Inc/daq_dac_control.h`.

## Not yet verified

Host tests cover the waveform maths, clamping, gating and every auto-off
trigger. They also run the production glue against register mocks. They cannot
verify:
- GPDMA linked-list circular operation on silicon;
- the DAC high-impedance level when disabled;
- the buffer rework;
- real loopback accuracy.

Bench-check with a scope on P10 after the rework, before trusting a validation
report.
