# Load-cell inputs

The repository's `Schematic PDF_[No Variations] (2).pdf`, sheets 1, 9, and 10,
maps the load-cell connectors as follows:

| Connector | Amplifier | MCP3564R input | Telemetry |
| --- | --- | --- | --- |
| P7 | AMP1, U9/U10 | CH0 | KG1000 |
| P8 | AMP2, U11/U12 | CH1 | KG50 |

Both amplifiers accept differential bridge signals and produce voltages
referenced to ground. The ADC converts CH0 and CH1 as separate single-ended
inputs (CH0 in the fast scan, CH1 in the slot). Selecting CH0 minus CH1 would
subtract the outputs of two different load cells.

## Comparison with the working input

The existing software calls the working CH0 input `KG1000`. That telemetry
name does not configure the ADC for a particular load-cell capacity; capacity
and sensitivity are accounted for by the channel's independent calibration.

Sheet 10 uses the same signal-conditioning circuit on both inputs, with a
different bias divider:

| Component/function | P7 / AMP1 / CH0 | P8 / AMP2 / CH1 |
| --- | --- | --- |
| Input amplifier | U9, TLV2888 | U11, TLV2888 |
| Difference amplifier / bias buffer | U10, OPA2197 | U12, OPA2197 |
| First-stage feedback | R40A/B, 10 kΩ each | R48A/B, 10 kΩ each |
| First-stage gain resistor | R42, 2.2 kΩ | R50, 2.2 kΩ |
| Second-stage input resistors | RN1A/B, 1 kΩ each | RN2A/B, 1 kΩ each |
| Second-stage feedback/reference resistors | R39A/B, 10 kΩ each | R47A/B, 10 kΩ each |
| Input pull-downs | R43/R44, 1 MΩ | R51/R52, 1 MΩ |
| Output filter | R41, 100 Ω; C46, 10 nF | R49, 100 Ω; C52, 10 nF |
| Upper output clamp | D24 to +3V3A | D25 to +3V3A |
| Amplifier supply | +12VA | +12VA |
| Bridge excitation | Shared U13, nominal 9.6 V | Shared U13, nominal 9.6 V |
| Bias divider | R37/R38, 10 kΩ / 1 kΩ | R45/R46, 30 kΩ / 10 kΩ |
| Nominal bias | 0.218 V | 0.600 V |

The resistor-array values follow the `1001` = 1 kΩ and `1002` = 10 kΩ
part-number coding in the [Vishay ACAS datasheet](https://www.vishay.com/docs/28770/acasat.pdf),
page 3. From these schematic values, the nominal differential gain magnitude
is `(1 + 2*10000/2200) * (10000/1000)`, about 100.9, for both paths.
Following the drawn signal polarity, the ideal linear transfer is
`Vout = Vbias + 100.9*(V(IN_N) - V(IN_P))`. This describes the circuit before
output clipping and component errors. A rising bridge differential can
therefore lower the ADC reading; the GS calibration fit accounts for its sign.

The different biases do not require different ADC drivers or gains. They do
require separate zero offsets and leave different amounts of room before
the amplifier reaches its lower output limit. Copying the working cell's
calibration coefficients onto the 50 kg cell would not be appropriate.

## Selecting the 50 kg input

GroundStation owns the selection (Settings, Ground Station, Loadcells) and sends
it as the read-only `DAQ_KG50_INPUT` variable. The DAQ keeps a flash copy, so a
reboot without GroundStation resumes the same input instead of falling back to
AMP2. GroundStation's value overrides that copy as soon as it arrives: its file
holds the operator's selection and the calibration bound to that input.

GroundStation learns the input in use from the report stream, `KG50` for AMP2
and tagged `DAQ_KG50_SELECTED` for P6, and re-sends its selection while they differ.

## Connector wiring

P7 and P8 share this pinout. Use the connector's physical pin numbers and the
load-cell manufacturer's wire assignments; wire colors are not standardized.

| Pin | Schematic net | Bridge connection |
| --- | --- | --- |
| 1 | +9V6A_LC | E+, nominal 9.6 V excitation |
| 2 | GND | E− |
| 3 | AMP1/2_IN_P | S+ |
| 4 | AMP1/2_IN_N | S− |
| 5 | GND | Additional ground |
| 6 | +VREF | Nominal 2.4 V reference output |

Pin 6 is a reference output, not a bridge sense input. A four-wire bridge uses
pins 1–4, provided it supports the board's nominal 9.6 V excitation. The
schematic alone does not identify the attached cell's wire colors, bridge
type, or excitation rating.

## ADC configuration and voltage units

`CONFIG0=0x82` enables the internal nominal 2.4 V reference. `CONFIG2=0xCF`
selects PGA gain 1 (`GAIN[2:0]=001`) for both load-cell channels (P6 uses
0xEF, gain 16, in its own scan). `CONFIG3=0xB0` includes the channel ID and
signed 25-bit result in each 32-bit read. Nominal ADC input voltage is:

```text
volts = signed_code * VREF / 8388608      (VREF nominal 2.4 V; per-board setting below)
```

Register values per scan (written as one 10-byte burst from CONFIG0 while the
ADC is in standby; [data pipeline](data-pipeline.md) has the schedule):

| Register | Value | Meaning |
| --- | --- | --- |
| CONFIG0 | 0x82, then START makes it 0x83 | Internal reference, external MCLK, standby until START |
| CONFIG1 | 0x08 FILL fast pair, 0x0C FILL slot / LAUNCH fast pair, 0x04 LAUNCH slot, 0x84 temperature | MCLK/1 (MCLK/4 for TEMP), OSR 128 / 256 / 64 |
| CONFIG2 | 0xCF (0xEF for P6) | Gain 1 (16), boost 2, MUX auto-zero enabled |
| CONFIG3 | 0xB0 | One-shot SCAN cycle (back to standby), tagged 32-bit data, no digital calibration |
| SCAN | 0x000041 fast pair; 0x000002 CH1 (AMP2 KG50); 0x000200 P6; one CH2..7 bit; 0x001000 TEMP | One scan cycle per START |
| TIMER | 0x000000 | Scan timing comes from TIM2, not the ADC timer |

SCAN overrides the MUX register. The scanner converts the scan's channels in
descending order (CH6 then CH0 in the fast pair) and each result is routed by
its returned channel ID.

The previous voltage calculation included a divide by 16 despite configuring
gain 1, and used 2.2104 V rather than the nominal internal reference voltage.
Firmware now uses the corrected formula everywhere: KG1000, KG50, GS
calibration and SD raw values are ADC-pin volts, `code * 2.4 / 2^23`.

Earlier firmware reported KG1000/KG50 on a historical raw scale,
`code * 2.2104 / 2^27`, which is neither volts nor kg. One raw unit is exactly
`k = (2.4 / 2^23) / (2.2104 / 2^27) = 16000 / 921` (about 17.3724) volts
(`MCP3564R_LEGACY_RAW_TO_VOLTS`). The DAQ converts a stored raw-scale
calibration once at boot, so the same ADC code gives the same on-board kg:
KG1000 slope / k (intercept unchanged); KG50 `x0 * k`, `c_i / k^i` for
i = 1..4 (`c0` and tare unchanged); thermal raw/C terms * k (reference
temperatures unchanged); IADC and filter settings unchanged. The converted
record is stored again with a trailing volts mark (72 bytes, same key); a
record without it (16, 44, 60 or 68 bytes) is treated as raw scale.
`g_daq_calibration_migrations` counts completed conversions. SD files declare
the volts scale with `loadcell_input=adc_volts` at the end of each
`# calibration` line; files without it use the raw scale.
SD diagnostic rows are `kg1000_window_v` for CH0 and
`kg50_window_v` for CH1.

The register definitions and code scaling are described in the
[Microchip MCP3561/2/4R datasheet](https://www.microchip.com/content/dam/mchp/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf),
sections 5.6, 5.15, 6.2–6.4, and 8.2–8.8.

## Over range

The MCP3564R runs with `DATA_FORMAT[1:0] = 11` (25-bit code, sign in bit 24,
channel ID in bits 31-28). Datasheet DS20006391C section 5.6 and Table 5-8:

- codes -2^23 .. 2^23-1 cover the specified range, +/-VREF/GAIN;
- the 25-bit formats carry over range on to the clip at 0x0FFFFFF / 0x1000000
  (+/-2 VREF/GAIN), but "the ADC accuracy is not maintained on the full
  extended range, but only on a smaller range, which is approximately equal
  to +/-1.05 x VREF";
- the 24-bit formats (00, 01) lock at 0x7FFFFF / 0x800000 instead;
- there is no over-range status bit: STATUS holds only DR, CRCCFG and POR.

The firmware therefore classifies every conversion as over range when
|code| > 2^23 - 1 (`Core/Inc/daq_over_range.h`), the same code threshold at
every gain: +/-VREF at the CH0/CH1 pins, +/-VREF/16 (+/-150 mV nominal) on P6,
+/-11 VREF (26.4 V nominal) at the HVAIN connectors, and the die temperature.
An over-range conversion:

- is written to SD with its code and a flag (SD format v4, `value` = 1/-1,
  `calibrated_value` = +/-inf);
- never enters a median, report bin, statistic, filter or calibration;
- makes a report bin of only such conversions publish +/-INFINITY, which the
  station shows as OVR;
- reaches the rail checks as +/-INFINITY (at or past the rail) and is counted
  per channel in DAQ_INPUT_HEALTH v3 (words 18-24).

The 2026-11-16 fire's KG1000 code 0xB40000 (+11 796 480 = 1.40625 x 2^23)
is over range: beyond even the +/-1.05 VREF accuracy band and short of the
documented clip, so it is an overloaded code, not a voltage. At the nominal
2.4 V it would read 3.375 V; with this board's measured ~2.2 V reference
it corresponds to about 3.09 V at the pin.

## MCP3564R reference setting

Every MCP3564R volt is `code x VREF / 2^23` (`mcp3564r_code_to_voltage`, and
the die temperature, equation 5-1, scales with VREF too). The internal
reference is nominally 2.4 V +/-2 %. This board's part drifted to about
2.2 V after heat during repair; with 2.4 V assumed every MCP3564R reading
(KG1000/AMP2 pin volts, P6, the chamber PT and the other HVAIN inputs, the
temperature) reads about 9 % high.

The reference is therefore a per-board setting:

- The station owns it and sends it as the managed variable
  `DAQ_MCP_REFERENCE` (Float32 volts). The DAQ requests it every boot until
  it arrives, like the calibrations and `DAQ_KG50_INPUT`, keeps a flash copy
  (key `DAVR`) for boots without the station, and refuses values outside
  1.9-2.6 V. Until either arrives it uses the nominal 2.4 V.
- It is reported in DAQ_INPUT_HEALTH word 27 (mV, and whether it came from the
  station or flash), in the config identity CRC (BOARD_HEALTH word 14, after
  the filter floats; flags bits 9 and 10), and in every SD `# calibration`
  line (`mcp_vref_v`). SD keeps raw codes, so any file can be re-converted.
- Changing it changes every published volt by r = new/old. Load-cell
  calibrations are kg per volt, so the DAQ converts its on-board copy in the
  same step (`daq_calibration_scale_input`: KG1000 slope / r, KG50 c_i / r^i
  and x0 x r, thermal V/C x r; intercepts, c0, tare and reference
  temperatures unchanged), which keeps kg for every code unchanged. The
  station stores the reference each calibration was fitted under and
  converts by the same rule, then re-sends.
- The code-based over-range threshold does not depend on it. The rail
  checks' volt limits (`DAQ_FAULT_ADC_FULL_SCALE_V`) are unchanged: an
  in-range conversion is below full scale at any reference, so the high rail
  is reached only through over range.

**Measuring it.** With the board powered, measure P7 (or P8) pin 6 (+VREF,
the MCP3564R REFIN+/OUT pin) against pin 2 or 5 (GND) with a DMM of at least
4 1/2 digits, after 1 minute of warm-up, and enter the value in the station
(Settings, Ground Station, Loadcells). Measure it twice, with the load cells
and P6 bridge connected as flown and with them unplugged. The internal
reference buffer is rated for only 8 mA into a short (DS20006391C, IREF_SC),
and the P6 bridge and the amplifier bias dividers hang on +VREF (or
+VREF_BUF; check which on the board). A 350 ohm bridge at 2.4 V alone would
draw about 7 mA. If the two readings differ, the low reference is loading,
not drift, and the value to enter is the loaded one, since that is what the
ADC converts against. The reference is also specified only for AVDD
2.7-3.6 V. AVDD is +3V3A (schematic, ADC & VREF sheet), so while a clamped
front end lifts +3V3A above 3.6 V (DAQ_INPUT_HEALTH faults 8/9, word 25),
every MCP3564R reading, the reference included, is outside the datasheet. There is no firmware self-measurement:
the schematic shows no path without external wiring. +VREF goes to the bias
dividers, the +VREF_BUF buffer and connector pin 6; the DAC outputs go to
their connectors with ADC4 readback; neither reaches an STM32 ADC input or an
MCP3564R input. Confirm this on the board before building on it. Two jumpered
options would allow a "calibrate reference" action later, both measured
against the VREFINT-corrected VDDA and so valid only once the D24 clamp no
longer lifts +3V3A:
- +VREF to a spare SAR input;
- a DAC output into a spare MCP3564R input (CH4/CH5), read by both ADCs. Current value for this board: 2.2 V, measured approximately;
re-measure after the December repair.

## Software verification

`tests/test_acq_schedule.py` runs the complete production driver
(`tests/acq_engine_sim.c`) against a modelled TIM2, SPI DMA, DRDY pin and ADC.
It verifies the register bursts per scan, strict 1 kHz fast-pair timing in
both profiles, the slot rotation, spurious DRDY edges, active-low STATUS
validation and recovery, lost DMA completion recovery, lost DRDY edges and
the analog-enable failure path.
The mock supplies
ADC frames; it does not prove that physical bridge voltages change under load.

`tests/test_kg50_acquisition.py` checks separation of interleaved samples into
independent averages and calibrated SD records. The network streams use the
median and microsecond-bin filters of `daq_median.h`, with separate state. Both publish raw float32 values and retain their own GS calibration.

## Checking a flat reading

### Live plot without reflashing

Run `python3 tools/loadcell_plot.py` with the ST-Link attached to the DAQ.
Python 3 and OpenOCD must be installed. The script opens a browser GUI at
`http://127.0.0.1:8765`, verifies the running firmware against
`build/Release_Script/DAQ-Board.elf`, and reads the ADC ring buffer while the
processor keeps running. Use `--elf PATH` if a different ELF was flashed.
If OpenOCD is already serving the DAQ on TCL port 6666, add `--attach`.
Avoid starting a second debugger against the same probe.

The GUI defaults to CH0/P7 (1000 kg) and CH9/P6 (50 kg differential). Each
panel can select CH0–7 or CH9, including chamber pressure on CH6/P4 pin3.
Select ADC counts, gain-corrected ADC-pin volts, raw input, converted value or
ADC temperature. CH6 converted values use the default 11:1 divider and
0–5 V / 0–1000 psi chamber sensor. CH0/CH1 mass uses saved calibration;
CH9 mass is deliberately absent because P6 calibration belongs to
GroundStation, not the firmware. Raw P6 differential volts remain available.
Current firmware has 16-byte entries and 17 calibration floats; the tool checks
symbol sizes and every immutable flash section against the selected ELF before
reading samples, then displays its SHA256 identity. Small ring blocks are
double-read and changed entries discarded so a slow debug scan cannot accept
a torn whole-ring snapshot. **Set plot baseline**
subtracts the displayed starting value without changing firmware calibration.
Stale samples are marked. CSV files are recorded under `build/loadcell-plots/`;
**Download visible CSV** exports the current time window. Ctrl+C stops the
script and any OpenOCD process it started. With `--attach`, the existing
debugger is left running.

The SD health section reads verified firmware counters once per second.
**WRITING** requires an observed increase in raw or CSV rows, not merely a ready
flag. It shows init stage, errors, cumulative written/flushed counters, protected
drops, observed row rate and time since last progress. Missing/failed/stale reads
show **UNKNOWN**; a ready card without progress shows **READY / IDLE**. Raw and
PT flush counters are displayed separately because CSV has no independent
flushed-row counter.

This is a sampled debug capture, not a lossless substitute for the SD logs.
The firmware's saved calibration is used to calculate unsmoothed CH0/CH1 kg; identity defaults
must be calibrated using GroundStation before treating that display as mass.

### Analog measurements

Sheet 10 gives AMP1 a nominal 2.4/11 V bias (about 0.218 V) and AMP2 a
nominal 2.4/4 V bias (0.600 V). Each input has a 1 MΩ pull-down. With the
cell disconnected, an output near its bias with noise is plausible; this
does not establish whether a connected cell responds to load.

For a connected cell on P8, compare these measurements unloaded and loaded:

1. Pin 1 relative to pin 2: excitation should be near 9.6 V.
2. Pin 3 relative to pin 4: the small differential bridge voltage should change.
3. `AMP2_OUT` / ADC CH1 relative to ground: the amplified voltage should change.
4. KG50 raw and then GS calibrated output: both should follow the change.

If the bridge differential does not change, investigate excitation, wiring,
the cell, and load application. If it changes but AMP2_OUT does not, investigate
the amplifier and clipping. If AMP2_OUT changes but ADC codes do not, investigate
ADC acquisition. A constant scale error alone cannot make a changing ADC code
constant. Establish this response before using the GS zero/known-mass routine.
