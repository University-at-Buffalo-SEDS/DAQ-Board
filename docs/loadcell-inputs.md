# Load-cell inputs

The repository's `Schematic PDF_[No Variations] (2).pdf`, sheets 1, 9, and 10,
maps the load-cell connectors as follows:

| Connector | Amplifier | MCP3564R input | Telemetry |
| --- | --- | --- | --- |
| P7 | AMP1, U9/U10 | CH0 | KG1000 |
| P8 | AMP2, U11/U12 | CH1 | KG50 |

Both amplifiers accept differential bridge signals and produce voltages
referenced to ground. The ADC must scan CH0 and CH1 as separate single-ended
inputs (`SCAN=0x000003`). Selecting CH0 minus CH1 would subtract the outputs
of two different load cells.

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
selects PGA gain 1 (`GAIN[2:0]=001`). Both scan channels use that configuration;
the register is shared. `CONFIG3=0xF0` includes the channel ID and signed
25-bit result in each 32-bit read. Nominal ADC input voltage is:

```text
volts = signed_code * 2.4 / 8388608
```

The complete shared setup retains the working input's analog settings:

| Register | Written value | Applies to both inputs |
| --- | --- | --- |
| CONFIG0 | 0x82, then START makes it 0x83 | Internal reference, external MCLK, bias current sources off |
| CONFIG1 | 0x14 | MCLK/1, OSR 1024 |
| CONFIG2 | 0xCF | Gain 1, boost 2, MUX/reference auto-zero enabled |
| CONFIG3 | 0xF0 | Continuous scan cycles, tagged 32-bit data, hardware calibration off |
| SCAN | 0x000003 | CH1-AGND and CH0-AGND |
| TIMER | 0x000000 | No extra delay between scan cycles |

SCAN overrides the MUX register, so its retained `MUX=0x08` does not restrict
conversions to CH0. The scanner visits CH1 then CH0, and each result is routed
by its returned channel ID. The driver sends START once; it does not restart
the scan at each read. There is no separate CH1 gain/reference/filter register
to initialize. Auto-zero and scan settling consume conversion time, and the
two inputs share the available conversion rate.

The previous voltage calculation included a divide by 16 despite configuring
gain 1, and used 2.2104 V rather than the nominal internal reference voltage.
Firmware now uses the corrected formula for voltage diagnostics. It retains
the historical raw scale for KG1000, KG50, GS calibration, and SD raw values
so saved coefficients and captured calibration points remain compatible.
SD diagnostic rows are `mcp3564r_voltage_v` for CH0 and
`mcp3564r_ch1_voltage_v` for CH1.

The register definitions and code scaling are described in the
[Microchip MCP3561/2/4R datasheet](https://www.microchip.com/content/dam/mchp/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf),
sections 5.6, 5.15, 6.2–6.4, and 8.2–8.8.

## Software verification

`tests/test_mcp3564r_driver.py` compiles the complete production driver against
a mocked HAL/SPI boundary. It verifies the actual transmitted register writes,
identical scaling for identical codes on either channel, sign extension,
changing CH1 samples with CH0 held constant, ring-buffer wrap, skipped polls
without fresh data, and recovery after a DMA start failure. The mock supplies
ADC frames; it does not prove that physical bridge voltages change under load.

`tests/test_kg50_acquisition.py` checks separation of interleaved samples into
independent averages and calibrated SD records. Both network streams then use
the same `daq_downsample_add` function and reporting interval, with separate
state. Both publish raw float32 values and retain their own GS calibration.

## Checking a flat reading

### Live plot without reflashing

Run `python3 tools/loadcell_plot.py` with the ST-Link attached to the DAQ.
Python 3 and OpenOCD must be installed. The script opens a browser GUI at
`http://127.0.0.1:8765`, verifies the running firmware against
`build/Release_Script/DAQ-Board.elf`, and reads the ADC ring buffer while the
processor keeps running. Use `--elf PATH` if a different ELF was flashed.
If OpenOCD is already serving the DAQ on TCL port 6666, add `--attach`.
Avoid starting a second debugger against the same probe.

The GUI shows separate CH0/P7 and CH1/P8 traces, with selectable ADC counts,
nominal volts, raw calibration input, and calibrated kg. **Set plot baseline**
subtracts the displayed starting value without changing firmware calibration.
Stale samples are marked. CSV files are recorded under `build/loadcell-plots/`;
**Download visible CSV** exports the current time window. Ctrl+C stops the
script and any OpenOCD process it started. With `--attach`, the existing
debugger is left running.

This is a sampled debug capture, not a lossless substitute for the SD logs.
The firmware's saved calibration is used to calculate kg; identity defaults
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
