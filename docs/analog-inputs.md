# Analog input map and testing

Source: `Schematic PDF_[No Variations] (2).pdf`, revision A, sheets 1–3 and
7–10. All exposed inputs below are independent, ground-referenced signals.
The existing load-cell connectors P7/P8 remain MCP CH0/CH1, with unchanged
KG1000/KG50 raw calibration coordinates and gain/filter settings.

## Connector map

| Signal | Connector pin | ADC / MCU pin | Telemetry and zero-based index |
|---|---|---|---|
| SAR_IN1 | P9/3 | ADC4 IN1 / PC0 | DAQ_SAR_VOLTAGES[0] |
| SAR_IN2 | P9/4 | ADC4 IN2 / PC1 | DAQ_SAR_VOLTAGES[1] |
| SAR_IN3 | P3/3 | ADC4 IN3 / PC2 | DAQ_SAR_VOLTAGES[2] |
| SAR_IN4 | P3/4 | ADC4 IN4 / PC3 | DAQ_SAR_VOLTAGES[3] |
| SAR_IN5 | P2/3 | ADC1 IN5 / PA0 | DAQ_SAR_VOLTAGES[4] |
| SAR_IN6 | P2/4 | ADC1 IN6 / PA1 | DAQ_SAR_VOLTAGES[5] |
| SAR_IN7 | P1/3 | ADC1 IN7 / PA2 | DAQ_SAR_VOLTAGES[6] |
| SAR_IN8 | P1/4 | ADC1 IN8 / PA3 | DAQ_SAR_VOLTAGES[7] |
| MCP CH2 | P6/3 | MCP3564R CH2 | DAQ_SDADC_VOLTAGES[0] |
| MCP CH3 | P6/4 | MCP3564R CH3 | DAQ_SDADC_VOLTAGES[1] |
| MCP CH4 | P5/3 | MCP3564R CH4 | DAQ_SDADC_VOLTAGES[2] |
| MCP CH5 | P5/4 | MCP3564R CH5 | DAQ_SDADC_VOLTAGES[3] |
| HVAIN1 | P4/3 | MCP3564R CH6, 10k/1k divider | DAQ_SDADC_VOLTAGES[4] |
| HVAIN2 | P4/4 | MCP3564R CH7, 10k/1k divider | DAQ_SDADC_VOLTAGES[5] |
| ISENSE1_OUT | J7 current path, U3 output | ADC1 IN12 / PA7 | DAQ_CURRENT_SENSE[0] volts, [2] amps |
| ISENSE2_OUT | J8 current path, U4 output | ADC1 IN11 / PA6 | DAQ_CURRENT_SENSE[1] volts, [3] amps |
| VMON | internal R57/R69 | ADC1 IN17 / PB2 | DAQ_POWER_MONITOR[0] ADC volts, [2] VMAIN volts |
| IMON | internal U14/R60/R56 | ADC1 IN16 / PB1 | DAQ_POWER_MONITOR[1] ADC volts, [3] input amps |

Architecture labels call PC2/PC3 ADC1_IN3/4; the same physical pins also support
ADC4_IN3/4, used here with PC0/PC1. P1/P2/P3/P9 use unity-gain buffers, not
pairwise differential measurement. Their pins 1/6 supply +3V3A, pins 2/5 are
GND. P4 pins 1/6 supply +12VA/+3V3A; P5/P6 pins 1/6 supply +VREF_BUF/+3V3A.
These are supply outputs, not analog input pins. Input clamp rails and connector
supply voltages do not specify a calibrated measurement range.

## Nominal conversion and acquisition

STM32 ADC1 uses single-ended 14-bit conversion; ADC4 uses 12-bit conversion.
Both use nominal VDDA=3.3 V. ADC4 explicitly clears the previous channel selection
before each conversion: HAL channel selection is additive, so leaving all four
selected and restarting after the first conversion can repeat IN1.

MCP CH2–5 report ADC-pin volts (`code * 2.4 / 8388608`); CH6/7 report connector
volts after multiplying by 11. No load-cell thermal compensation, smoothing or
mass calibration is applied to these channels. Raw SD records retain ADC-pin
volts and code; their `calibrated_value` column contains nominal connector volts.

TMCS1108A1B estimates use `(Vout - 3.3/2) / 0.05` amperes. PC5 drives Q1's gate
low to power both sensors. VMAIN uses `VMON * 11`. LM74202 input current uses
`IMON / (78.28e-6 * 6200)` amperes; below about 50 mA the monitor offset dominates.
These estimates require hardware validation/calibration; output volts are also
provided so later sensor calibration does not require changing the driver.
See [TI TMCS1108-Q1](https://www.ti.com/lit/gpn/TMCS1108-Q1),
[TI LM74202-Q1](https://www.ti.com/lit/ds/symlink/lm74202-q1.pdf), and
[STM32U585 pin mapping](https://www.st.com/resource/en/datasheet/stm32u585zi.pdf).
The old ADC1 code incorrectly subtracted SAR5–6, SAR7–8 and ISENSE2–1 and
misidentified them as supply voltage/current and auxiliary voltage. Old readings
of those diagnostic fields are not comparable to the corrected measurements.

Internal ADC inputs are sampled once per `DAQ_ANALOG_REPORT_PERIOD_MS` (100 ms),
not on every 2 ms load-cell service iteration. Per-channel read failure produces
NaN and does not prevent the other inputs or external ADC queue from being read.
The current sensor's unconnected *current path* should nominally report near
zero amps; open voltage inputs may float and are not detected as disconnected.

MCP CH0/CH1 remain the normal scan. Every 100 ms, a bounded CH2–7 pass uses the
same reference, gain, clock and OSR, then restores CH0/CH1. Temperature has a
separate, higher-priority slow-clock measurement. The auxiliary pass shares
conversion time with load cells; their achieved rate must be measured again.
A 25 ms auxiliary timeout restores the normal scan if a conversion is missing.
`MCP3564R_AUX_SCAN` in `mcp3564r_board_config.h` can mask individual CH2–7 inputs
or be zero to disable the auxiliary pass. Unread/stale fields report NaN after
300 ms. The unchanged load-cell raw encoding preserves saved calibrations.

Each drained MCP sample is queued to SD with its channel and original timestamp:
CH0 `mcp3564r_raw`, CH1 `kg50_raw`, CH2–7 `mcp3564r_chN_raw`. Internal analog
voltage/current rows are captured at 10 Hz. A bounded, time-sliced worker
publishes analog telemetry and formats these rows with the original capture
timestamp, outside the load-cell acquisition loop. `g_daq_analog_work_drop_count`
counts snapshots rejected if all three worker slots are occupied. The worker
shares the acquisition/SD priority with a 1 ms slice, so a continuously busy SD
writer cannot starve analog reporting. Queue/write diagnostics
still determine whether storage kept up; this feature does not establish a
lossless 12 kHz recording qualification.

## Network and GroundStation

Loadcells default to **250 reports/s each** (500 combined), with a 4 ms
downsampling window. ADC acquisition and raw SD capture keep their own rates.
`cmake -S . -B build/Release_Script -DDAQ_BROADCAST_RATE_HZ=500` enables an
explicit 500 Hz-per-channel test; use `250` to restore the normal setting.

Loadcell and temperature publication now runs in a separate worker with 16
bounded report slots. Acquisition submits without waiting. Reports retain the
captured values, calibration and time; network timestamps subtract queue age.
`g_daq_report_drop_count` counts rejected work items (which may contain both
loadcells), and `g_daq_report_max_age_ms` records the maximum worker delay.
These counters are independent of raw ADC/SD loss counters. No new hardware
throughput qualification has been performed for this worker.

Append-only SEDSnet IDs (existing IDs unchanged):

| ID | Type | float32 count | Order |
|---|---|---|---|
| 143 | DAQ_SAR_VOLTAGES | 8 | SAR1..8 volts |
| 144 | DAQ_SDADC_VOLTAGES | 6 | CH2,3,4,5,HVAIN1,HVAIN2 volts |
| 145 | DAQ_CURRENT_SENSE | 4 | J7 V, J8 V, J7 A, J8 A |
| 146 | DAQ_POWER_MONITOR | 4 | VMON V, IMON V, VMAIN V, input A |

All four target GroundStation, best-effort at 10 Hz, with explicit DAQ CAN routes.
They add approximately 40 messages/s and 880 payload bytes/s before framing.
GroundStation's matching runtime schema advertises the GROUND_STATION endpoint;
its default, test-fire and HITL layouts have a DAQ analog tab with explicitly
bound DAQ chart series and separate voltage/current axes, including both load cells.
The DAQ publishes its schema at startup; existing gateways merge the new type
definitions at runtime. Routine discovery polling alone does not send schema.
A failed initial announcement is retried once per second until queued successfully;
After discovering a peer, the board also requests its schema once, so restarting
a board does not depend on catching the peer's earlier startup advertisement.
Failed submissions retry at one-second intervals; successful submissions stop
repeating. This is not an acknowledgment that every peer has merged the schema.

Rebuild/reflash DAQ and rebuild/restart GroundStation to activate. The gateway
does not need the new analog definitions compiled in. Confirm the installed bridge's throughput and
unknown-type forwarding during bench testing. For a future sensor, choose its
existing type/index in the layout and add the physical-unit calibration in GS;
changing the fixed payload order requires a coordinated schema update.

Bench check with a known voltage below the input's documented limits, applied
to one input at a time. Verify the matching chart changes independently and its
SD channel label is correct, then test zero/known current on J7/J8 and compare
VMON to a meter. Check acquisition, queue-loss and SD-write counters while all
streams are active; host mocks cannot verify board wiring, reference accuracy,
physical sensor response or real SD/network throughput.

## Validation status

Host tests cover independent channel selection, nominal scaling, auxiliary scan
restore/timeout, stale values, wire/layout mappings and queued snapshot ownership.
The production release/factory build succeeds. The local simulator rebuilt from
FirmwareSimulator `a916318` reports both load-cell streams, 40 analog packets/s,
and no analog-queue, SD-row or raw-batch drops in the two-second diagnostic run.
The unchanged strict zero-overrun check still fails on one startup overrun;
the unchanged baseline firmware has three overruns under the same simulator.
The diagnostic run is not a full-suite qualification: memory/network soak stages
were not reached, and the pinned public v0.4.12 image was unavailable. Bench
validation remains required, especially for ADC accuracy and sustained throughput.
