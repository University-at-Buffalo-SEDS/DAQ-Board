# Combustion chamber pressure

The specified transducer is **OMEGA PX309-1KG5V**: 0–1000 psi gauge with a 0–5 V
output and 9–30 VDC excitation. Its nominal conversion is connector volts × 200 psig/V.
The 5V-output cable uses red excitation +, black excitation −/return, white signal +.
The datasheet lists ±0.25% BSL combined static accuracy; this does not include the
DAQ divider/reference error or constitute calibration of the installed chain.
See [OMEGA PX309 datasheet, pages 2 and 4](https://assets.omega.com/pdf/test-and-measurement-equipment/pressure/pressure-transducers/PX309-100mv.pdf).

DAQ scales its measured **connector voltage** after the 11:1 divider. At 5 V input,
the ADC sees about 0.4545 V. It does not reuse ValveBoard's unrelated 3103-count /
2000 psi conversion, nor a 0.5–4.5 V model. Finite overrange measurements are retained
rather than clamped; the UI/storage range-status row flags values outside 0–1000 psig.
Physical wiring, rail capacity, calibration and operating envelope remain unverified.

## P4 input and sampling

The checked-in default selects `DAQ_CHAMBER_ADC_CHANNEL=6`: the installed
HVAIN1 input. Explicit `-1` disables chamber conversion and restores scalar
KG1000 publication; KG1000 is then the only fast channel. A real zero-pressure
sample remains valid; lack of a sample never creates a zero-pressure reading.

The checked-in schematic sheet 9 traces P4/3 HVAIN1 through R27/R31 (10k/1k),
U8B and R28 to MCP3564R U7 CH6. P4/4 HVAIN2 is CH7 and remains an explicit
override. P4/1 is +3V3A; P4/6 is +12VA. Pins 2/5 are ground. These supply
outputs are not signal inputs; the PX309 requires 9–30 V excitation, not 3.3 V.
The GPIO enable state does not establish a measured excitation voltage or
calibrated measurement range; see the README analog-power notes.

The assigned channel is a **fast channel**: it is converted with KG1000 in
scan A of every 1 ms frame (TIM2-timed, unity gain, AZ_MUX), so it is sampled
at exactly 1 kHz with a microsecond timestamp, and it never shares a scan with
the P6 gain-16 slot or the auxiliary rotation. FILL converts the fast pair at
OSR 128 (192 us per channel), LAUNCH (launch window and its 0.5 s pre-roll) at
OSR 256 (384 us per channel). Every conversion goes to SD as a
`chamber_pressure_v` raw record; the network gets the mean of each 20 ms bin of
the 3-sample running median (50 Hz, in `DAQ_LOADCELL_CHAMBER` with the KG1000
bin of the same centre). Schedule, bandwidth and latency:
[data pipeline](data-pipeline.md).

The PX309's response time (under 1 ms) is covered by the 500 Hz Nyquist band of
the 1 kHz stream. There is no analog anti-alias filter ahead of the ADC; the
sinc3 conversion window (192 or 384 us) is the only one, so content above
500 Hz aliases. The fast stream is never smoothed beyond the 3-sample median on
the published values; SD keeps the unfiltered conversions.

The original 230976-byte variable SD raw pool remains intact. An assertion checks
that 800 ms of configured raw conversions plus per-batch ownership/allocation
metadata and a maximum tail fit. Larger rates must not silently shorten this
reserve. SD mount buffering, FIFO ownership, partial-write retention, flushes,
record sequences, and drop/error/flushed counters use the existing DAQ paths.

## Network and SD records

When pressure is fresh, `DAQ_LOADCELL_CHAMBER` (wire ID 259346466) replaces the
single KG1000 publication in that existing scheduler slot. Its Float32 vector is
`[uncalibrated_kg1000, chamber_psig, chamber_connector_volts]`. It adds 8 payload
bytes, not another packet slot: default KG1000 and KG50 each retain 40 slots/s and
the aggregate sensor cap remains 100 packets/s. Acquisition and full raw SD
logging remain independent of network coalescing. Pressure older than 100 ms or invalid pressure at report preparation restores
scalar KG1000. Transport queues can still delay an accepted frame; capture
timestamps remain attached. No pressure is invented when an input is unassigned.

SD raw rows use sensor `chamber_pressure_v`, the original ADC code,
connector volts in raw_value, and psig in calibrated_value. They retain monotonic
and network clocks, clock validity, uptime frequency and record sequence. Each
new file includes the selected channel, model constants and units as a comment.
Existing schemas and legacy KG1000 payloads are unchanged.

GroundStation stores the original combined vector and payload, projects the
existing KG1000 path for calibration/mass, and stores separate
`COMBUSTION_CHAMBER_PRESSURE` (psig) and `COMBUSTION_CHAMBER_VOLTAGE` (V) rows with
source and receive clocks. Existing recording/replay/CSV export handles them.
All normal/HITL/test-fire UI layouts include pressure charts and the load-cell
section includes both pressure and raw voltage. New dashboard defaults include
a sender-scoped DAQ chamber gauge. Existing user dashboards can add that channel.

Deploy the updated DAQ schema/firmware and both GS/RRLS ingestion/layout configs
together before enabling an input; older GroundStations do not project the new
combined vector into legacy KG1000 rows. The frontend change supplies a default
card; the backend layout supplies Data charts to separately built clients.
Gateway/relay code remains generic and uses discovery for the additive type.
No SDK reserved IDs or board control/ACK policy changes are involved.

A 0V-output sensor cannot distinguish real zero gauge pressure from every
possible wiring/power fault by voltage alone. The range flag identifies values
outside the nominal range; it does not certify sensor health.

The live input-fault check (`DAQ_INPUT_HEALTH`, `Core/Inc/daq_input_faults.h`)
therefore looks for "no signal" (below 0.05 V for 2 s) only while the chamber
must be pressurized: from T0 + 1.5 s to T0 + 5 s, where T0 is the launch
`DAQ_LOG_CLOCK` deadline minus 20 s. The start leaves margin for ignition
delay (thrust began at T0 + 1.0 s in the 2026-11-16 fire); the end stays inside
the burn (it ended at T0 + 5.4 s). In that fire CH6 read 0.008-0.020 V through
the burn, which this catches by T0 + 3.5 s. At rest, during fill, in HITL with
no launch, or in a build without the SD launch clock the chamber fault stays
inactive; the reading is still reported. A transducer with a live zero
(`DAQ_CHAMBER_LIVE_ZERO_V` of 1.0 or 0.5 V) is checked at all times. The DAQ
cannot tell an absent 0-5 V PT from a vented one either: when no chamber PT is
connected (as in the 2026-11-16 fire), mark it not installed on GroundStation
(`chamber_pt_installed`), which hides and silences this check and shows the
chamber as not installed instead of 0 psig.
