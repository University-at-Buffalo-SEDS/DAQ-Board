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

The checked-in default `DAQ_CHAMBER_ADC_CHANNEL=-1` is **unassigned**. It performs
no chamber conversion, creates no chamber SD samples, and publishes the unchanged
scalar KG1000 telemetry. A real zero-pressure sample remains valid after input
assignment; lack of a sample never creates a zero-pressure reading.

The revision A schematic maps P4/3 HVAIN1 to MCP3564R CH6, and P4/4 HVAIN2 to CH7,
both through 10k/1k dividers. `DAQ_CHAMBER_ADC_CHANNEL=6` or `7` selects one explicitly.
P4/1 and P4/6 are supply outputs (+12VA/+3V3A), not signal inputs. Selecting P4
alone does not identify which input is wired. Existing clamp/supply voltages do
not establish a safe transducer supply or a calibrated measurement range.

An assigned channel joins the unity-gain fast scan and is removed from the slow
auxiliary scan. It is excluded from the alternate P6 differential gain 16 scan.
Raw chamber conversions use the same acquisition queue, timestamps, and SD
record ownership as the load cells.

**Bandwidth constraint:** two fast load cells currently share 3700 aggregate
reads/s, roughly 1850 each outside diagnostic scans. A third equally sampled
channel needs at least 5556 configured reads/s (180us per read) to preserve
the legacy rounded 271us/two-channel cadence. The existing OSR1024
conversion ceiling is 3906.25/s, so simply adding a third channel would slow the
load cells. The build rejects this. A candidate explicit configuration of OSR512
and 5556 aggregate reads/s builds, but changes filtering/noise and remains a
hardware qualification choice, not the default. The configured per-channel
rate also depends on diagnostic scans and the selected P6 mode. No physical
sample-rate or transducer-accuracy acceptance is claimed.

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

SD raw rows use sensor `chamber_pressure_raw_volts_psi`, the original ADC code,
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
