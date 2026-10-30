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
KG1000 publication plus the legacy OSR1024/read3700 profile. A real zero-pressure
sample remains valid; lack of a sample never creates a zero-pressure reading.

The checked-in schematic sheet 9 traces P4/3 HVAIN1 through R27/R31 (10k/1k),
U8B and R28 to MCP3564R U7 CH6. P4/4 HVAIN2 is CH7 and remains an explicit
override. P4/1 is +3V3A; P4/6 is +12VA. Pins 2/5 are ground. These supply
outputs are not signal inputs; the PX309 requires 9–30 V excitation, not 3.3 V.
The GPIO enable state does not establish a measured excitation voltage or
calibrated measurement range; see the README analog-power notes.

The assigned channel joins the unity-gain fast scan and is removed from the
slow auxiliary scan. It is excluded from the alternate P6 differential gain 16
scan. Raw chamber conversions use the same owned acquisition FIFO, timestamps,
and copied SD batches as the load cells.

**Settled conversion budget:** polling frequency is not fresh sample rate.
The MCP3564R SCAN filter settles separately for every channel, and the retained
AZ_MUX setting doubles conversion time. At 16 MHz MCLK, the legacy OSR1024
profile takes 1024 us per channel before service overhead. Three channels at
OSR512 take 768 us each, reducing the two load cells' density. The default is
therefore OSR256/read5556:384 us per settled channel, nominal 1152 us per
three-channel cycle before diagnostics and service overhead. Clock, gain,
auto-zero and calibration scales remain unchanged. OSR256 provides less
oversampling/noise rejection than OSR1024; equal noise or effective resolution
is not promised. Physical noise and achieved cadence still need measurement.
See [MCP3564R SCAN/AZ_MUX and filter timing](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf).

The production-driver SCAN/DRDY model includes full per-channel settling,
auxiliary/temperature passes and 2 ms acquisition service. It compares the
legacy two-channel profile to the default three-channel profile, requires
at least the original fresh load-cell density, and uses the actual SD enqueue
routine to verify immutable ownership through a 600 ms writer stall. A deliberate
acquisition stall must expose exact FIFO overrun accounting. Virtual SPI service
is instantaneous in this model; hardware CPU timing and real-card bandwidth
are separate qualification measurements.

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
