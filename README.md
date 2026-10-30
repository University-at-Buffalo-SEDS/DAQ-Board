# DAQ Board firmware

The DAQ Board targets the STM32U585 and samples fill-system instrumentation for
publication over the SEDSNet CAN-FD network. The board uses rolling SEDSNet `main` and
SEDS LaunchCore v1.0.0, fetched by CMake without submodules.

LaunchCore derives the 2 MiB flash layout, linker scripts, Slot A, OTA staging,
and persistent regions from `Bootloader/board_config.h`. OTA artifacts use the
`.seds` extension and the build selects the update form supported by that
layout.

```sh
./build.py build --release
./build.py flash --release --method stm32prog-cli
./build.py clean
./build.py test
./build.py test --all --release
./build.py test --all --release --ultra-soak
```

On Docker hosts that cannot create bridge interfaces (including the Jupiter
validation host), prefix the command with
`SEDS_FIRMWARE_SIM_DOCKER_NETWORK=host`. The linked test requires GroundStation
to label all seven graph nodes, attribute real payload traffic to each board,
and correlate a routed valve command with its returned state ACK.

`--ultra-soak` keeps the normal 16-second full-network test first, then adds a
separate 600,000 ms firmware-time fault/rejoin, command/ACK, and memory-leak
qualification. Commands must execute and return an ACK throughout the soak,
including its final interval.

The normal blank-board flash writes the complete factory image at `0x08000000`.
Use `./build.py flash --help` for alternate programmers.

## SD-card data logging

Insert the SD card before powering the DAQ Board. On first use, firmware looks
for the `SEDSDAQ.ID` provisioning marker. If the marker is absent—or the volume
cannot be mounted—the entire card is erased and formatted as FAT, and the marker
is created. Later reboots see the marker and **do not reformat the card**. Do not
insert a card containing data you want to retain unless it was already prepared
by this firmware and contains that marker.

Firmware creates a new `DAQ_<network-time>_<sequence>.CSV` raw ADC file on every boot.
Raw ADC channels go into this file. Filtered network rows, other telemetry
snapshots and packet diagnostics go into separate
`DAQ_TELEMETRY_<network-time>_<sequence>.CSV` files. Each stream rotates
independently when calibration changes or before a file reaches 3 GiB.
Delayed telemetry cannot mix into or rotate the raw ADC log.

DAQ and FC share the [SD CSV format](docs/sd-record-format.md). Records include
network/local time, independent 64-bit boot uptime ticks, the nominal clock
frequency (1000 Hz), and a per-stream sequence. Uptime remains usable when
network UTC is corrected. Raw rows preserve ADC code, raw/calibrated values
and ADC temperature. Failed and partial writes retain their pending records;
queue rejection and storage errors are counted. The default live network cap
is 100 sensor packets/s total; the separate load-cell SD report target remains
250 rows/s per loadcell, using captured fresh samples.
Every file begins with the four active
load-cell calibration coefficients and contains both raw and calibrated sample
values. Values use nine-significant-digit decimal scientific notation, preserving
the original binary32 samples on replay without requiring newlib float printf.
GroundStation publishes those coefficients as retained
`DAQ_LOADCELL_CALIBRATION` (ID 137); DAQ stores them in LaunchCore persistent
storage. A changed calibration closes the current log and starts a new file so
one file is never interpreted using mixed calibration epochs.

Before removing the card, power the board down normally and wait a few seconds.
The writer batches records into 4 KiB writes and flushes the FAT volume once per
second (and immediately when the input-voltage power-loss threshold is crossed).
Raw batches are serviced with bounded work so CSV rows and flushes cannot starve.
The networking worker polls at a one-millisecond interval with the 1 kHz
RTOS tick, and scans its stack only every 250 ms. Acquisition accounts for work
time in its 2 ms period and exposes overruns as `g_daq_sample_overrun_count`.
CAN reception drains the three-frame hardware FIFO from its interrupt into a
64-slot software ring. Waiting for the worker's millisecond poll can lose burst
traffic, including discovery fragments needed to refresh a working route.
`g_fdcan_rx_hw_overflow_count` counts hardware loss notifications;
`g_fdcan_rx_ring_drop_count` counts frames rejected by the full software ring.
The ISR leaves the consumer's tail alone when the ring is full.
RX notifications are enabled only after HAL enters its started state, and
overflow-only notifications also drain retained frames rather than leaving a
full FIFO blocked after the original new-message event was cleared.
When the transmit FIFO is full, a running ThreadX caller sleeps one tick
between checks, retaining the five-millisecond timeout. This prevents a
disconnected CAN bus from starving the lower-priority SD writer. Interrupt
and startup callers do not sleep. The isolated-CAN test requires zero SD row
or raw-batch drops and at least two observations after its startup warmup.
TIM2 runs at 1 MHz (160 MHz input divided by 160), matching the ADC driver's
microsecond conversion delays; its prescaler is also recorded in CubeMX.
The created FAT volume is directly readable on macOS, Linux, and Windows.

The SD card is optional for board operation. If it is absent or cannot be
mounted, acquisition and SEDSNet still start, a rate-limited `WARNING` is sent
to GroundStation, and firmware retries card detection once per second. A card
inserted later begins a new timestamped log without rebooting the DAQ board.
Initial card negotiation also runs in the SD worker: pre-scheduler startup only
configures the handle. An unready card must not delay starting CAN/acquisition.

The runtime schema is `config/sedsnet.json`. The STM32U585 memory map, ADC and
other simulated devices, firmware artifacts, probes, and fault cases are in
`sim/board.json`. Full validation builds release firmware and OTA images,
enforces chip and allocator limits, boots the real ELF in FirmwareSimulator,
profiles long-running traffic, and verifies linked discovery, synchronization,
managed variables, and command/ACK traffic.


## Regenerating with STM32CubeMX

Open the checked-in `.ioc` file and generate with the CMake toolchain. Keep user
code enabled. The `.ioc` is the source of truth for the ThreadX and USBX pool
sizes; unit tests compare those values with the generated Azure RTOS headers so
regeneration cannot silently shrink, grow, or repartition the pools.
Keep SD card negotiation deferred to the SD worker when regenerating
`MX_SDMMC1_SD_Init`; `tests/test_sd_startup.py` checks this startup constraint.

The top-level CMake project is board-owned and reconnects generated STM32
sources with SEDSNet, LaunchCore, its generated linker scripts, persistence, and
the simulator probes. After generation, run
`python3 build.py test --full --release` before flashing or committing.

### Compile-time acquisition and telemetry rates

Normal network qualification keeps onboard ADCs connected. Network outages
must not stop acquisition or SD logging; the raw-acquisition and SD-write
counters must advance between every post-startup simulator observation.
ADC failure/disconnection injection belongs in separate peripheral-fault tests,
not the normal network-reconnection layout. A nonzero counter at startup alone
is not evidence of continued operation.
The normal layout uses a synthetic 1 GiB SD card so raw logging has room during
the ten-minute high-rate soak. This is test media capacity, not MCU RAM and not
a claim about an installed card. Small/full-card tests are separate from
network-loss qualification.

Edit `Core/Inc/daq_rates.h` and rebuild/reflash:

- `DAQ_ADC_OSR`: hardware ADC oversampling filter (default 256 with chamber input; 1024 when explicitly disabled).
- `DAQ_ADC_READ_RATE_HZ`: requested timer-paced ADC reads (default 5556 with chamber input; 3700 when explicitly disabled).
- `DAQ_ACQUISITION_PERIOD_MS`: raw queue drain cadence (default 2 ms).
- `DAQ_BROADCAST_RATE_HZ`: filtered report/SD replay cadence (default 250 Hz per loadcell).

Edit `Core/Inc/daq_network_config.h` for the aggregate live rate and split:

- `DAQ_NETWORK_RATE_HZ`: **100 sensor packets/s total**, independent of ADC and SD rates.
- `DAQ_NETWORK_WEIGHT_KG50` and `DAQ_NETWORK_WEIGHT_KG1000`: **10 each**, giving both loadcells equal top priority and 40 packets/s each at the default total.
- `DAQ_NETWORK_WEIGHT_TEMPERATURE`, `_SDADC`, `_SAR`, `_CURRENT`, `_POWER`: **1 each**, giving each auxiliary stream 4 packets/s.

Shares are normalized by the total weight. A zero weight disables only that
stream's live network output; at least one weight must be positive. Each stream
keeps one bounded latest sample. Coalescing replaces unsent NETWORK work after
the original SD mirror enqueue; it never removes acquisition or SD records.
The selected 50 kg input remains one stream, including when its input changes.
A single worker interleaves these samples and caps actual SDK submissions using
monotonic time. Stalls do not produce catch-up floods. Heartbeats, commands,
acknowledgements and discovery bypass this sensor-only scheduler.

ADC acquisition, raw SD batches, calibrated records and the 250 Hz loadcell SD
mirrors retain their cadence and capture timestamps. `g_daq_network_offered`,
`g_daq_network_coalesced`, `g_daq_network_sent` and `g_daq_network_failed` expose
per-stream counters in the order declared in `daq_network_schedule.h`.
Coalescing is intentional; an SDK refusal is a separate failure counter.

```sh
# Uses the total and split from daq_network_config.h:
python3 build.py build --release
python3 build.py flash --release --method stm32prog-cli
# Optional total-rate override; the header weights still apply:
python3 build.py build --release --network-rate-hz 100
```

A normal build clears a previous CMake rate override and uses the header again.
The optional CLI override accepts 1..1000; actual delivered rate also depends on
available source samples and transport capacity. Empty weighted slots stay empty.

The raw SD byte pool budgets 800 ms at the configured acquisition rate.
Batch capacity scales with the acquisition interval (16 records per batch and
401 full-batch equivalents at the defaults); the pointer queue can hold
smaller batches within that same byte budget. No stale
reports are burst out after a missed deadline. Fresh ADC data, CPU scheduling,
CAN capacity and the Gateway/Pico-Fi link constrain actual received cadence;
100 aggregate live sensor packets/s is a cap, not measured end-to-end throughput.

The report window preserves its phase across scheduler jitter. A low-voltage
SD flush notification no longer permanently switches acquisition to a 100 ms
sleep; telemetry retains its configured cadence while the processor is running.
`g_daq_loadcell_publish_ok_count` counts accepted router enqueues, not delivery.
For transport diagnosis, compare it with `g_daq_loadcell_wire_tx_count`,
`g_daq_loadcell_wire_fail_count`, and `g_daq_queue_service_error_count`.
Only GroundStation receipt confirms end-to-end delivery.

Locally generated `KG1000` and `KG50` have explicit CAN routes so loss of the discovered
GroundStation endpoint does not suppress its configured stream while other peers
remain discovered. The exception applies only to local load-cell data; other
traffic continues to use discovery routing.
The current per-type wire counters inspect raw packet headers and do not decode
the `SDT` compact transport wrapper. With that profile, their zero values do not
prove a missing load-cell stream. Use total FDCAN counter deltas and receiver
observations until those diagnostics support the transport wrapper.

For live route diagnosis over ST-Link, set `g_telemetry_diag_request` to 1 and
wait for it to return to 0. The telemetry thread exports runtime and topology
JSON into `g_telemetry_diag_runtime` and `g_telemetry_diag_topology`, with result
codes and `g_telemetry_diag_sequence` marking completion. These snapshots are
on-demand only. Read the buffers while the request is idle; do not call router
functions from GDB while firmware is halted. Runtime JSON includes per-type TX
counts, and topology JSON includes each announcer's last-seen time and age.

For a requested 12 kHz read ceiling, select OSR 256 and drain every 5 ms.
This is not an exact 12 kHz conversion clock: with the physical 16 MHz MCO,
OSR 256 has a 15.625 ksps continuous single-channel rate. Two-channel SCAN
mode settles the filter at each channel change, reducing the conversion rate;
reads are paced at rounded microsecond
intervals plus SPI/interrupt overhead. Only acquired readings are recorded.
Do not claim lossless capture of every ADC conversion with this timer-driven
driver. The compile-time checks reject settings exceeding conversion speed or
raw batch capacity. SD throughput and actual achieved rate still require testing.

A sample-weighted boxcar averages acquired raw samples before downsampling.
Individual raw ADC records remain unfiltered on SD, while `kg1000_network`
contains the identical uncalibrated filtered value sent over SEDSNet.
GroundStation applies display calibration; SD raw records separately retain
calibrated values and calibration coefficients. Averaging
reduces broadband noise, but is not a sharp anti-alias filter and does not
guarantee rejection of arbitrary interference. Lower OSR trades noise for speed.
See Microchip MCP3561/2/4R datasheet DS20006391C, conversion-time table.
# SD initialization limits

The SDMMC kernel runs at 80 MHz; divider 2 limits card transfers to 20 MHz
without requiring UHS negotiation. Keep `SDMMC1.ClockDiv=2` in CubeMX.
Keep SDMMC hardware flow control enabled in both CubeMX and the generated
handle configuration: polling transfers share CPU time with acquisition and
networking. The block adapter waits for card programming to complete between
writes, with a bounded timeout and RTOS yields. Partition boot-sector writes
use the mounted partition offset rather than overwriting the partition table.
The board CMake applies `sd_hal_limits.h` to HAL sources, limiting software
status waits to 2 seconds rather than the vendor's effectively infinite wait.
The FileX block adapter uses polling HAL transfers, not DMA. Its CPU-written
read buffer must not be cache-invalidated before copying to FileX. Host tests
cover multi-chunk read/write integrity and propagation of HAL errors; these
tests do not prove that a physical card mounts or accepts writes.
CSV rows may enter the fixed-size queue after its initialization while the
writer mounts the card. This retains early network rows without blocking
acquisition; a slow or absent card still causes explicit bounded backpressure.
An unavailable card must not prevent network telemetry. Automatic provisioning
can erase a card when the DAQ marker is absent; back up existing files first.

## 50 kg load cell

The MCP3564R scans CH0 (1000 kg) and CH1 (50 kg), each single-ended against
AGND. The schematic maps P7 through AMP1 to CH0 and P8 through AMP2 to CH1;
the onboard amplifiers already convert each bridge's differential signal to
a ground-referenced voltage. See [load-cell wiring and diagnostics](docs/loadcell-inputs.md).
The ADC's 32-bit tagged output identifies the channel; only data-ready readings
are queued. Both cells target independent sample-weighted 250 Hz raw telemetry
(`KG1000`, ID 118; `KG50`, ID 119). The configured 3700 Hz timer is a polling
ceiling shared by both channels, not the per-cell sample rate. SCAN settling
and SPI overhead reduce the acquired rate.

SD raw records identify CH1 as `kg50_raw` and retain its ADC code, historical
raw calibration value, calibrated kg, and acquisition timestamp. CH0 retains
the `mcp3564r_raw` name. Raw calibration values preserve the existing scaling;
they are not ADC-pin volts. Separate slow diagnostic rows `mcp3564r_voltage_v`
(CH0) and `mcp3564r_ch1_voltage_v` (CH1) use the nominal internal 2.4 V reference
and the configured ADC gain of 1.
The replay file records `kg50_network`, identical to the raw `KG50` payload.
Every file header includes the 50 kg polynomial coefficients, input shift, and
tare offset; calibration changes rotate files as for the original channel.

GroundStation has a separate **50kg** load-cell tab and calibration sensor.
With the cell unloaded, capture zero; then capture one or more known masses
in kg, fit the calibration, and save it. Confirm zero and a known mass again.
Until calibrated, the identity defaults are raw values, not measured kg.
GroundStation records `KG50` and `LOADCELL_50_WEIGHT_KG` separately, and test-fire
CSV exports include timestamped 50 kg raw/calibrated rows. This channel does
not change the existing 1000 kg fill-control input.

`DAQ_KG50_CALIBRATION` (ID 138) carries seven float32 values
`[c0,c1,c2,c3,c4,x0,tare]`, evaluated as
`c0+c1*x+c2*x^2+c3*x^3+c4*x^4-tare`, where `x=raw-x0`.
GroundStation publishes this managed variable on save and serves it after
reconnect. DAQ persists it alongside the original calibration, migrating older
four-float stored records without changing their coefficients. The existing
ID 137 payload remains four floats. Actual coefficients require unloaded and
known-weight measurements; they cannot be inferred from the 50 kg rating.

ADC format and scan configuration follow the
[Microchip MCP3561/2/4R datasheet](https://www.microchip.com/content/dam/mchp/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf),
sections 5.6, 5.15, and 8.8.

## Temperature compensation

ADC temperature telemetry and per-cell thermal offset calibration are described
in [Temperature compensation](docs/temperature-compensation.md). New live
calibration captures retain ADC temperature; learn drift using unloaded captures
at different stabilized temperatures, then capture zero/mass and save.

## Additional analog inputs

All eight SAR inputs, six auxiliary MCP inputs, current sensors and power
monitors are supported at a default 10 Hz. See [the connector and telemetry
map](docs/analog-inputs.md) for pinouts, nominal units, GroundStation testing
configuration, and the additional load on acquisition/network/SD.

## Hardware watchdog

[Board watchdog configuration and validation](docs/watchdog.md). Build with
`./build.py build --release --watchdog`.
Watchdogs are enabled by default and require the matching bootloader.
Use `--no-watchdog` only for a diagnostic build.

## SEDSnet build configuration

Use `python3 build.py build --release --sedsnet-ref dev` to build the current
SEDSnet `dev` commit. The same option is accepted by `test` and `flash`.
Normal builds continue to select `main`. Each branch has its own source cache;
when the network is unavailable the last usable on-disk source is retained.
The selected revision is printed during configure. An explicit CMake source
override remains local and is never fetched or reset.

### Default TLSF and compact storage

```sh
python3 build.py build --release
```

This selects the latest compatible SEDSnet `main` commit and initializes a 8192-byte,
64-handle arena before the router starts. It uses the existing allocator
pool and leaves ThreadX scheduling unchanged. Compression stays disabled.
Use `--packet-store heap` to disable the arena, including in a previously enabled build cache.
Startup fails cleanly if the reservation does not fit; router retries reuse
the arena. TLSF and compact stores are the defaults; physical hardware qualification remains separate.

`--allocator tlsf` selects the board-owned TLSF allocator for SEDSnet.
ThreadX scheduling and thread stacks retain their existing ownership. The defaults are `tlsf` and `compact`. Use `--allocator threadx --packet-store heap` for an explicit fallback.
TLSF coalesces adjacent free blocks; packet-arena compaction is a separate option.

### HVAIN1 chamber pressure and analog power

The default chamber sensor is the OMEGA PX309-1KG5V: 0–1000 psig,
0–5 V output, 9–30 V excitation. The checked-in schematic sheet 9 traces
P4 / HVIN HVAIN1 through R27/R31 (10k/1k), U8B and R28 to U7 CH6;
HVAIN2 is CH7. P4 pin 6 supplies +12VA, pin 1 supplies +3V3A, and
pins 2/5 are ground; the PX309 excitation must not use the 3.3 V supply.
Firmware selects CH6 by default, reverses the 11:1 divider,
and converts connector volts to psi at 200 psi/V. Out-of-range measurements
are preserved, not clamped. The existing combined KG1000/pressure/connector-V
network slot shares the 40 Hz KG1000 schedule; the aggregate live cap stays 100 Hz.
The raw SD stream preserves each captured sample and both timestamps.

`DAQ_CHAMBER_ADC_CHANNEL=-1` explicitly disables the chamber channel and
restores the legacy OSR1024 / read3700 profile. The assigned-channel default uses
OSR256 / read5556 without changing the 16 MHz clock, unity gain, auto-zero settings
or load-cell calibration. SCAN settles each channel separately and AZ_MUX
doubles conversion time; OSR512 with three channels would reduce fresh
load-cell density. OSR256 preserves density with the pressure channel, at the
cost of less oversampling/noise rejection than OSR1024. It does not promise the
same effective resolution or noise floor; evaluate physical load-cell noise
before interpreting small changes.

The driver owns a 128-entry fresh-conversion FIFO. Acquisition drains up to 16
records every 2 ms into copied SD batches. A full FIFO increments ADC overruns;
SD admission and storage errors have separate counters. Polling and firmware
service delays can still miss hardware conversions before FIFO admission.
The host SCAN/DRDY accounting test includes auto-zero, auxiliary/temperature
passes, a 600 ms SD-worker stall and a deliberate acquisition stall; it proves
software ownership/accounting under that model, not real-card throughput.

PC4/+12VA_EN drives U17 TPS7A4501 SHDN directly: HIGH enables the analog rail.
Startup already asserts HIGH once, waits 10 ms, and now reads the GPIO back
before touching the ADC. `g_mcp3564r_12v_enable_readback` is UINT32_MAX before
initialization, then0/1; a low readback stops initialization with status 4. This
is logic-level readback, not a voltage measurement. The schematic net is named
+12VA, but R61/R65 (9.1k/820) imply approximately 14.64 V nominal using the
regulator's 1.21 V reference; verify actual excitation on hardware. Firmware does
not change the regulator or cycle other rails.

Sensor and conversion references: [OMEGA PX309](https://assets.omega.com/pdf/test-and-measurement-equipment/pressure/pressure-transducers/PX309-100mv.pdf),
[Microchip MCP3564R](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf),
[TI TPS7A45](https://www.ti.com/lit/ds/symlink/tps7a45.pdf).
