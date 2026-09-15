# DAQ Board firmware

The DAQ Board targets the STM32U585 and samples fill-system instrumentation for
publication over the SEDSNet CAN-FD network. The board uses SEDSNet v4.0.29 and
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
Only `mcp3564r_raw` samples go into this file. Filtered `kg1000_network` rows,
other telemetry snapshots, and packet diagnostics go into separate
`DAQ_TELEMETRY_<network-time>_<sequence>.CSV` files. Each stream rotates
independently when its queued records change calibration; delayed telemetry
cannot mix into or rotate the raw ADC log.
The CSV contains the
network timestamp, local monotonic timestamp, sensor name, and value; the
`mcp3564r_raw` rows retain every drained raw sigma-delta ADC conversion while
the network receives a 50 Hz average. Every file begins with the four active
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
The networking worker polls at a one-millisecond interval even with DAQ's 50 kHz
RTOS tick, and scans its stack only every 250 ms. Acquisition accounts for work
time in its 20 ms period and exposes overruns as `g_daq_sample_overrun_count`.
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

- `DAQ_ADC_OSR`: hardware ADC oversampling filter (default 1024).
- `DAQ_ADC_READ_RATE_HZ`: requested timer-paced ADC reads (default 3700).
- `DAQ_ACQUISITION_PERIOD_MS`: raw queue drain cadence (default 20 ms).
- `DAQ_BROADCAST_RATE_HZ`: filtered network/SD replay cadence (default 50 Hz).

The report window preserves its phase across scheduler jitter. A low-voltage
SD flush notification no longer permanently switches acquisition to a 100 ms
sleep; telemetry retains its configured cadence while the processor is running.
`g_daq_loadcell_publish_ok_count` counts accepted router enqueues, not delivery.
For transport diagnosis, compare it with `g_daq_loadcell_wire_tx_count`,
`g_daq_loadcell_wire_fail_count`, and `g_daq_queue_service_error_count`.
Only GroundStation receipt confirms end-to-end delivery.

Locally generated `KG1000` has an explicit CAN route so loss of the discovered
GroundStation endpoint does not suppress its 50 Hz stream while other peers
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
OSR 256 produces 15.625 ksps internally; reads are paced at rounded microsecond
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
