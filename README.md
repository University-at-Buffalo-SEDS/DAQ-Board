# DAQ Board firmware

The DAQ Board targets the STM32U585 and samples fill-system instrumentation for
publication over the SEDSNet CAN-FD network. The board uses SEDSNet v4.0.27 and
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

Firmware creates a new `DAQ_<network-time>_<sequence>.CSV` file on every boot.
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
TIM2 runs at 1 MHz (160 MHz input divided by 160), matching the ADC driver's
microsecond conversion delays; its prescaler is also recorded in CubeMX.
The created FAT volume is directly readable on macOS, Linux, and Windows.

The SD card is optional for board operation. If it is absent or cannot be
mounted, acquisition and SEDSNet still start, a rate-limited `WARNING` is sent
to GroundStation, and firmware retries card detection once per second. A card
inserted later begins a new timestamped log without rebooting the DAQ board.

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

For a requested 12 kHz read ceiling, select OSR 256 and drain every 5 ms.
This is not an exact 12 kHz conversion clock: with the physical 16 MHz MCO,
OSR 256 produces 15.625 ksps internally; reads are paced at rounded microsecond
intervals plus SPI/interrupt overhead. Only acquired readings are recorded.
Do not claim lossless capture of every ADC conversion with this timer-driven
driver. The compile-time checks reject settings exceeding conversion speed or
raw batch capacity. SD throughput and actual achieved rate still require testing.

A sample-weighted boxcar averages acquired raw samples before downsampling.
Individual raw ADC records remain unfiltered on SD, while `kg1000_network`
contains the identical calibrated filtered value sent over SEDSNet. Averaging
reduces broadband noise, but is not a sharp anti-alias filter and does not
guarantee rejection of arbitrary interference. Lower OSR trades noise for speed.
See Microchip MCP3561/2/4R datasheet DS20006391C, conversion-time table.
