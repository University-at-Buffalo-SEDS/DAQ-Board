# DAQ Board firmware

The DAQ Board targets the STM32U585 and samples fill-system instrumentation for
publication over the SEDSNet CAN-FD network. The board uses SEDSNet v4.0.20 and
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
```

The normal blank-board flash writes the complete factory image at `0x08000000`.
Use `./build.py flash --help` for alternate programmers.

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
