# DAQ Board firmware

The DAQ Board targets the STM32U585 and samples fill-system instrumentation for
publication over the SEDSNet CAN-FD network. The board uses SEDSNet v4.0.18 and
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
