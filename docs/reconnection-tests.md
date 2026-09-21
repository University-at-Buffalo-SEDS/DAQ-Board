# Reconnection qualification

Run ./build.py test --all --release --ultra-soak for the unit, short linked,
and 600-second seven-board network tests. The long test restarts only
GroundStation after sample four, then GroundStation and the avionics boards
after sample eight. The other boards and Pico-Fi pair remain running.

Each new GroundStation process must rediscover every board and rebuild traffic
attribution. Fresh Valve and Actuator commands require matching state responses
throughout the run; memory and latency bounds remain enforced. Topology-generation
unit tests catch malformed scenarios without Docker. Passing simulation does not
replace hardware validation.

Firmware continues to fetch SEDSNet main through CMake; the simulator image
and source fallback are pinned to v0.4.12. Topology-generation unit tests pass,
but the complete 600-second suite has not been rerun for this commit. Earlier
qualification results are not proof that the current firmware passes.

The cached v0.4.12 image may lack the tagged, dual-channel MCP3564R model needed
by current DAQ firmware. Use an updated simulator checkout with
`SEDS_FIRMWARE_SIM_SOURCE=/path/to/FirmwareSimulator` to build the matching
image. Current DAQ testing still reports overruns/drops at two-channel 500 Hz;
see DAQ-Board's `docs/sd-log-sessions.md` for that outstanding limitation.
