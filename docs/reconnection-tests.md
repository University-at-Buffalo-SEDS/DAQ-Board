# Reconnection qualification

Run `python3 build.py test --full --release` for the native tests, factory and
OTA builds, standalone memory and disconnected-CAN tests, and linked network
qualification. The linked stage runs a healthy seven-board scene first, then a
separate DAQ CAN outage/rejoin scene. `--ultra-soak` additionally runs the
600-second restart and network-fault scenario.

The simulator image and source fallback use the development branch and the
runner's declared interface version. A local simulator checkout can be selected
with `SEDS_FIRMWARE_SIM_SOURCE=/path/to/FirmwareSimulator`. Firmware and simulator
must support the same peripheral models and probe symbols; missing diagnostics
fail rather than silently skipping a check.

## Missing CAN acknowledgements

A board with no acknowledging CAN peer eventually fills its bounded transmit
queues. The DAQ scheduler may then receive `SEDS_IO` for a sensor publication;
it retains the next fresh value and does not retry that publication slot.
Acquisition and ordinary SD logging continue at their configured rates.

Only the explicit disconnected-CAN scenario permits these IO refusals. It bounds
submissions by the configured aggregate publication rate, checks the SDK's
24,576-byte shared budget and the transport's 48 pending packets/6,144 bytes,
and rejects every other publication error. Allocation failures, memory-admission
failures, panics, sampling overruns, invalid offers and SD errors/drops must stay
zero. The classifier and queue-budget observers exist only in simulator builds;
production does not run their diagnostic work.

## Rejoin and fresh data

The recovery scene detaches only DAQ's CAN controller after sample three and
reattaches it after sample five. It gives the transport two recovery observations
to drain accepted work. Every subsequent interval must have no new publication
refusals or CAN expiries, while acquisition, SD writes and CAN progress continue.

GroundStation must then accept at least 95% of each configured loadcell stream's
cadence in a complete ten-second observation window, with gaps no longer than
500 ms. This checks unique received packets, not producer counters. Fresh,
alternating Valve and Actuator commands must each return their matching board
status within the existing 2,500 ms limit throughout the recovery rounds.
The healthy linked scene retains its original zero-refusal and latency gates.
