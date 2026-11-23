# SD file transfer over SEDSnet

GroundStation can list and copy SD files after a test fire or flight, so no card
needs to be pulled. The service is on P2P stream port 4511 and speaks SD
transfer protocol v2, the same wire as the FC. The authoritative description is
GroundStation26 `docs/backend/sd-transfer.md`;
the DAQ side is `Core/Inc/sd_file_wire.h`, `Core/Src/sd_file_stream.c` and the
writer turn in `Core/Src/sd_card.c`. The sender engine (`Core/Src/sd_xfer.c`)
and the SDX encoder (`Core/Src/sd_xfer_codec.c`) are shared verbatim with
FlightComputer26.

| Request | Bytes | Answer |
| --- | --- | --- |
| LIST | `21 dir index16` | one regular file: size, `live` flag, name |
| OPEN | `25 xfer dir flags offset32 length32 rate16 msg16 window block len name` | `A5` (19 bytes), then streamed DATA `A6` messages |
| ACK | `26 xfer una16 bitmap32 rate16 window` | none; opens the window, sets the rate, selects retransmissions |
| HASH | `27 dir len name` | `A7`: size and SHA-256 of the file as it is now |
| CANCEL | `23` | ends the transfer or hash; `A3` |

Folders: `0` `/LAUNCH`, `1` `/UTC`, `2` `/NO_UTC`. Names are plain leaf names
(letters, digits, `_ - .`, at most 63 bytes). v1 READ (`22`) is gone and
answers BAD_MESSAGE.

- The range is cut into independent blocks (the station chooses 1–64 KiB);
  each is sent as DATA messages of at most 1024 bytes (the station chooses
  the size), SDX-encoded when asked. Files on the card stay CSV. The last
  message of a block carries its raw length and CRC-32.
- The station acknowledges with a selective ACK. The board keeps at most
  `window` (≤ 32) messages unacknowledged, sends at most the station's
  `rate`, and regenerates only the blocks holding missing messages. It ends
  the transfer once the END block is acknowledged, after 15 s without an ACK,
  on CANCEL, a new OPEN or a stream reset.
- After the last byte the station asks HASH and compares the SHA-256 with its
  copy. The hash uses LaunchCore's software SHA-256 under private names
  (`Core/Src/sd_file_sha256.c`): the application's `launchcore_sha256_*` is
  the HASH peripheral, one global context that OTA checks and SEDSnet crypto
  re-initialise from the telemetry thread.

## Where it can run

- The flight state must be Idle, Landed or Recovery. The board refuses new
  streams in every other state. A live stream is reset on the first poll after
  the state changes; that also ends any transfer.
- Only the GroundStation (`GS`) may open the stream.
- The SD writer works for the service only during a turn in which it had no
  raw, telemetry or pressure rows to write. It never reads while a launch file
  is open or its final flush is pending; a request then answers BAD_STATE.
- Each such turn does one bounded step: a request's answer (a directory step
  of at most 32 entries, or opening the file), one DATA message (at most
  1 KiB of reads and the re-encoding of at most one block), or 8 sectors
  (4 KiB) of a HASH. The read handle is a separate `FX_OPEN_FOR_READ` owner.
  It never writes, truncates or renames a file. Directory listing borrows the
  FileX default directory and restores the root within the same turn.
- The telemetry thread sends answers first, then DATA at the station's rate.
  It sends only when the router has no other queued work, or after a 25 ms
  gap. Frames are at least 2 ms apart.

A file that is still open for writing (the current session) is marked `live`
in LIST, the OPEN answer, every DATA message and the HASH answer. Only data
already flushed to the card is served, up to the size that was current at
OPEN (or at HASH).

## Cost

- **RAM.** About 320 bytes static (the request slot, answer, hash cursor and
  counters). A
  6168-byte area is taken from the SEDSnet TLSF heap only while a station
  session exists, after the normal 4 KiB heap reserve check: the 512-byte
  block, the transfer state (328 B), the workspace (5040 B: the SDX encoder
  with 128 token contexts, a 1 KiB raw window and a 1 KiB message; or the
  SHA-256 context while hashing) and the FileX read handle (288 B). It is
  returned once there is no session, no request in progress and no open
  handle. A bind failure is logged and does not stop telemetry, OTA or
  logging.
- **CPU.** One 1 KiB SDX message encodes about 4–5 KB of CSV: about 0.82 M
  Cortex-M33 instructions (`-Os`), roughly 5–8 ms of the lowest-priority
  writer thread per message (8–12 % of the CPU at 16 KiB/s on the wire). A
  4 KiB HASH step is about 0.23 M instructions (1.4–2 ms) plus its 8 sector
  reads.
- **Network.** A 1 KiB message crosses CAN FD as about 12 SDK chunks; the
  gateway reassembles it within its 16 KiB SEDSnet budget (about 2 KB per
  message) and forwards it to the GS.

## Throughput

DAQ reaches the station over CAN, so the station sets the rate. SDX carries
DAQ rows at about a quarter of their CSV size, so a ~13 MB launch file needs
about 3 MB on the wire. Transfers resume from the station's verified prefix,
so a large file can be fetched over several sessions.
