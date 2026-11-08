# SD file download over SEDSnet

GroundStation can list and copy SD files after a test fire or flight, so no card
needs to be pulled. The service is on P2P stream port 4511. The protocol is in
`Core/Inc/sd_file_wire.h`.

| Request | Bytes | Answer |
| --- | --- | --- |
| LIST | `21 dir index16` | one regular file: size, `live` flag, name |
| READ | `22 offset32 dir len name` | up to 512 bytes, in frames of at most 100 data bytes |
| CANCEL | `23` | drops any queued answer |

Folders: `0` `/LAUNCH`, `1` `/UTC`, `2` `/NO_UTC`. Every frame fits the
117-byte bound the OTA receiver already uses.

## Where it can run

- The flight state must be Idle, Landed or Recovery. The board refuses new streams in
  every other state. A live stream is reset on the first poll after the state
  changes.
- The SD writer reads only during a turn in which it had no raw, telemetry or
  pressure rows to write. It never reads while a launch file is open or its
  final flush is pending.
- Each turn does at most one sector-aligned read of 512 bytes, or a directory step of at most 32 entries. Both go into one
  static 512-byte buffer. The read handle is a separate `FX_OPEN_FOR_READ` owner.
  It never writes, truncates or renames a file. Directory listing borrows the
  FileX default directory and restores the root within the same turn.
- The telemetry thread sends one answer frame per poll. It sends only when the
  router has no other queued work, or after a 25 ms gap. Frames are at least 2 ms apart.
- The station requests one block at a time and also caps the average rate
  (`GS_SD_DOWNLOAD_MAX_BYTES_PER_SEC`, default 16 KiB/s).

A file that is still open for writing (the current session) is marked `live`.
Only data already flushed to the card is served, up to the size that was current when
the file was opened.

Static RAM cost is about 220 bytes (the request slot). The 512-byte block and
the read handle (812 bytes together) are taken from the SEDSnet heap only while
a station session exists, after the normal 4 KiB heap reserve check. They are
returned once there is no session, no request in progress and no open handle.
A bind failure is logged and does not stop telemetry, OTA or logging.

## Throughput

DAQ reaches the station over CAN, so a launch file of tens of MB takes minutes and a
multi-GB session file takes hours to days. Transfers resume at the byte they stopped at,
so a large file can be fetched over several sessions.
