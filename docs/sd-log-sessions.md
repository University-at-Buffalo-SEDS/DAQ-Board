# SD recording sessions

`FillSdCard` is the schema alias for the existing `SD_CARD` endpoint (ID 100).
It describes the fill-system logger, not an SD card on Actuator. The wire name,
numeric endpoint ID, and routing remain compatible with older boards.

The writer opens files lazily when the first record arrives. Raw ADC records
go to `DAQ_*.CSV`; network/report-rate values go to a separate
`DAQ_TELEMETRY_*.CSV`. A calibration change rotates each stream when its first
record with the new coefficients arrives. Duplicate calibration updates or
network reconnects do not rotate files. Each file includes its regression
coefficients. Files are not rotated just because a write buffer fills.

GroundStation publishes the retained `DAQ_LOG_CLOCK` network variable (type 139,
16 bytes: little-endian u64 session ID followed by u64 close deadline in Unix
milliseconds). Only GroundStation writes it; DAQ subscribes read-only. A launch
starts `DAQ_LAUNCH_*.CSV` and `DAQ_LAUNCH_TELEMETRY_*.CSV`; sequence confirmation
and Pilot-open timing can correct the deadline without starting another session.
At T+120 seconds producers stop enqueueing log records, the writer drains the
queues, closes both files, and flushes the media. Network telemetry continues.
Resetting the launch clock, starting a new launch, or changing calibration after
completion allows logging again. Both GroundStation and DAQ need this update.

GroundStation retains the session and absolute deadline on disk. Reconnecting
or restarting it must not restart a two-minute countdown. DAQ ignores duplicate
and older launch IDs and already-expired launches received after boot. After
obtaining UTC it also keeps a monotonic deadline, so losing time sync does not
prevent closure. If no valid UTC has ever arrived, it cannot safely infer when
an absolute network deadline expires; it keeps recording until time is known.

FileX creation/modification timestamps use network UTC when available. Closing a
file explicitly sets its modification time to the close time. FAT timestamps
have two-second resolution and no timezone; desktop software may interpret UTC
as local time. Before UTC is known the filesystem date starts at 1980-01-01,
not the FileX 2017 default. CSV samples retain local monotonic timestamps before
network sync, with `time_source=local`. Existing card files are not retimestamped.

A low-voltage warning requests an immediate flush; it does not permanently
disable logging. Regular flushes occur once per second. This is **not a guarantee
of zero data loss on abrupt power removal**: acquisition queues, pending writes,
filesystem metadata, and the SD controller's internal buffers can still be lost.
Reliable shutdown requires adequate power hold-up and advance power-fail warning.
Do not remove the card while recording. A FAT32 file cannot exceed 4 GiB; this
implementation does not promise unlimited single-file recordings.

The SD worker drains up to four report-rate rows per raw batch, so the two
500 Hz channels are not restricted to one report write per acquisition cycle.
An established launch deadline uses local monotonic time without querying the
router for every log row.

Validation: `./build.py test` covers calendar conversion, buffer I/O, bounded
queues, calibration/session rotation, duplicate/replayed launch clocks and loss
of time sync at the close deadline. Physical SD power-cut recovery remains a
hardware qualification task.

Qualification on 2026-09-21: host tests pass, including 300,000 two-channel
report bursts, session replay, and closure after time-sync loss. The remote
instruction-level check boots and writes real FileX records, but does **not**
yet pass the zero-overrun/zero-drop thresholds at the current two-channel
500 Hz reporting setting. Do not interpret the host results as qualification
of sustained, lossless recording at that rate. The cached v0.4.12 simulator
image also predates tagged dual-channel ADC support; this check used the
repository's updated `SedsSpiSensors.cs` model as a read-only container mount.
