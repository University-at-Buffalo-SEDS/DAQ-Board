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

First-use provisioning can delay recording substantially on large cards. The
64 GB bench card required roughly 30,000 FAT-sector writes, and the original
single-sector formatter took many minutes while acquisition/networking ran.
The provisioning adapter now combines up to eight contiguous sectors per
physical write. It copies FileX's reused buffer and flushes before other I/O
and format completion; normal log writes keep their synchronous behavior.
Initialization stage 6 means formatting, and `g_sd_format_sectors_written`
reports completed provisioning sectors. `g_sd_mount_status` and
`g_sd_format_status` preserve FileX results. Stage 5 / `g_sd_ready=1` means the
logger is writing. Initialized queues accept startup data before mounting, up
to their bounded capacities. A completed provisioning marker prevents another
format on subsequent boots. Resetting before provisioning finishes restarts it.

Acquisition and SD writing share priority 6 with 1 ms time slices, below the
priority-5 telemetry worker. Previously the priority-8 writer was starved when
acquisition overran its 2 ms period and slept for only one 20 us tick. On the
physical board, mounting succeeded but the writer saved only about 19 raw
records/s while its queue continuously dropped batches. With equal-priority
slices, it saved 9,354 raw records over 10.171 s (about 920 records/s combined),
with no new raw-batch drops and no write errors in that first short window.
Longer observation exposed intermittent raw and report queue overflows: the
writer sometimes took about 1,029 ms to service one iteration. Giving it CPU
time alone was insufficient.
The before/after captures are `build/temperature-sd-starvation.json` and
`build/temperature-sd-live.json`.

The queues now allocate variable-sized records from two fixed ThreadX byte
pools, retaining the previous 224,560-byte raw and 178,176-byte report memory
budgets. A short ADC batch reserves only its actual samples; a CSV row reserves
only its actual text plus a terminator. Each record still owns its calibration,
session, and acquisition timestamps. No general-purpose heap is used. Queue
pointers are capped at 1,024 per stream; pool exhaustion remains nonblocking
backpressure. Records are explicitly eight-byte aligned and release their
original pool allocation. The release image uses 739,696 of 786,432 bytes of RAM
(94.06%); further reserve increases must account for this budget.

Both queues can buffer while an already provisioned card mounts. Missing cards
and long first-use formatting can still exhaust the bounded reserves.
`g_sd_raw_slots_used/peak`, `g_sd_line_slots_used/peak`, and
`g_sd_max_service_ms` expose live queue pressure and maximum writer iteration
duration. The nominal `DAQ_SD_RAW_BUFFER_MS` sizes the raw pool using full-batch
equivalents; actual stall tolerance depends on batch sizes and operating rates.

FileX's `FX_FAT_MAP_SIZE` is now 4,096 bytes rather than the default 128. The
default groups approximately 16 FAT sectors under one dirty bit on this 64 GB
card, making a flush mirror untouched sectors as well as changed ones. The
larger map tracks individual sectors for FATs up to 32,768 sectors. Both FAT
copies and the one-second flush policy are preserved; no reformat is needed.
FileX and application code are rebuilt together using the same `fx_user.h`
because this setting changes `FX_MEDIA` layout. A host regression exercises
the fetched FileX mirroring routine, including boundary sectors and failed-write
retry. Map precision alone did not eliminate drops on the physical card; compact
queues are also needed to absorb the remaining stalls.

Raw and telemetry streams each have a separate 4 KiB write buffer. Previously,
telemetry rows reached FileX individually, generating many partial-sector
updates even though raw writes were batched. Both buffers now flush before the
regular media flush; telemetry also flushes before calibration/session rotation
and final closure. A failed pending-buffer write prevents closure/rotation and
retains the pending bytes. CSV columns, maximum row length, stream separation,
and acquisition-time metadata are unchanged.

**Earlier physical-board failure (2026-09-22), before compact pools:** simply
expanding the fixed-size queues still lost data.
One 184.351 s observation averaged 866 raw records/s but ended with 6,464 raw
batch drops and 9,278 report-row drops since boot; maximum writer iteration
duration was 5,897 ms (`build/temperature-sd-final.json`). Additional timing
diagnostics then isolated a 2,086 ms media-flush phase in a later run, with
individual driver writes reaching 223 ms. By uptime 169.917 s that run had
written 153,328 raw records, with 159 raw-batch drops and 415 report-row drops.
There were no reported write errors or ADC queue overruns, and temperature
publishing remained about 9.75 Hz. A clean first minute did not predict the
later overflow (`build/temperature-sd-diagnostics.json`). No calibration
rotation occurred during that run; the calibration generation stayed at 2.

`g_sd_max_read_ms`, `g_sd_max_write_ms`, `g_sd_max_raw_service_ms`,
`g_sd_max_telemetry_service_ms`, and `g_sd_max_flush_ms` record completed
operation durations, including time spent preempted or waiting. They do not
alone prove that the SD card is defective. Investigation continued with the
same card and identified the FAT map and wasted queue capacity above.
The one-second flush policy remains unchanged; increasing the flush interval
would trade recording pressure against more unflushed data on power loss.
The release build and 102 Python plus 3 GoogleTest cases pass, including
compact-pool alignment, bounds, allocation failure, payload integrity and
repeated release under AddressSanitizer/UndefinedBehaviorSanitizer. Host
coverage also checks telemetry batching, tail flush failure/retry, and blocked
rotation when old-file data cannot flush. Host checks do not exercise physical-card stalls. First-use format batching
has host coverage; the card had already finished provisioning before that
adapter was installed, so its new-format wall time has not been measured.

**Intermediate same-card validation, before telemetry batching:** the 308.473 s
measurement in `build/temperature-sd-compact.json` saved 280,949 additional raw
records, with zero raw-batch drops, report-row drops, write errors, or ADC
overruns since boot. Temperature publishing averaged 9.76 Hz. A 1,854 ms writer
iteration and 1,646 ms flush were absorbed without loss; peak queued/in-service
records reached 358 raw batches and 646 report rows, then drained. These are
bench results at the observed acquisition/report rates, not a guarantee at the
500 Hz per-channel report ceiling or under arbitrary card stalls/power loss.
Direct SD-sector readback verified headers, both raw channels, temperature, and
final samples in the closed raw and telemetry logs. However, a subsequent boot
still overflowed both queues during repeated longer stalls
(`build/temperature-sd-reboot.json`), prompting the telemetry batching fix above.

**Telemetry-batched same-card measurement:** `build/temperature-sd-batched.json`
covers uptime 26.435 through 335.028 seconds. Raw records written advanced from
23,906 to 307,744, with zero raw/report queue drops, write errors, or ADC queue
overruns. The maximum writer iteration was 2,448 ms and media flush 1,633 ms.
This validates approximately 920 raw samples/second combined only. It does not
qualify 12,000 samples/second, 500 network reports/second per channel, restart
behavior, or recording through a firing. The current disk format is CSV; the
40-byte in-memory record size is not its on-disk bandwidth requirement.

At 12,000 40-byte records/second, even binary payload alone is 480,000 bytes/s.
A 1.633-second writer stall would require 783,840 bytes of pending payload at
that rate, exceeding the current raw queue reserve and leaving essentially no
RAM for the firmware. Sustained bandwidth and bounded stall absorption must
both be demonstrated; increasing the configured sample rate alone is not a fix.

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
