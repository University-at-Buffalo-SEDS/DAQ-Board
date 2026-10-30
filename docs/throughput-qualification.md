# Acquisition, recording, and network rate qualification

The requested operating point is every ADC sample recorded to SD. Current
firmware caps each fast loadcell network stream at 250 reports per second;
actual cadence depends on acquisition and the selected input. The proposed raw
stream is 12,000 samples per second combined. These are separate measurements;
none can be established by changing a rate constant or counting queue accepts.

## Current transport and simulated delivery (2026-10-21)

Deployment uses the Pi's main Ethernet interface for discovery and framed TCP
with the single gateway-side Pico-Fi, then 1 Mbaud UART into gateway. There is no
GroundStation-side Pico or I²C hop. I²C observations below are historical.

The corrected P6 differential ADC model and all-current seven-board firmware
pass strict 120-second and 600-second delivery checks: zero gateway CAN-ring
drops, at least 99.9565% of actual publications preserved in every measured
window, and loadcell gaps no longer than 30 ms. DAQ publication and SD error/drop
counters remain zero. A separate GroundStation restart check still gates release.
This verifies simulated network preservation, not physical 12 kHz SD throughput.

## Observations on 2026-09-22

- The telemetry-batched DAQ image recorded approximately 920 raw samples/s
  combined in a five-minute observation with no reported SD queue drops, write
  errors, or ADC software queue overruns. See `sd-log-sessions.md` for scope and
  the earlier failed runs. This is not 12 kHz qualification.
- Live DAQ publication counters advanced approximately 182–185 times/s per
  channel. CAN accepted approximately 378 frames/s, without bus-off or HAL
  transmit failures. Acceptance does not establish remote delivery.
- During the reported outage, the DAQ topology contained only DAQ and AB.
  Restarting the gateway restored board visibility, as confirmed by the user.
  The original cause of that outage has not been established.
- The connected gateway exactly matched the `gateway-board` checkout's binary,
  rather than `gateway-board26`'s binary. Both sources used 115,200 baud for the
  Pico UART. The restarted gateway completed approximately 89 serial frames/s
  in an observed interval, with zero UART transmit failures and CAN ring drops.
- A debugger checksum operation initially overwrote live gateway RAM because
  OpenOCD's STM32G4 target defaults to `-work-area-backup 0`. Those subsequent
  UART failures and corrupted HAL clock values were diagnostic side effects,
  not evidence for the original outage. The debugger was changed to
  `-work-area-backup 1`, and a reset restored normal clock values. Prefer direct
  flash reads for matching a running image without executing target algorithms.

## Bridge rate configuration

The gateway now accepts the CMake cache setting
`GATEWAY_PICO_UART_BAUD_RATE`; Pico-Fi accepts the build environment setting
`PICO_FI_UART_BAUD_RATE`. Both default to 115200. A paired test build uses
1000000 for both. The Pico's persisted network/role configuration is independent
of this compiled UART rate and must be retained. Update the two peers together;
an unmatched rate interrupts the bridge. Successful builds do not qualify the
physical link or its framing/recovery behavior.

The paired 1 Mbaud gateway/client-Pico images were subsequently installed.
Gateway application and boot metadata passed flash verification; its bootloader
and persistent settings were retained. The Pico UF2 ended at `0x10025700`,
below the saved configuration sector at `0x101ff000`. Pico host framing and
transport-soak tests passed (two test wrappers). Both firmware builds passed.
Gateway live `huart2.Init.BaudRate` read back as 1000000. Between uptime 12.456
and 22.572 seconds, UART transmit completions advanced from 1504 to 2759
(124.1 frames/s) and CAN receive frames from 1887 to 3464. UART transmit
failures, UART ring drops, CAN ring drops, and panic counts remained zero;
network-ready/discovery/time-sync flags were set. This short check demonstrates
operation at the current offered load, not capacity at 1,000 reports/s or
long-duration outage recovery. The DAQ timing-diagnostic build is prepared but
has not yet been installed.

GroundStation subsequently reported 46.7 RX messages/s and 1.62 KiB/s on
`umbilical_comms` (35.6 bytes/message), with 17.5 TX messages/s. A later gateway
check over 10.123 seconds measured 1555 CAN receives and 1261 UART completions,
with no recorded software ring drops, UART failures, or allocation failures.
The counters describe different framing layers and are not by themselves an
end-to-end packet-loss ratio.

Pico-Fi code review found that the UART bridge races multi-step TCP and UART
frame readers in `select`, cancelling the losing future after it may already
have consumed part of a frame. The frame-reading progress was local to those
futures. A new persistent `FrameReader` retains header and payload progress
across cancellation; its caller retains the payload buffer. Tests cancel at
every byte boundary, alternate both directions, and cover empty, oversized,
truncated, and resynchronized frames. The actual reader is included in the
Pico host test crate. All three new tests and the two existing bridge test
wrappers pass, and the 1 Mbaud release image builds. The gateway-side Pico
was subsequently updated. The I2C and SPI sessions share the corrected TCP
reader. The I2C-side Pico was subsequently flashed through the Pi at
192.168.3.7 using the I2C server build (`pico-fi-server.json`). The transferred
UF2 SHA-256 was f9d6dcb66f533a2a61df3200b620e2d11ff741f8b36973e6efc21d13d7c37505.
Its 599 blocks end at 0x10025700, below the saved configuration sector. The
USB copy completed and the RP2 boot drive disappeared. End-to-end I2C receipt
and throughput after this update still require verification. This bug is a plausible loss mechanism,
not yet a demonstrated explanation for all of the observed rate shortfall.

## Required evidence

1. Count unique conversions, including channel attribution and acquisition
   timestamps. A software queue overrun counter cannot detect an ADC result
   overwritten before the firmware reads it. The current timer polls ADC status;
   every-conversion capture needs separate verification.
2. Compare acquired samples with records recovered from the SD file, including
   initial and final partial buffers. Check sustained operation, restart,
   filesystem flushes, and the recording start/stop transitions.
3. Measure 500 reports/s per channel at DAQ publication, router transmission,
   gateway ingress/egress, and GroundStation receipt over the same interval.
   Separate samples, transport frames, and display refreshes. Do not substitute
   repeated stale values or catch-up bursts for fresh periodic reports.
4. Exercise discovery and reconnection under that load. Record queue drops,
   transmission failures, memory headroom, and latency as well as average rate.

At 40 bytes per binary sample, 12,000 samples/s require 480,000 bytes/s before
filesystem overhead. Current SD output is CSV and has a different, larger
bandwidth requirement. The pressure-enabled default ADC OSR is 256 (legacy 1024 when disabled);
the 12 kHz request is not the
active conversion/read configuration. Acquisition settings, storage format,
buffer reserves, and end-to-end network capacity all remain part of the work.

## Follow-up fixes and remaining checks

The gateway's polling CAN path showed a latched hardware FIFO loss flag even
when its software drop counter was zero. The receive path now uses interrupts
and counts hardware loss notifications. Its initial live test exposed a startup
regression: HAL_FDCAN_Stop returns an error when the peripheral is already READY.
Initialization now stops only a BUSY peripheral; host tests cover READY startup,
stop failure, notification failure, and reception during startup. The corrected
application and boot metadata passed flash verification. Live CAN reception and
network-ready status returned, with zero initialization errors. Startup counters included 233 software ring drops and one hardware loss
notification. Neither counter increased in the subsequent 30.557-second check,
which measured 172.3 CAN receives/s and 137.7 UART completions/s, with zero UART
errors or initialization failures. This does not establish loss-free delivery
from startup or capacity at the requested rate.

The calibration frontend freshness check now uses the client's receipt timestamp
rather than the source timestamp. This prevents a source clock slightly ahead
of the client from blanking a newly received temperature. Three regression tests
pass, including clock skew, the two-second expiry, and invalid values. The change
has not yet been deployed and visually verified in the user's GroundStation.

The user reported 46.0 RX messages/s after the I2C-side update (1.69 KiB/s),
with 21.0 TX messages/s (1.34 KiB/s). Pi bus 1 maps to RP1 i2c@74000; its
live device-tree clock-frequency is 100000 Hz. The mailbox carries 14 bytes
of payload per 32-byte slot. Allowing 9 clocks per address/data byte gives
an ideal aggregate payload ceiling of 100000/(33*9)*14 = 4714 bytes/s,
before transaction gaps, padding, idle reads, and higher-level framing.
This is insufficient for 1000 reports/s at the observed message sizes.
A 400 kHz configuration comparison is prepared but not installed; remote
sudo requires a password. The gateway snapshot at uptime 1787.185 s had
106780 CAN software ring drops, one hardware overflow notification and ten
UART RX ring drops. A later debug memory read failed, so this run provides
no reliable current rate delta.

## Matched 400 kHz observation

The Pi's live bus-1 clock-frequency after reboot is 400000 Hz.
`build/gateway-gs-400khz.json` contains a matched observation with gateway
counters over 30.048 device seconds and GroundStation router totals over
30.265 seconds. Gateway firmware was verified by direct reads without halting;
no debug read errors occurred. CAN ingress was 178.21 frames/s, with 168 new
software ring drops (5.59/s) and zero new hardware overflow events. UART TX
completed 138.45 frames/s with zero TX failures or TX queue drops. UART RX
ring drops increased by 57. GroundStation umbilical RX was 121.59 messages/s
and 4091 bytes/s, TX 35.72 messages/s and 2532 bytes/s. All listed boards
remained online at the final snapshot.

The live websocket delivered 1335 KG50 rows and 1338 KG1000 rows in the
30-second collection window (approximately 44.5 and 44.6/s), plus 276 ADC
temperature rows (9.2/s). Initial websocket snapshots were discarded. The
row timestamp spans were about 30.25 seconds due to batching at the window
boundaries, so rates are approximate. Derived weight/fill rows were excluded
from the per-channel counts. The REST history endpoint is bucketed at 20 ms
and was deliberately not used as a raw receipt counter. Live websocket counts
measure delivery to that client; frame counters at different transport layers
cannot be directly subtracted to infer end-to-end packet loss. These results
confirm remaining gateway receive loss and rates below the 500/s per-channel
target; they do not yet identify every source of loss or rate limitation.
Pending user additions (2026-10-21)
- Warning confirmed on RRLS, not main GS: synchronize explicitly paired mission calibration and settings, preserving fresh data interlocks; separate revisions/channel merges and no hardware/auth secret sync.
- Compact PT acceptance limit editor and distinguish About from Save; Tools also exposes existing clear-all-graphs/no-history-reseed action. UI code updated; native 108 passed/7 pre-existing ignored; browser validation pending.
- Valve-board high-rate PT to DAQ SD: dedicated DAQ-targeted/batched pressure samples, source timestamps/raw+converted values; keep existing GS PT rate unchanged; gateway must not forward this stream. Adds shared CAN load, needs simulator delivery/ACK/gap and recovered SD count tests before enabled.
- FC SD must retain raw sensor codes alongside converted/calibrated values. Trace acquisition before conversion; paired fields/timestamps; bounded storage, drop counters, throughput and flash-fit tests required.
- C14 failed amplifier boot with 8 analog publish refusals: first 24475/24576 logical bytes at 2124ms, TX22550/storage48936, no TLSF failures. Enlarging DAQ cap 16 to24KiB merely delays saturation; do not release on this result or continue increasing blindly. Investigate queue dispatch starvation (control-RX overrun can disallow bulk TX in combined queue processing), routing/startup backlog, then requalify unchanged loss/latency gates. C13 P6 120/600s passed; default AMP still fails startup.

Pressure logging acceptance update: user selected **250 samples/second** from
ValveBoard to DAQ SD, with GroundStation pressure publication unchanged at 50 Hz.
ADC3/TIM3 already acquires 500 Hz into a 16-entry DMA ring. Capture every second
conversion through half/full completion events (four samples per 16 ms half),
not by polling the power-sensor task or repeating its latest value. Keep ISR work
bounded to integer capture/queueing; convert and transmit in a worker. Use a
DAQ-only VALVE_PRESSURE_LOG endpoint/type, batched source timestamps/raw codes/pressure, and verify
no copies reach the Pico/GS link. Count ADC/ring, CAN, SD and sequence gaps;
validate the recovered log against produced samples and command latency under
this additional shared CAN traffic. Simulator DMA trigger/callback fidelity is
part of this feature, not an optional test shortcut.


### Current250Hz pressure cohort

C16 isolated fair TX plus maintenance-free DAQ queue draining passed amplifier
startup and two GS service restarts at the original16KiB logical queue cap.
C18 rebuilt every consumer with that SDK and added the pressure/FC logging and
accurate ADC DMA model. It failed startup with two auxiliary refusals at1424ms:
16163/16384 shared bytes,9840 TX bytes, two CAN submission refusals, and no
allocator failures. The counters did not grow after startup and CAN queues
subsequently drained. This is retained as failed evidence.

The next controlled run raises only the DAQ logical allowance to24KiB while
retaining the dispatch fixes, all source rates, allocator reserves and loss
thresholds. C14's earlier queue-only failure remains evidence that increasing
capacity alone did not repair starvation. Current full-source startup,250Hz SD
record recovery and sustained network qualification remain pending.


The pressure recorder uses its own `VALVE_PRESSURE_LOG` endpoint rather than
claiming the general `SD_CARD` endpoint. Only the new batch type targets this
endpoint. The dedicated sink prevents enabling pressure recording from adding
local delivery for unrelated SD-card data; dynamic routing still learns the
DAQ owner, and GroundStation's ordinary pressure report rate is unchanged.
This scope correction still requires combined CAN/SD delivery qualification.

### ADC model correction and current qualification

C19 passed amplifier startup with the repaired dispatch path and24KiB DAQ
allowance, then failed the pressure/P6 short gate. No long run was accepted.
C20 read-only peripheral capture found ADC3 CR=0x10000004: ADSTART remained
set but the model incorrectly cleared ADEN. TIM3 TRGO was correctly UPDATE,
and pressure initialization/read failure counters were zero. The apparent
TRGO_RESET initializer was ruled out by executing its full USER section.

The simulator now preserves ADC read-set bits across HAL command writes;
its old-model negative control fails the exact enable-then-start sequence.
The repaired actual Renode test passes500Hz timer gating, half/full circular
DMA and explicit stop. Physical PT firmware was not changed for this bug.
C21 will qualify the dedicated VALVE_PRESSURE_LOG endpoint, gateway bounded
pre-admission reclamation, source preservation and recovered SD records together.
Passing the peripheral model test alone does not qualify250Hz end-to-end logging.

C21 now captures/transports pressure correctly, but its amplifier startup fails:
19 auxiliary report refusals at24458/24576 logical bytes. Pressure capture and
DAQ enqueue drop counters remain zero. Pressure RX was doing four CSV formats
and taking SD-pool locks; it now copies one fixed batch/session into a16-message
ThreadX queue. The SD writer formats at most four pressure rows per turn, retains
an unsuccessful row for retry, and separately tracks written/media-flushed rows.
The actual handoff/retry regression passes under ASan/UBSan;190 DAQ tests pass.
The macOS production factory fits935248/1900032 flash and768264/786432 RAM.
Each board needs its own prepared SDK tree: sharing one mutable SDK build tree
between board-specific CRC32 implementations contaminates the static archive.

C22 retains this improvement but still fails startup: five gateway CAN ring drops,
eight DAQ auxiliary refusals confined to startup, and four valve pressure publish
refusals. DAQ pressure accepted/written counts match at every observation, with
zero pressure SD drops and progressing media flushes. These are failed network
qualification results, not a reason to relax the counters or start the long soak.
C23 reuses the exact C22 firmware with extra first-drop-phase/admission probes to
identify the remaining startup contention. No physical firmware has been flashed.

### Current pressure-delivery qualification (C24–C27)

C24's first descriptor-vector prototype failed an existing 6600-byte free / 4096-byte
reserve assembly regression and was rejected. The corrected isolated C25 prototype
uses bounded descriptor storage plus 256-byte assembly scratch and a 16-descriptor
tail, releasing fragment allocations before output growth. Heap and compact native
matrices and strict Clippy passed without relaxing the original headroom guards.
Host median profiling of 500 transfers showed about 11% fewer allocations, 8% fewer
allocated bytes and 8% lower extra live memory; this is not an MCU speed guarantee.

C25 amplifier startup passed. Its P6/pressure short test failed: loadcells reached
GroundStation at 100% of actual publications with 20 ms maximum gaps, gateway CAN
ring drops stayed zero, and RF receipt latency was 50–110 ms, but pressure had four
source admission losses plus missing in-transit batches. Read-only inspection of
its retained DAQ SD image independently detects a pressure sequence/time gap.
It does not qualify for a long run or release.

C26 makes only the dedicated pressure stream Unordered reliable, with a bounded
200 ms retry of refused source enqueues. Accepted packets are never re-enqueued
with a new identity by the application. The ordinary GroundStation PT rate remains
unchanged. Both changed firmware builds passed; amplifier startup failed continued
DAQ auxiliary refusals (41 by its last probe). Gateway ring drops remained zero.
No long run was started.

An executable 44-byte reliable-batch regression passes lost-data and lost-receipt
recovery without duplicate delivery for heap and compact storage. A further busy-SD
handler negative control exposed an SDK defect: its failure was recorded but then
ignored when generating a final receipt. The isolated candidate now propagates
Packet/Packed local-handler errors before final receipts, preserves forwarding to
other destinations, and records no successful local delivery for a refusal. The
busy-handler regression covers Packet/Packed ingress and Packet/Packed handlers.
The DAQ callback now reports success only after an atomic whole-batch SD handoff.
C27 rebuilds all seven nodes with this candidate and retains first/latest DAQ
publication-refusal snapshots in the existing simulator-only diagnostic buffer.
C27 results and recovered-card validation are pending; no release is qualified.

C27 all-seven simulator factories pass, but amplifier startup again fails auxiliary
publication refusals (35 at the last probe). Latest snapshots confirm sustained
TX accounting of23165 bytes / shared24142 against24576, with RX0 and no allocator
failure. C28 is a DAQ-only diagnostic comparison: it independently sums live queue
entries, counts/types/delays and retains these read-only values in the existing
simulator buffer. It changes no capacity, rate or scheduling interval. Its result
is pending. The production macOS factories for the C27 DAQ/valve sources pass:
DAQ932312/1900032 ROM bytes, Valve463536/474624 FLASH bytes. Full191 DAQ tests,
139 valve tests, SDK heap/compact native matrices and strict Clippy pass. These
unit/build results do not override the failed startup gate.

### C28–C32 receive and publication investigation

C28 proves the retained DAQ TX backlog is real:109–113 live loadcell entries,
22345–23165 independently summed bytes, matching the queue ledger. The retained
items are priority16 data, not schema, ACKs or end-to-end retry records. C29's
bounded backlog-aware worker wait passes192 DAQ tests and the macOS factory,
but the combined startup still fails three gateway CAN-ring drops. No long
qualification follows a failed startup.

C30 reduces small-publication copying and repeated source route selection in an
isolated SDK candidate. A five-round host benchmark of10000 C-API publications
shows median CPU time about14% lower, allocations53% lower and allocated bytes43%
lower; this does not establish the same MCU speedup. All seven simulator factories
pass. Startup fails26 gateway ring drops, all before the first observation and
unchanged afterward. DAQ loadcell/50kg network failure counters remain zero in
all observed samples; an earlier auxiliary-refusal snapshot is still retained.
The first gateway drop is at tick1014 in CAN processing. Individual CAN callbacks
reach4ms and queued processing reaches7ms despite smaller requested budgets.
Existing source rates, queue capacities, clock speeds and memory reserves remain.

Further exact regressions expose a clock-forwarding bug: planning and dispatch
both spent the slow-link allowance. The candidate makes quota checks read-only,
commits only after successful handoff, and makes reachability checks non-mutating.
Router/relay, queued/immediate forwarding and busy-handoff retry tests pass.
Incomplete chunk assemblies now expire from accepted progress only; rejected,
invalid and duplicate fragments do not extend their lifetime. The complete native
heap/compact suites (511/517 library tests plus integration/C-system tests) and
strict Clippy pass. C31 and C32 simulator FC factory links exceed FLASH by124bytes;
these remain failed builds. No current-source long soak or release is qualified.

### C103 qualified return-path repair (2026-10-23)

The final SDK103 cohort passes the original amplifier startup, 120-second
pressure/source/SD gates, GroundStation restart regression, and 600-second
simulated traffic soak. The long run took 6088 wall-clock seconds. Network and
source-preservation checks and recovered DAQ pressure/raw and FC SD inspections
all return zero without relaxing rates, retries, budgets or acceptance limits.
The receipt fix keeps a known ingress-side publisher's ACK on its return path
instead of applying the unknown-publisher sole-link fallback. Local CAN pressure
receipts no longer spill into unrelated Ethernet/radio segments.

The immutable SDK runtime archive is
`90a4a64bde7cf1e7abf651dfc9c5eea09b3f4b0ef8c0ed92c3155e7f09ee7caa`;
qualified source is pushed to SEDSnet main/dev at `24d095b`. Firmware continues
selecting main. Pressure logging retains reliable unordered delivery to its
DAQ-only endpoint; its GroundStation report rate is unchanged. Valve capture
uses independent sample-sequence and observed-HAL clocks, with drift, duplicate,
missed-half, wrap and UTC-age regressions.

All seven production factories compile on macOS. The remaining five factories
pass both fresh dependency resolution and the qualified dependency lock. DAQ's
193-test suite passes with two existing skips; Valve140, gateway143, RF165
(one existing skip) and actuator115 pass. An isolated test-checkout naming bug
was repaired without changing actuator runtime code. The isolated DAQ build
uses the already-cached FileX commit pinned by CMake; its first offline configure
attempt correctly failed when that cache was not supplied.

These results qualify the simulated network/logging path, not physical radio
latency, card throughput or current calibration. Public simulator/package
release and the remaining GroundStation acceptance work are separate gates.
