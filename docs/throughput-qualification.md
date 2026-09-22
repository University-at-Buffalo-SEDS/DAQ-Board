# Acquisition, recording, and network rate qualification

The requested operating point is every ADC sample recorded to SD, with 500
load-cell reports per second on each channel (1,000 combined). The proposed raw
stream is 12,000 samples per second combined. These are separate measurements;
none can be established by changing a rate constant or counting queue accepts.

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
bandwidth requirement. Current ADC OSR is 1024; the 12 kHz request is not the
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
