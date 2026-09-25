# Communications throughput review — 2026-09-24

Static review of the current DAQ, gateway-board, pico-fi, and GroundStation26
working trees. No hardware access was used for this review. Historical live
measurements are evidence from earlier in the session, not new validation.

## Implementation prepared after review

The findings below describe the inspected baseline. Subsequent offline fixes:

- DAQ defaults to 250 reports/s per loadcell. Acquisition submits captured
  reports to a bounded nonblocking worker queue; SD raw acquisition is unchanged.
- Gateway UART uses DMA, chains queued transfers on UART completion, retains
  buffers through errors/abort, and counts rejected frames and exhausted retries.
- GroundStation yields between full I²C receive bursts instead of sleeping 1 ms;
  idle receive calls still sleep and each burst releases the shared TX/RX lock.
- Pico exposes aggregate `/queues` loss/high-water diagnostics, rejects oversize
  UART frames, and preserves a partially transmitted UART frame under pressure.
  Existing mailbox overflow remains explicitly lossy, not a lossless guarantee.
- Gateway and DAQ pin SEDSnet b3f0cdd. Gateway's `Pico1M` preset and Pico's
  `tools/build-paired-i2c.sh` retain matched 1 Mbaud build settings.

Host tests and firmware builds validate these changes offline. They do not
establish sustained hardware throughput or latency. Both 1 MHz I²C and 1 Mbaud
UART remain below the 5,000-message/s target at the observed packet size.

## Conclusion

The gateway/bridge path cannot provide 5,000 separate approximately 35-byte
SEDSnet packets per second with the current 1 Mbaud UART and 1 MHz I²C settings.
The DAQ also has processing and scheduling bottlenecks. Static analysis cannot
attribute the earlier approximately 60–70 Hz per loadcell exclusively to either
board. Increasing link speed alone will not establish the required throughput.

## Wire and scheduling budgets

Here N is the complete serialized packet carried inside the bridge frame, not
the four-byte sensor value. These are optimistic bounds, excluding extra
control traffic, retries, software execution, and electrical timing margins.

| Path | Cost for N = 35 bytes | Optimistic limit |
| --- | --- | --- |
| Gateway UART, 1 Mbaud, 8N1 | 10 × (N + 4) = 390 bits | 2,564 packets/s in one direction |
| Pico I²C v2 receive, 1 MHz | 9 × (N + 10) = 405 clock pulses | 2,469 packets/s with no competing transfers |
| GroundStation receive worker, burst 8 plus 1 ms sleep | 8 × 405 µs + 1 ms | 1,887 packets/s before other work |

I²C receives a four-byte header peek, then rereads the header with the payload.
Both reads include an address byte, giving N + 10 acknowledged bytes. The bus
is shared by both directions. Startup selection is additional, infrequent work.
The worker sleeps after every receive call, including when it drains a full
burst; that sleep is not restricted to idle periods.

At 5,000 packets/s, UART alone would need at least 1.95 Mbaud and the present
I²C framing would need at least 2.025 MHz, before any overhead or reverse traffic.
These are arithmetic requirements, not supported or validated operating rates.
The Pico build currently rejects UART rates above 1 Mbaud. Simply raising its
validation ceiling would not qualify the hardware or firmware at a higher rate.

## Concrete code findings

1. **Gateway UART blocks the router thread.**
   `gateway-board/Core/Src/telemetry_uart.c:495` calls `HAL_UART_Transmit`.
   The six-entry queue flushes one frame per normal `telemetry_uart_process`
   call. When full, enqueue synchronously flushes another frame. Consequently,
   the one-millisecond thread sleep is not a strict 1,000-message/s ceiling,
   but UART transmission occupies time otherwise available to CAN and routing.
   At 1 Mbaud, 1,500 frames/s of this size consumes 585 ms/s of blocking wire
   time, before parsing, forwarding, and reverse traffic.

2. **A gateway UART transmission error loses an already dequeued frame.**
   `telemetry_uart_reply_next_data_frame` pops before transmission. The write
   helper returns void and records failure, so that frame is not restored to
   the queue. Router enqueue success does not imply successful wire delivery.

3. **Pico queues deliberately discard old traffic under pressure.**
   `pico-fi/src/bridge/overwrite_queue.rs` bounds I²C to eight packets and
   8,192 bytes, evicting old entries when full. Eight small packets cover just
   4 ms at 2,000 packets/s. The UART egress ring also overwrites on pressure.
   The new v2 FIFO read fixes latest-only draining, but does not make these
   queues lossless. Larger queues alone would trade loss for more latency.

4. **GroundStation has an avoidable busy-path sleep.**
   `backend/src/telemetry_task/radio_io.rs` sleeps 1 ms after every receive
   call. `backend/src/comms.rs` defaults to an eight-packet I²C receive burst.
   The older 10 ms TX spacing belongs to different worker branches; main
   selects `legacy_single_worker=false` and `prioritize_rx=false`. It is not
   correct to attribute the active I²C path's rate to that older TX delay.

5. **DAQ publication can stall acquisition.**
   `Core/Src/daq_thread.c` samples, drains raw ADC data, publishes temperature
   and both loadcells, then services the ADC and sleeps. Publication enters
   shared router code. `Core/Src/telemetry_hooks.c` takes its mutex with
   `TX_WAIT_FOREVER`. Network processing and allocation can therefore delay
   the acquisition loop even though the target report interval is 2 ms.
   The analog worker is separate, but loadcell publication is not.

6. **CAN adds padding and fragmentation overhead.**
   Both boards use CAN FD without bit-rate switching. `can_bus_send_large`
   wraps traffic in a packed nine-byte fragment header and always sends
   64-byte CAN payloads, including a small single-fragment message. Thus a
   35-byte packet is not a 35-byte CAN frame on the wire. CAN arbitration,
   CRC, stuffing, and other nodes consume additional capacity. No bus-rate
   change should be deployed without a coordinated board configuration.

7. **Clean-build settings can undo the paired baud upgrade.**
   The inspected gateway build cache selects 1,000,000 baud, but gateway
   CMake and Pico build.rs both default to 115,200. Rebuilding without the
   explicit overrides can silently restore the old rate or mismatch the pair.
   Shared-library fixes are committed on SEDSnet dev as b3f0cdd; DAQ now pins
   that revision. Gateway still references main and needs the same dependency
   pin before its current local vendor changes are reproducible elsewhere.

## Recommended implementation order

1. Make gateway UART TX asynchronous with DMA and explicit buffer ownership.
   Release a queued frame only on successful completion; propagate failures
   and bound retries. Never let a DMA buffer be overwritten while in use.
2. Let GroundStation immediately service another full RX burst, with a bounded
   work budget and fairness for TX; sleep only when idle. Preserve separate
   packets and timestamps. Draining several packets per worker wake is not
   waiting to batch samples into one network packet.
3. Expose queue occupancy, high-water marks, drops, and enqueue/completion
   counters at each hop. Replace silent bridge eviction with an explicit,
   bounded overload policy. Backpressure must not stall ADC acquisition.
4. Decouple DAQ loadcell reporting from raw acquisition using a bounded handoff
   with explicit overflow accounting. Preserve every ADC sample separately
   for SD; do not equate a network report count with the ADC conversion rate.
5. For the >5,000-message/s target, qualify a faster gateway-to-Pico link and
   use USB or SPI at the GroundStation end, or establish a supported faster
   I²C mode across both devices. Keep I²C at 1 MHz only with a lower, measured
   traffic budget. Changing the Pi clock setting alone is insufficient.
6. Persist paired transport settings and dependency revisions in a named build
   configuration. Run host regressions and finite-buffer traffic simulations;
   reserve sustained throughput, electrical integrity, and worst-case latency
   acceptance for when hardware becomes available.

The earlier live run restored the actuator and all four extra DAQ analog
packet types. It did not establish 500 Hz per loadcell or >5,000 network
messages/s. Those targets remain unverified.
