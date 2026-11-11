# SD write path and throughput

## Bench fault (2026-11)

The live bench offered about 2410 raw records/s, but the writer accepted only
1703/s and rejected 706/s. Processing kept pace with acceptance at about
1660/s, so the raw pool stayed full: accepted minus processed held at about
7800 records. There were no SD write errors, and `sd_max_write_ms` was 398.

## Where the time went

The limit was SD command latency, not CPU.

### CPU per raw record

Instruction counts were taken for the production code under QEMU
`mps2-an505` (Cortex-M33), using the firmware's `-Os` and FPU flags. These are
instruction counts, not cycles.

| Step | M33 instructions |
|---|---|
| `sd_format_csv_raw_record` (130-byte row) | 3054 |
| Copy of the row into the log buffer (newlib-nano memcpy) | 541 |
| Bounce copy into the IDMA buffer (former path) | about 520 |
| Enqueue copy of a 40-byte record | under 50 |
| Software CRC | none; SDMMC computes CRC16 in hardware |

That totals about 4.1k instructions, or 26-38 µs at 160 MHz. At 1.7k
records/s this is 4-7% of the CPU.

### Card commands per 4 KiB batch (former writer)

Each 4 KiB batch ended at an arbitrary byte offset. FileX therefore did all of
the following for every batch:

- read the partial head sector;
- wrote that head sector;
- wrote at most 7 sectors directly;
- later wrote the dirty tail sector, out of order.

Commands averaged 4.1 sectors. Every command also paid:

- CMD12, then CMD13 polling;
- a `tx_thread_sleep(1)` per poll while the card was busy, so busy time was
  rounded up to whole 1 ms ticks.

Multi-block writes were capped at 8 sectors and always copied through the
bounce buffer.

SDMMC1 uses internal DMA on a 4-bit bus at 20 MHz (PLL1P 80 MHz / (2 x 2)).
The media is synced (`fx_media_flush`) once per second, on file open and close,
and on a power-loss notice.

## Change

### Sector-aligned staging

Each open log stages rows in a 16 KiB buffer, `_Alignas(32)`.
`sd_log_flush_sectors` works as follows:

- When the buffer is full, it writes up to the last sector boundary and keeps
  fewer than 512 bytes staged.
- If a sync or the header left the file offset mid-sector, it first completes
  that one sector on its own.
- After that, every write starts sector-aligned at the front of the buffer, so
  FileX passes whole sectors straight to the driver.

### Zero-copy IDMA

`sd_hal_write` hands 32-byte-aligned internal-SRAM sources (the log buffers and
the FileX cache) to IDMA in place. One command carries up to
`SD_STORAGE_IO_MAX_SECTORS` (64). Any other source still uses the 4 KiB bounce
buffer.

### Result

A steady-state data write is now one 31-32 sector command, against about 2.3
commands of 4.1 sectors plus a read per 4 KiB before.

## File format

Rows and headers are unchanged. Aligned writes never pad or reorder bytes; only
the write boundaries moved.

`tests/test_sd_throughput.py` compares the new output with the former
writer's output for the same records:

- the whole file for a raw-only run;
- each stream in order for a mixed run.

The interleaving of raw rows and loadcell rows follows writer scheduling,
exactly as before.

## Power loss

The loss window is unchanged: data since the last successful sync, nominally
1 s, can be lost. That covers:

- up to 16 KiB staged per open log (about 50 ms at the bench rate);
- data already on the card whose FAT and directory size update waits for the
  sync;
- records still queued in the RAM pools, which can be up to about 3 s when a
  card stall delays the sync.

A power-loss notice syncs immediately.

The partial last sector written by a sync is rewritten once by the next aligned
write. The former writer rewrote such a sector on every 4 KiB batch, about 50
times per second.

## RAM and flash

To fund the extra 24 KiB of staging, the raw pool went from 332 to 321 KiB and
the line pool from 62 to 49 KiB. Both still meet the existing three-second
target on the target build:

| Pool | Before | After | Required |
|---|---|---|---|
| Raw (records) | 7808 | 7552 | 7532, for 2500/s x 3 s |
| Line (rows) | 2240 | 1792 | 1532, for 500/s x 3 s |

Raw-pool stall tolerance:

- 3.1 s at 2410 records/s;
- 2.1 s at 3615 records/s.

Optional raw channels are still shed above a 500 ms backlog. The release
firmware is unchanged in total size:

- RAM: 785,136 of 786,432 B (1,296 B free);
- flash: 894,032 of 1,900,032 B (47%).

## Throughput model and test

`tests/sd_throughput/harness.c` runs the production `sd_card.c` and
`sd_storage_io.c` with the fetched FileX on the host, against a virtual-time
card:

- each write command costs `CMD_US`, plus a bus transfer at 10 bytes/µs, plus
  `PROG_FIXED_US`, plus a programming term at 10 bytes/µs;
- an injected 400 ms busy stall occurs every 4 s;
- the writer's CPU is charged at 40 µs per raw row and 60 µs per CSV row, with
  70% of the CPU available to it.

A 6 ms per-command latency reproduces the bench with the former writer. That
value is fitted, not measured. The model is not cycle-accurate.

Model results at 6 ms per command:

| | Former writer | This writer |
|---|---|---|
| Capacity | 1744 records/s | 7017 records/s |
| 2410/s with 400 ms stalls | 13,156 of 48,200 dropped | 0 dropped |

Zero drops at 1.5 x 2410 = 3615/s with 400 ms stalls hold from 2 to 16 ms per
command. Drops start at 17 ms per command. There, the backlog after a stall
passes the existing optional-channel shedding limit (500 ms of ADC-rate
records), and optional raw channels are shed. At 1.0 x the rate, zero drops
hold up to 25 ms.

The former writer already dropped at 2 ms per command at 3615/s, and at 4 ms
per command at 2410/s.

Run the test with:

```sh
python3 -m unittest tests.test_sd_throughput -v
```
