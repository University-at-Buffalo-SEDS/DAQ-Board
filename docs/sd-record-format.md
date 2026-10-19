# SD record format

DAQ and FC CSV files use the same columns, in this order:

```
timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence
```

`timestamp_ms` is network UTC when available; otherwise it is boot uptime.
`time_source` identifies `network` or `local`. `uptime_ticks` is independent of
network UTC changes and extends the HAL millisecond counter across its 32-bit
wrap. `uptime_frequency_hz` is the nominal counter scale, currently 1000. It is
not the CPU frequency or an oscillator calibration. Its resolution is 1 ms;
multiple ADC samples can share a tick. `monotonic_ms` carries the same elapsed
time in milliseconds. Uptime resets on reboot; identify separate
boots by their files rather than joining uptime values across boots.

`record_sequence` is a per-boot, per-stream counter. Raw DAQ and FC counters
wrap modulo 2^32; DAQ telemetry rows use a 64-bit counter. It is not an ADC
sample rate. Compare sequences within their own log stream. Raw DAQ records assign it at acquisition; other records assign it
while serializing. Loss before serialization must be checked with the queue,
acquisition and storage counters as well as sequence gaps.

DAQ raw rows populate ADC code, raw/calibrated values and ADC temperature.
FC leaves those DAQ-specific columns empty and preserves its complete vector
in the quoted `value` field, separated by spaces. Text values quote commas,
quotes and newlines using CSV escaping; use a CSV parser rather than splitting
lines or commas. Floats retain nine significant decimal digits. DAQ files also
contain `# calibration` metadata rows, which readers should handle separately.

Writers retain a buffer when FileX rejects a write and retry from its original
file offset, including after a partial write. Buffer acceptance and successful
FileX writes do not establish power-loss durability. Media-flush counters mark
successful FileX flushes; actual card throughput, card-controller behavior and
power-loss recovery still require hardware tests. Acquisition is bounded and
cannot promise zero loss during an unbounded card stall; rejected records and
I/O errors are counted rather than silently treated as written.

DAQ and FC start a new file part before 3 GiB, leaving margin below FAT32's
4 GiB single-file limit. Rotation happens between complete records and retains
the acquisition calibration, boot uptime and sequence progression. Files are
not rotated on ordinary flushes. This avoids the single-file ceiling; enough
free space and sustained card performance are still required.
