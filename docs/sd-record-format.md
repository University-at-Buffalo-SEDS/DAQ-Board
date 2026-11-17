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

## DAQ sensor names

DAQ sensor names are `<sensor>_<unit>`. No name contains `network`: the SD
card holds each measurement once, and the SEDSnet stream is derived from it.

ADC records (empty `value`; `raw_adc_code`, `raw_value`, `calibrated_value`):

| Sensor | Source | `raw_value` | `calibrated_value` |
| --- | --- | --- | --- |
| `kg1000_v` | MCP3564R CH0, P7 / AMP1 | ADC-pin V | on-board kg |
| `kg50_v` | MCP3564R CH1, P8 / AMP2 | ADC-pin V | on-board kg |
| `kg50_p6_v` | MCP3564R CH2-CH3 differential, P6 (gain 16) | input V | input V |
| `chamber_pressure_v` | chamber PT channel (CH6 or CH7) | connector V | psig |
| `mcp3564r_ch2_v` .. `mcp3564r_ch7_v` | other MCP3564R inputs | ADC-pin V | connector V |
| `input_voltage_v` | VMAIN through VMON, ADC1 (PB2) | VMON pin V | board input V |
| `input_current_a` | LM74202 IMON, ADC1 (PB1) | IMON pin V | board input A |

`input_voltage_v`/`input_current_a` share the load-cell record stream, clock
and `record_sequence`. They are written every 100 ms, and every
`DAQ_SUPPLY_LAUNCH_PERIOD_MS` (4 ms) from 0.5 s before the launch window
until it closes, so supply dips line up with load-cell samples.

Scalar rows (value in `value`), written every 100 ms: `adc1_aux_v`,
`analog_in_1_v` .. `analog_in_8_v`, `isense1_v`, `isense2_v`, `isense1_a`,
`isense2_a`, `vmon_v`, `imon_v`, `analog_out_1_v`, `analog_out_2_v`,
`kg1000_window_code`, `kg1000_window_v`, `mcp3564r_temp_c`,
`kg50_window_v`. The `kg*_window_*` rows are the mean of the conversions
drained in one acquisition cycle.
`valve_pressure_psi` carries VB OX-tank pressure (ADC code and psi).

The SEDSnet `KG1000`/`KG50` values are not stored separately. Each one is the
mean of one `loadcell_boxcar_ms` bin of `kg1000_v` (or of the selected
`kg50_v`/`kg50_p6_v`) records, where bin = `uptime_ticks` / bin width. The bin
is summed in double precision in record order and the quotient is rounded once
to binary32 (`daq_bin_average_add`), so a reader repeating this on the raw
records gets the network values bit for bit. A bin closes when the first record
of a later bin arrives, and the value carries the time of the bin's last record.
The `# calibration` line gives `loadcell_boxcar_ms`.

### Older files

| Old name | New name |
| --- | --- |
| `kg1000_network` | removed: rebuild from `kg1000_v` |
| `kg50_network` | removed: rebuild from `kg50_v` |
| `kg50_p6_network_volts` | removed: rebuild from `kg50_p6_v` |
| `mcp3564r_raw` | `kg1000_v` |
| `kg50_raw` | `kg50_v` |
| `mcp3564r_ch2_ch3_gain16_raw` | `kg50_p6_v` |
| `chamber_pressure_raw_volts_psi` | `chamber_pressure_v` |
| `mcp3564r_chN_raw` | `mcp3564r_chN_v` |
| `input_voltage_v`, `input_current_a` (scalar rows) | ADC records with the same names |
| `mcp3564r_code` | `kg1000_window_code` |
| `mcp3564r_voltage_v` | `kg1000_window_v` |
| `kg1000_raw_window_average` | `kg1000_window_input_v`, then removed: equal to `kg1000_window_v` |
| `mcp3564r_ch1_voltage_v` | `kg50_window_v` |

Files without `loadcell_input=adc_volts` on the `# calibration` line hold
KG1000/AMP2 KG50 values on the legacy raw scale (volts = raw * 16000/921).

