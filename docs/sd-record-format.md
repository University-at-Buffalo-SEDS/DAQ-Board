# SD record format

DAQ and FC CSV files use the same columns, in this order:

```
timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence
```

## Format block

Every DAQ file and every 3 GiB part starts with the column header, then the
SEDS-SD format block (`Core/Inc/sd_format.h`), then the `# channel=`,
`# calibration` and `# chamber_pressure` lines:

```
timestamp_ms,monotonic_ms,sensor,...,record_sequence
# seds-sd,format=daq,version=4,board=DAQ,firmware=<rev>,tick_hz=1000,schema=bc9e710a
# seds-sd-columns,u64:ms,u64:ms,label,vec,vec,vec,vec,clock,f32:degC,i32:code,u64:tick,u32:Hz,u64:seq
# seds-sd-records,kg1000_v:v=ovr:a=code:r=V:c=kg@1000000,...
# seds-sd-records,...
```

- `firmware` is the short git revision (12 hex digits, `-dirty` for a
  modified tree, `unknown` without git), set by CMake
  (`cmake/firmware_revision.cmake`). It is informational: readers never accept
  or refuse a file by it.
- `tick_hz` is the `uptime_frequency_hz` of every record without an `@hz`
  override.
- `seds-sd-columns` gives each of the 13 columns positionally as
  `type[:unit]`; `vec` columns hold the values the record entry describes.
- Each `seds-sd-records` entry is `<label>(:<col>=<units>)+[@<tick_hz>]`,
  with `v`=value, `a`=raw_adc_code, `r`=raw_value, `c`=calibrated_value.
  MCP3564R records carry `@1000000` (microsecond `uptime_ticks`); ADC1
  supply, VB pressure, scalar and DAC rows use the 1000 Hz default.
- `schema` is the first 8 hex digits of SHA-256 over the columns line and the
  records lines, each with its CRLF. The firmware revision and version are
  not hashed.

**Version rule.** Changing the columns line or any record entry (adding,
removing or renaming a label, changing its columns, units or tick rate) or
the row layout requires a new `version`, the new hash in `SD_FORMAT_SCHEMA`
and in the frozen table of `tests/test_sd_format_block.py`, and the version
registered in the GroundStation26 SD parser. The test recomputes the hash,
compares it with the compiled `schema=` and the frozen table, and checks
that every label the source writes is declared, so a layout change without a
bump fails. The ground station refuses an unknown version and a damaged
hash.

**Version 4** (from version 3):

- Every MCP3564R record (`kg1000_v`, `kg50_v`, `kg50_p6_v`,
  `chamber_pressure_v`, `excitation_v`, `mcp3564r_ch2_v`..`_ch7_v`) declares
  `v=ovr`: the `value` column, empty before, is the ADC over-range flag. It is
  empty in range, `1` above +full scale and `-1` below -full scale
  (|raw_adc_code| > 2^23 - 1, `Core/Inc/daq_over_range.h`). A flagged row
  keeps the true `raw_adc_code` and the linear (invalid) `raw_value`; its
  `calibrated_value` is `inf` or `-inf`. Nothing is lost, and no reader can
  take a flagged row for a measurement by reading `calibrated_value`.
- `vdda_v` (`a=code:r=V:c=V`, milliseconds) is written beside a supply
  reading whenever VDDA moved 5 mV or more since the last one, and at least
  every 100 ms (so a quiet rail adds no launch-window rows): `raw_adc_code` is the STM32 VREFINT conversion and `raw_value` =
  `calibrated_value` the VDDA (+3V3A) it gives (`Core/Inc/daq_vdda.h`).
  `input_voltage_v`/`input_current_a` pin volts are converted with that VDDA
  instead of an assumed 3.3 V.
- The `# calibration` line gains `mcp_vref_v`, the MCP3564R reference every
  MCP3564R volt in the file was converted with (absent: 2.4 V). Volts are
  `raw_adc_code x mcp_vref_v / 2^23` (`kg50_p6_v`: divided by 16). A new line
  is written whenever the reference or the calibration changes.

Files without the block are legacy DAQ files: legacy daq v1 (mapping profile
`daq-20261101`) and legacy daq v2 (profiles `daq-20261117` and
`daq-20261123`), recognised by their manifest firmware pins as before.

`timestamp_ms` is network UTC when available; otherwise it is boot uptime.
`time_source` identifies `network` or `local`. `uptime_ticks` is independent of
network UTC changes and extends the HAL millisecond counter across its 32-bit
wrap; `uptime_frequency_hz` is its nominal scale (not the CPU frequency or an
oscillator calibration). DAQ MCP3564R raw rows (load cells, chamber PT,
auxiliary channels) carry microseconds, `uptime_frequency_hz=1000000`: the
conversion midpoint on TIM2, which counts from the same clock as the HAL tick,
so `uptime_ticks / 1000` is `monotonic_ms`. Every other row (ADC1 supply
records, scalar rows, FC rows) carries milliseconds at 1000. Use
`uptime_ticks / uptime_frequency_hz` for intervals (for example impulse
integration); `timestamp_ms` keeps 1 ms resolution. `monotonic_ms` carries the
elapsed time in milliseconds. Uptime resets on reboot; identify separate
boots by their files rather than joining uptime values across boots.

`record_sequence` is a per-boot, per-stream counter. Raw DAQ and FC counters
wrap modulo 2^32; DAQ telemetry rows use a 64-bit counter. It is not an ADC
sample rate. Compare sequences within their own log stream. Raw DAQ records assign it at acquisition; other records assign it
while serializing. Loss before serialization must be checked with the queue,
acquisition and storage counters as well as sequence gaps.

DAQ raw rows populate ADC code and raw/calibrated values. Their
`adc_temperature_c`/`adc_temperature_code` columns are written when the die
temperature changes, when the writer switches between the session and launch
logs, and at least every 256 rows (a quarter second at 1 kHz); an empty pair
means "unchanged since the previous row that had one", so a file's first rows
may be empty until the next repeat (the
temperature also has its own 10 Hz `mcp3564r_temp_c` row).
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
| `excitation_v` | HVAIN2 (CH7) when `DAQ_EXCITATION_ADC_CHANNEL=7` (load-cell excitation wired to P4/4) | ADC-pin V | connector V |
| `input_voltage_v` | VMAIN through VMON, ADC1 (PB2) | VMON pin V | board input V |
| `input_current_a` | LM74202 IMON, ADC1 (PB1) | IMON pin V | board input A |

`input_voltage_v`/`input_current_a` share the load-cell record stream and
`record_sequence` (millisecond `uptime_ticks`). They are written every 100 ms, and every
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
mean of one bin of a running median of the in-range (unflagged) `kg1000_v`
(or of the selected `kg50_v`/`kg50_p6_v`) `raw_value`s. Over-range rows
(`value` = 1/-1) never enter the median or the mean; they only place their
bin in time, and a bin holding nothing but over-range rows is published as
+INFINITY or -INFINITY (the direction of its last row). Otherwise: KG1000 a 3-sample median and 20000 us bins,
KG50 a 5-sample median and 50000 us bins, bin = `uptime_ticks` (us) / bin
width. The median is taken in record order over the last N values of that
sensor (fewer at the start); the bin sums the medians in double precision and
rounds the quotient once to binary32 (`daq_median_add`, `daq_bin_us_add`), so a
reader repeating this on the raw records gets the network values bit for bit.
A bin closes when the first record of a later bin arrives; the value is dated
at the bin centre. The chamber value in `DAQ_LOADCELL_CHAMBER` is built the
same way from `chamber_pressure_v` (3-sample median, 20000 us bins). The
`# calibration` line states the filters:
`loadcell_report=median_bin,kg1000_median=3,kg1000_bin_us=20000,kg50_median=5,kg50_bin_us=50000,adc_uptime_hz=1000000`.
Files from earlier firmware state `loadcell_boxcar_ms` instead (a 4 ms boxcar
of the raw records dated at the last record, millisecond `uptime_ticks`).

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

