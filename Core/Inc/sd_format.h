#ifndef SD_FORMAT_H
#define SD_FORMAT_H
/* SEDS-SD format block, written after the column header of every DAQ SD
 * file and file part (docs/sd-record-format.md "Format block").
 *
 * Any change to SD_FORMAT_COLUMNS_LINE or SD_FORMAT_RECORDS_LINES (adding,
 * removing or changing a label, its columns, units or tick rate) or to the
 * row layout needs a new SD_FORMAT_VERSION, the new schema hash in
 * SD_FORMAT_SCHEMA and in tests/test_sd_format_block.py, and the version
 * registered in the GroundStation26 SD parser. The test enforces it.
 *
 * SD_FORMAT_SCHEMA is the first 8 hex digits of SHA-256 over the columns
 * line and every records line, each with its CRLF. The firmware revision is
 * informational and injected by CMake. */
#ifndef SEDS_FIRMWARE_REVISION
#define SEDS_FIRMWARE_REVISION "unknown"
#endif

#define SD_FORMAT_ID "daq"
#define SD_FORMAT_VERSION "3"
#define SD_FORMAT_BOARD "DAQ"
#define SD_FORMAT_TICK_HZ "1000"
#define SD_FORMAT_SCHEMA "ade30ff7"

#define SD_FORMAT_COLUMNS_LINE \
    "# seds-sd-columns,u64:ms,u64:ms,label,vec,vec,vec,vec,clock,f32:degC,i32:code,u64:tick,u32:Hz,u64:seq\r\n"

/* MCP3564R raw records carry microsecond uptime_ticks (@1000000); ADC1
 * supply, VB pressure, scalar and DAC rows carry milliseconds. */
#define SD_FORMAT_RECORDS_LINES \
    "# seds-sd-records,kg1000_v:a=code:r=V:c=kg@1000000,kg50_v:a=code:r=V:c=kg@1000000,kg50_p6_v:a=code:r=V:c=V@1000000,chamber_pressure_v:a=code:r=V:c=psig@1000000,excitation_v:a=code:r=V:c=V@1000000\r\n" \
    "# seds-sd-records,mcp3564r_ch2_v:a=code:r=V:c=V@1000000,mcp3564r_ch3_v:a=code:r=V:c=V@1000000,mcp3564r_ch4_v:a=code:r=V:c=V@1000000,mcp3564r_ch5_v:a=code:r=V:c=V@1000000,mcp3564r_ch6_v:a=code:r=V:c=V@1000000,mcp3564r_ch7_v:a=code:r=V:c=V@1000000\r\n" \
    "# seds-sd-records,input_voltage_v:a=code:r=V:c=V,input_current_a:a=code:r=V:c=A,valve_pressure_psi:v=psi:a=code:r=code:c=psi\r\n" \
    "# seds-sd-records,adc1_aux_v:v=V,analog_in_1_v:v=V,analog_in_2_v:v=V,analog_in_3_v:v=V,analog_in_4_v:v=V,analog_in_5_v:v=V,analog_in_6_v:v=V,analog_in_7_v:v=V,analog_in_8_v:v=V\r\n" \
    "# seds-sd-records,isense1_v:v=V,isense2_v:v=V,isense1_a:v=A,isense2_a:v=A,vmon_v:v=V,imon_v:v=V,analog_out_1_v:v=V,analog_out_2_v:v=V,kg1000_window_code:v=code,kg1000_window_v:v=V,mcp3564r_temp_c:v=degC,kg50_window_v:v=V\r\n" \
    "# seds-sd-records,dac_ch1_active:v=1,dac_ch1_waveform:v=code,dac_ch1_amplitude_v:v=V,dac_ch1_offset_v:v=V,dac_ch1_frequency_hz:v=Hz,dac_ch1_steps:v=1,dac_ch1_dwell_ms:v=ms,dac_ch1_sequence:v=1\r\n" \
    "# seds-sd-records,dac_ch1_result:v=code,dac_ch1_off_reason:v=code,dac_ch1_update_hz:v=Hz,dac_ch1_samples:v=1,dac_ch1_duty:v=1,dac_ch1_output_v:v=V,dac_ch1_readback_v:v=V\r\n" \
    "# seds-sd-records,dac_ch2_active:v=1,dac_ch2_waveform:v=code,dac_ch2_amplitude_v:v=V,dac_ch2_offset_v:v=V,dac_ch2_frequency_hz:v=Hz,dac_ch2_steps:v=1,dac_ch2_dwell_ms:v=ms,dac_ch2_sequence:v=1\r\n" \
    "# seds-sd-records,dac_ch2_result:v=code,dac_ch2_off_reason:v=code,dac_ch2_update_hz:v=Hz,dac_ch2_samples:v=1,dac_ch2_duty:v=1,dac_ch2_output_v:v=V,dac_ch2_readback_v:v=V\r\n" \
    "# seds-sd-records,seds_packet:v=text\r\n"

#define SD_FORMAT_BLOCK \
    "# seds-sd,format=" SD_FORMAT_ID ",version=" SD_FORMAT_VERSION ",board=" SD_FORMAT_BOARD \
    ",firmware=" SEDS_FIRMWARE_REVISION ",tick_hz=" SD_FORMAT_TICK_HZ ",schema=" SD_FORMAT_SCHEMA "\r\n" \
    SD_FORMAT_COLUMNS_LINE SD_FORMAT_RECORDS_LINES

#endif
