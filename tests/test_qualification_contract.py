import json
import shutil
import subprocess
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_acquisition_and_network_worker_use_real_time_periods(self):
        root = Path(build.__file__).resolve().parent
        worker = (root / "Core/Src/telemetry_thread.c").read_text()
        self.assertIn("(TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U", worker)
        self.assertIn("tx_thread_sleep(poll_ticks)", worker)
        self.assertIn("(now - last_stack_sample) >= stack_sample_ticks", worker)
        acquisition = (root / "Core/Src/daq_thread.c").read_text()
        self.assertIn("DAQ_SAMPLE_PERIOD_TICKS - elapsed", acquisition)
        self.assertIn("g_daq_sample_overrun_count++", acquisition)
        ioc = (root / "DAQ-Board.ioc").read_text()
        self.assertIn("RCC.APB1TimFreq_Value=160000000", ioc)
        self.assertIn("TIM2.Prescaler=159", ioc)
        self.assertIn("htim2.Init.Prescaler = 159;",
                      (root / "Core/Src/main.c").read_text())

    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertEqual(runner.count('str(max(1000, layout["execution"]["virtual_time_ms"]))'), 2)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"GS_SIM_VALIDATE_SOAK_COMMANDS": "1" if ultra_soak else "0"', runner)
        self.assertIn("Valve command path remained alive during soak interval", runner)
        self.assertIn("Every ten-minute soak command returned an acknowledgement", runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn("routed status ACK toward GroundStation", runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn("Long-duration memory profile", script)
        self.assertIn("Network discovery and time sync", script)

    def test_layout_exposes_network_convergence(self):
        root = Path(build.__file__).resolve().parent
        from sim.run_full import load_layout_for_build
        layout = load_layout_for_build(root, None)
        self.assertLess(layout["execution"].get("memory_probe_warmup_samples", 0), layout["execution"]["sample_count"])
        probes = {
            probe["name"]: probe["symbol"]
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(probes["network_ready"], "g_telemetry_network_ready")
        self.assertEqual(probes["discovery_seen"], "g_telemetry_discovery_seen")
        self.assertEqual(probes["timesync_valid"], "g_telemetry_timesync_valid")

        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

    def test_hal_tick_injection_matches_linked_elf_when_available(self):
        root = Path(build.__file__).resolve().parent
        from sim.run_full import load_layout_for_build
        layout = load_layout_for_build(root, None)
        source = json.loads((root / "sim/board.json").read_text())
        if "hal_tick_address" not in source.get("execution", {}):
            self.assertNotIn("hal_tick_address", layout["execution"])
            return
        elf = root / layout["artifacts"]["elf"]
        nm = shutil.which("arm-none-eabi-nm")
        if nm is None or not elf.is_file():
            self.skipTest("linked ELF or arm-none-eabi-nm is unavailable")
        output = subprocess.check_output([nm, "-n", str(elf)], text=True)
        address = next(int(line.split()[0], 16) for line in output.splitlines()
                       if line.split()[-1:] == ["uwTick"])
        self.assertEqual(layout["execution"]["hal_tick_address"], address)

    def test_shared_can_avoids_hop_retry_storms(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("seds_router_add_side_packed_profile(", telemetry)
        self.assertIn("SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE", telemetry)
        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("can_bus_wait_for_tx_slot", can_bus)
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can_bus)
        self.assertNotIn("< (uint32_t)frag_cnt", can_bus)
        self.assertIn("BOARD_CAN_MAX_FRAME_BYTES 128U", telemetry)
        self.assertIn('SEDSNET_MAX_QUEUE_BUDGET "12288"', cmake)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertIn("g_telemetry_discovery_seen = 1U", telemetry)

    def test_green_leds_report_completed_can_packet_activity(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        daq_thread = (root / "Core" / "Src" / "daq_thread.c").read_text(encoding="utf-8")

        send_success = telemetry.index("if (can_bus_send_large(bytes, len, can_id) == HAL_OK)")
        tx_indicator = telemetry.index("HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)", send_success)
        self.assertGreater(tx_indicator, send_success)

        receive_callback = telemetry.index("static void telemetry_can_rx")
        rx_indicator = telemetry.index("HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)", receive_callback)
        receive_dispatch = telemetry.index("rx_asynchronous(data, len)", receive_callback)
        self.assertLess(rx_indicator, receive_dispatch)
        self.assertNotIn("HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)", daq_thread)

    def test_sigma_delta_adc_uses_fast_low_noise_timing_and_bounded_queue(self):
        root = Path(build.__file__).resolve().parent
        adc = (root / "Core" / "Src" / "mcp3564r.c").read_text(encoding="utf-8")
        header = (root / "Core" / "Inc" / "mcp3564r.h").read_text(encoding="utf-8")

        # MCO is 16 MHz. OSR=1024 gives 3.90625 ksps after the SINC3 filter,
        # while gain-16 noise remains sub-microvolt RMS per the datasheet.
        self.assertIn(".config1_reg = (DAQ_ADC_OSR_BITS << 2U)", adc)
        self.assertIn("MCP3564R_FIRST_CONVERSION_US DAQ_ADC_FIRST_CONVERSION_US", adc)
        self.assertIn("MCP3564R_DEFAULT_START_OFFSET_US DAQ_ADC_READ_INTERVAL_US", adc)
        self.assertIn("first_conversion_pending", adc)
        self.assertIn("MCP3564R_SAMPLE_QUEUE_DEPTH (128U)", adc)
        self.assertIn("overrun_count++", adc)
        self.assertIn("uint8_t mcp3564r_pending_samples(void)", header)

    def test_daq_batches_high_rate_adc_without_starving_telemetry(self):
        root = Path(build.__file__).resolve().parent
        daq = (root / "Core" / "Src" / "daq_thread.c").read_text(encoding="utf-8")
        sd = (root / "Core" / "Src" / "sd_card.c").read_text(encoding="utf-8")
        writer = (root / "Core" / "Src" / "sd_writer_thread.c").read_text(encoding="utf-8")

        self.assertIn("DAQ_SAMPLE_PERIOD_MS DAQ_ACQUISITION_PERIOD_MS", daq)
        self.assertIn("daq_drain_ext_adc", daq)
        self.assertIn("sd_card_enqueue_raw_adc_samples", daq)
        self.assertIn('sd_card_enqueue_csv_row("kg1000_network"', daq)
        self.assertIn("calibration->kg1000_slope * snapshot->ext_adc_loadcell_kg1000", daq)
        self.assertIn("const daq_calibration_t calibration = daq_calibration_current();", daq)
        self.assertLess(
            daq.index('sd_card_enqueue_csv_row("kg1000_network"'),
            daq.index("log_telemetry_asynchronous(SEDS_DT_KG1000"),
        )
        self.assertIn("g_daq_sd_network_row_ok_count", daq)
        self.assertIn("g_daq_raw_samples_drained_count", daq)
        self.assertNotIn("sd_card_request_flush();", daq)
        self.assertIn("SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY", sd)
        self.assertIn("fx_media_open", sd)
        self.assertIn("fx_file_create", sd)
        self.assertIn("fx_file_write", sd)
        self.assertIn("g_sd_write_buffer[4096U]", sd)
        self.assertIn("last_flush", sd)
        self.assertIn('SD_PROVISION_MARKER "SEDSDAQ.ID"', sd)
        self.assertIn("sd_mount_or_provision", sd)
        self.assertIn("fx_media_format", sd)
        self.assertIn("DAQ SD card unavailable; acquisition continues without logging", sd)
        self.assertIn("g_sd_warning_publish_count", sd)
        self.assertIn("g_sd_services_initialized", sd)
        self.assertIn("g_sd_raw_records_written_count", sd)
        self.assertIn("g_sd_csv_rows_written_count", sd)
        self.assertIn("HAL_SD_Init(&hsd1)", sd)
        self.assertNotIn("return TX_NOT_DONE;", sd[sd.index("UINT sd_card_init") :])
        self.assertIn("network_unix_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value", sd)
        self.assertIn("# calibration,kg1000_slope=", sd)
        self.assertIn("sd_calibration_snapshot", sd)
        drain = sd.index("if (tx_queue_receive(&g_sd_raw_queue")
        self.assertLess(sd.index("sd_select_calibration(&slot->calibration)", drain),
                        sd.index("for (uint16_t i", drain))
        self.assertIn("slot->calibration = *calibration;", sd)
        self.assertIn("sd_open_timestamped_log(calibration)", sd)
        # Raw acquisition is continuous: an unbounded drain can prevent CSV
        # rows, flushes and calibration rotation from ever being serviced.
        self.assertNotIn("while (tx_queue_receive(&g_sd_raw_queue", sd)
        csv = sd.index("if (tx_queue_receive(&g_sd_queue", drain)
        self.assertIn("TX_NO_WAIT", sd[csv:csv + 90])
        self.assertIn("else if (serviced_work == 0U)", sd[csv:])
        calibration = (root / "Core" / "Src" / "daq_calibration.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("persistent_store_set", calibration)
        self.assertIn("persistent_store_get", calibration)
        self.assertIn("sd_card_set_calibration(&next)", calibration)
        self.assertIn('"SD Writer"', writer)
        self.assertGreaterEqual(writer.count("8U,"), 2)

if __name__ == "__main__":
    unittest.main()
