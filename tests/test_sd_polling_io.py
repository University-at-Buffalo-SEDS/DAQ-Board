"""Exercise the production DMA/FileX chunk adapter without an SD card or an RTOS."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class SdPollingTests(unittest.TestCase):
    def test_csv_queue_accepts_mount_pending_but_remains_bounded(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        adapter = "sd_card_status_t sd_card_enqueue_csv_row" + source.split(
            "sd_card_status_t sd_card_enqueue_csv_row", 1)[1].split(
            "sd_card_status_t sd_card_enqueue_raw_adc_samples", 1)[0]
        required=source[source.index("static uint8_t sd_required_sensor_name("):source.index("static uint64_t sd_row_sequence;")]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "daq_timestamp.h"
#include "sd_record_format.h"
typedef int sd_card_status_t;
#define SD_LINE_MAX 384U
typedef struct { float slope; } daq_calibration_t;
typedef struct { daq_calibration_t calibration; char line[384]; uint16_t len; uint64_t session; } sd_line_slot_t;
enum { SD_CARD_STATUS_OK, SD_CARD_STATUS_BUSY, SD_CARD_STATUS_BACKPRESSURE, SD_CARD_STATUS_IO_ERROR };
static unsigned g_sd_services_initialized, g_sd_line_drop_count, g_sd_optional_telemetry_dropped, g_sd_essential_telemetry_dropped;
static unsigned optional_allowed=1;
static unsigned sd_optional_telemetry_admitted(void){return optional_allowed;}
static sd_line_slot_t slot;
static int full, enqueued;
static sd_line_slot_t *sd_alloc_slot(uint16_t len) { assert(len < 384); return full ? NULL : &slot; }
static unsigned sd_calibration_snapshot(daq_calibration_t *c) { c->slope=1; return 0; }
static uint64_t sd_run_snapshot(void) { return 0; }
static uint64_t sd_next_row_sequence(void) { static unsigned sequence; return ++sequence; }
uint64_t board_uptime_ticks(void) { return 30; }
static uint64_t network_ms = 1234;
static uint64_t daq_log_unix_ms(void) { return network_ms; }
static uint64_t daq_log_monotonic_ms(void) { return 30; }
static int sd_enqueue_line(sd_line_slot_t *s) { assert(s->len); enqueued++; return SD_CARD_STATUS_OK; }
static int sd_scalar_sensor_index(const char *name){(void)name;return -1;}
static int sd_enqueue_scalar(uint8_t s,uint64_t n,uint64_t u,float v,const daq_calibration_t *c){(void)s;(void)n;(void)u;(void)v;(void)c;assert(0);return -1;}
''' + required + adapter + r'''
int main(void) {
  assert(sd_card_enqueue_csv_row("kg1000_network", 20, 0.25f, NULL)==SD_CARD_STATUS_BUSY);
  g_sd_services_initialized=1;
  /* No card-ready flag exists in this harness: the initialized bounded queue
   * must be usable before mounting completes. */
  assert(sd_card_enqueue_csv_row("kg1000_network", 20, 0.25f, NULL)==SD_CARD_STATUS_OK);
  assert(enqueued==1);
  assert(strcmp(slot.line, "1224,20,kg1000_network,\"2.50000000e-01\",,,,network,,,20,1000,1\r\n")==0);
  network_ms=0;
  assert(sd_card_enqueue_csv_row("kg1000_network", 25, 0.25f, NULL)==SD_CARD_STATUS_OK);
  assert(strcmp(slot.line, "25,25,kg1000_network,\"2.50000000e-01\",,,,local,,,25,1000,2\r\n")==0);
  enqueued=1;
  full=1;
  assert(sd_card_enqueue_csv_row("kg1000_network", 40, 0.25f, NULL)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_line_drop_count==1 && enqueued==1 && g_sd_essential_telemetry_dropped==1);
  full=0;
  assert(sd_card_enqueue_csv_row("kg1000_network", 60, 0.25f, NULL)==SD_CARD_STATUS_OK);
  optional_allowed=0;
  assert(sd_card_enqueue_csv_row("temperature",60,26.0f,NULL)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_optional_telemetry_dropped==1);
  assert(sd_card_enqueue_csv_row("kg50_network",60,0.25f,NULL)==SD_CARD_STATUS_OK);
  assert(sd_card_enqueue_csv_row("kg50_p6_network_volts",60,0.25f,NULL)==SD_CARD_STATUS_OK);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "queue-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c",
                 "-", "-I", str(ROOT / "Core/Inc"), "-o", str(binary)], input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

    def test_polling_transfers_enable_hardware_flow_control(self):
        self.assertIn("hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;",
                      (ROOT / "Core/Src/main.c").read_text())
        self.assertIn("SDMMC1.HardwareFlowControl=SDMMC_HARDWARE_FLOW_CONTROL_ENABLE",
                      (ROOT / "DAQ-Board.ioc").read_text())

    def test_chunking_data_integrity_and_errors(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        adapter = source.split("static UINT sd_hal_read", 1)[1].split(
            "static VOID sd_filex_driver", 1)[0]
        adapter = "static UINT sd_hal_read" + adapter
        # No cache APIs are supplied: the shared bounce buffer is internal SRAM;
        # completion, timeout and quiescence are covered by the DMA driver tests.
        code = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef unsigned char UCHAR;
#define SD_TRANSFER_SECTORS 8U
#define SD_SECTOR_SIZE 512U
#define HAL_OK 0
#define HAL_SD_CARD_TRANSFER 4U
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
static unsigned g_sd_ready=1;
static unsigned tick, reinit, mask;
static uint64_t g_sd_last_sector_write_ms;
static unsigned g_sd_write_progress_valid;
static uint64_t board_uptime_ticks(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static unsigned g_sd_max_read_ms, g_sd_max_write_ms;
static unsigned HAL_GetTick(void) { return ++tick; }
static unsigned sd_storage_io_requires_reinit(void){return reinit;}
static UCHAR g_sd_transfer_buffer[8*512];
static UCHAR disk[20*512];
static unsigned calls, fail_call;
static int transfer(UCHAR *buffer, ULONG sector, ULONG count, int write) {
  assert(count > 0 && count <= 8 && sector + count <= 20);
  if (++calls == fail_call) return FX_IO_ERROR;
  if (write) memcpy(disk + sector*512, buffer, count*512);
  else memcpy(buffer, disk + sector*512, count*512);
  return FX_SUCCESS;
}
static unsigned sd_storage_io_read(UCHAR *b, ULONG s, ULONG n) { return transfer(b,s,n,0); }
static unsigned sd_storage_io_write(const UCHAR *b, ULONG s, ULONG n) { return transfer((UCHAR*)b,s,n,1); }
''' + adapter + r'''
int main(void) {
  UCHAR input[17*512], output[17*512];
  for (size_t i=0; i<sizeof(input); ++i) input[i]=(UCHAR)(i*17+i/512);
  assert(sd_hal_write(input, 2, 17) == FX_SUCCESS && calls == 3);
  calls=0;
  assert(sd_hal_read(output, 2, 17) == FX_SUCCESS && calls == 3);
  assert(memcmp(input, output, sizeof(input)) == 0);
  calls=0; fail_call=2;
  assert(sd_hal_read(output, 2, 17) == FX_IO_ERROR && calls == 2);
  calls=0;
  assert(sd_hal_write(input, 2, 17) == FX_IO_ERROR && calls == 2);
  calls=0;
  assert(sd_hal_read(output, 2, 0) == FX_SUCCESS && calls == 0);
  assert(sd_hal_write(input, 2, 0) == FX_SUCCESS && calls == 0);
  calls=0; fail_call=1; reinit=1;
  assert(sd_hal_read(output,2,1)==FX_IO_ERROR && !g_sd_ready);
  g_sd_ready=1; calls=0;
  assert(sd_hal_write(input,2,1)==FX_IO_ERROR && !g_sd_ready);

}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "sd-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c",
                 "-", "-o", str(binary)], input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
