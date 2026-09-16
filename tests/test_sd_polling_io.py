"""Exercise the production polling adapter without an SD card or an RTOS."""
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
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "daq_timestamp.h"
typedef int sd_card_status_t;
typedef struct { float slope; } daq_calibration_t;
typedef struct { daq_calibration_t calibration; char line[384]; uint16_t len; } sd_line_slot_t;
enum { SD_CARD_STATUS_OK, SD_CARD_STATUS_BUSY, SD_CARD_STATUS_BACKPRESSURE, SD_CARD_STATUS_IO_ERROR };
static unsigned g_sd_services_initialized, g_power_loss_mode, g_sd_line_drop_count;
static sd_line_slot_t slot;
static int full, enqueued;
static sd_line_slot_t *sd_alloc_slot(void) { return full ? NULL : &slot; }
static void sd_free_slot(sd_line_slot_t *s) { (void)s; }
static unsigned sd_calibration_snapshot(daq_calibration_t *c) { c->slope=1; return 0; }
static void sd_format_float(char *s, float v) { snprintf(s,24,"%.3f",(double)v); }
static void sd_format_u64(char *s, uint64_t v) { snprintf(s,21,"%llu",(unsigned long long)v); }
static uint64_t network_ms = 1234;
static uint64_t telemetry_unix_ms(void) { return network_ms; }
static uint64_t telemetry_now_ms(void) { return 30; }
static int sd_enqueue_line(sd_line_slot_t *s) { assert(s->len); enqueued++; return SD_CARD_STATUS_OK; }
''' + adapter + r'''
int main(void) {
  assert(sd_card_enqueue_csv_row("kg1000_network", 20, 0.25f, NULL)==SD_CARD_STATUS_BUSY);
  g_sd_services_initialized=1;
  /* No card-ready flag exists in this harness: the initialized bounded queue
   * must be usable before mounting completes. */
  assert(sd_card_enqueue_csv_row("kg1000_network", 20, 0.25f, NULL)==SD_CARD_STATUS_OK);
  assert(enqueued==1);
  assert(strcmp(slot.line, "1224,20,kg1000_network,0.250,,,,network\r\n")==0);
  network_ms=0;
  assert(sd_card_enqueue_csv_row("kg1000_network", 25, 0.25f, NULL)==SD_CARD_STATUS_OK);
  assert(strcmp(slot.line, "25,25,kg1000_network,0.250,,,,local\r\n")==0);
  enqueued=1;
  full=1;
  assert(sd_card_enqueue_csv_row("kg1000_network", 40, 0.25f, NULL)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_line_drop_count==1 && enqueued==1);
  g_power_loss_mode=1;
  assert(sd_card_enqueue_csv_row("kg1000_network", 60, 0.25f, NULL)==SD_CARD_STATUS_BUSY);
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
        adapter = source.split("static UINT sd_wait_ready", 1)[1].split(
            "static VOID sd_filex_driver", 1)[0]
        adapter = "static UINT sd_wait_ready" + adapter
        # No cache APIs are supplied: DMA cache maintenance is invalid for
        # these CPU-only transfers and must not creep back into the adapter.
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
static int hsd1;
static unsigned tick, busy, program_ticks = 2U;
static unsigned HAL_GetTick(void) { return tick; }
static void tx_thread_sleep(unsigned n) { tick += n; }
static unsigned HAL_SD_GetCardState(int *h) {
  assert(h == &hsd1);
  if (busy) { --busy; return 7U; }
  return HAL_SD_CARD_TRANSFER;
}
static UCHAR g_sd_transfer_buffer[8*512];
static UCHAR disk[20*512];
static unsigned calls, fail_call;
static int transfer(UCHAR *buffer, ULONG sector, ULONG count, int write) {
  assert(busy == 0U); /* No command may interrupt card programming. */
  assert(count > 0 && count <= 8 && sector + count <= 20);
  if (++calls == fail_call) return 1;
  if (write) { memcpy(disk + sector*512, buffer, count*512); busy = program_ticks; }
  else memcpy(buffer, disk + sector*512, count*512);
  return HAL_OK;
}
static int HAL_SD_ReadBlocks(int *h, UCHAR *b, ULONG s, ULONG n, unsigned t) {
  assert(h == &hsd1 && t == 2000); return transfer(b, s, n, 0);
}
static int HAL_SD_WriteBlocks(int *h, UCHAR *b, ULONG s, ULONG n, unsigned t) {
  assert(h == &hsd1 && t == 2000); return transfer(b, s, n, 1);
}
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
  fail_call=0; busy=5000; tick=0;
  assert(sd_hal_read(output, 2, 1) == FX_IO_ERROR && calls == 0);
  assert(tick == 2000U);
  busy=0; tick=0; program_ticks=5000;
  assert(sd_hal_write(input, 2, 1) == FX_IO_ERROR && calls == 1);
  assert(tick == 2000U); /* Returning HAL_OK is not durable completion. */
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "sd-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c",
                 "-", "-o", str(binary)], input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
