"""Execute the production CAN timeout loop with thread/ISR scheduling mocks."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CanTxYieldTests(unittest.TestCase):
    def test_bounded_wait_yields_only_in_running_thread_context(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        function = source.split("static HAL_StatusTypeDef can_bus_wait_for_tx_slot", 1)[1]
        function = function.split("static can_bus_rx_frame_t g_rx_ring", 1)[0]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_TIMEOUT } HAL_StatusTypeDef;
#define CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U
#define TX_NULL NULL
#define TX_THREAD_GET_SYSTEM_STATE() system_state
static unsigned tick, sleeps, ipsr, system_state, free_slots, recover_error;
static int task;
static void *g_hfdcan, *current = &task;
static unsigned HAL_GetTick(void) { return tick++; }
static unsigned __get_IPSR(void) { return ipsr; }
static void *tx_thread_identify(void) { return current; }
static unsigned tx_thread_sleep(unsigned ticks) {
  assert(ticks == 1); ++sleeps; return 0;
}
static unsigned HAL_FDCAN_GetTxFifoFreeLevel(void *handle) {
  (void)handle; return free_slots;
}
static HAL_StatusTypeDef can_bus_recover_if_bus_off(void) {
  return recover_error ? HAL_ERROR : HAL_OK;
}
''' + "static HAL_StatusTypeDef can_bus_wait_for_tx_slot" + function + r'''
int main(void) {
  assert(can_bus_wait_for_tx_slot() == HAL_TIMEOUT);
  assert(sleeps > 0 && tick <= 7);
  tick = sleeps = 0; ipsr = 16;
  assert(can_bus_wait_for_tx_slot() == HAL_TIMEOUT && sleeps == 0);
  tick = 0; ipsr = 0; system_state = 1;
  assert(can_bus_wait_for_tx_slot() == HAL_TIMEOUT && sleeps == 0);
  tick = 0; system_state = 0; current = NULL;
  assert(can_bus_wait_for_tx_slot() == HAL_TIMEOUT && sleeps == 0);
  tick = 0; current = &task; free_slots = 1;
  assert(can_bus_wait_for_tx_slot() == HAL_OK && sleeps == 0);
  free_slots = 0; recover_error = 1;
  assert(can_bus_wait_for_tx_slot() == HAL_ERROR && sleeps == 0);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source_file = Path(directory) / "test.c"
            executable = Path(directory) / "test"
            source_file.write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(source_file), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
