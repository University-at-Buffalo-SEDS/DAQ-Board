"""Exercise the production ISR ring's overflow and wraparound behavior."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CanRxRingTests(unittest.TestCase):
    def test_overflow_only_interrupt_drains_retained_frames(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        callbacks = source[source.index("void HAL_FDCAN_RxFifo0Callback"):]
        code = r'''
#include <stdint.h>
#include <assert.h>
typedef int FDCAN_HandleTypeDef;
static FDCAN_HandleTypeDef controller, other;
static FDCAN_HandleTypeDef *g_hfdcan = &controller;
static unsigned g_fdcan_rx_hw_overflow_count, drains[2];
#define FDCAN_RX_FIFO0 0
#define FDCAN_RX_FIFO1 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 4
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 8
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST 32
static void can_bus_drain_rx_fifo(FDCAN_HandleTypeDef *handle, unsigned fifo) {
    assert(handle == g_hfdcan); ++drains[fifo];
}
''' + callbacks + r'''
int main(void) {
    HAL_FDCAN_RxFifo0Callback(g_hfdcan, 4);
    HAL_FDCAN_RxFifo1Callback(g_hfdcan, 32);
    assert(drains[0] == 1 && drains[1] == 1 && g_fdcan_rx_hw_overflow_count == 2);
    HAL_FDCAN_RxFifo0Callback(g_hfdcan, 1);
    HAL_FDCAN_RxFifo1Callback(g_hfdcan, 8);
    assert(drains[0] == 2 && drains[1] == 2 && g_fdcan_rx_hw_overflow_count == 2);
    HAL_FDCAN_RxFifo0Callback(&other, 4);
    HAL_FDCAN_RxFifo1Callback(&other, 32);
    HAL_FDCAN_RxFifo0Callback(g_hfdcan, 0);
    HAL_FDCAN_RxFifo1Callback(g_hfdcan, 0);
    assert(drains[0] == 2 && drains[1] == 2 && g_fdcan_rx_hw_overflow_count == 2);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / "callbacks.c"
            exe = Path(directory) / "callbacks"
            c.write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_overflow_preserves_consumer_slot_and_wraparound(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        ring = source.split("static can_bus_rx_frame_t g_rx_ring", 1)[1].split(
            "// Reassembly state", 1)[0]
        code = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#define CAN_BUS_RX_RING_DEPTH 4
#define __DMB() do {} while (0)
typedef struct { uint32_t std_id; uint8_t len; uint8_t data[64]; } can_bus_rx_frame_t;
static volatile uint16_t g_rx_head, g_rx_tail;
static volatile uint32_t g_fdcan_rx_ring_drop_count;
''' + "static can_bus_rx_frame_t g_rx_ring" + ring + r'''
int main(void) {
  uint8_t payload[64]; can_bus_rx_frame_t out;
  for (unsigned i=1; i<=3; ++i) {
    memset(payload, i, sizeof(payload));
    rb_push_drop_newest(i, payload, sizeof(payload));
  }
  uint16_t tail=g_rx_tail;
  rb_push_drop_newest(99, payload, sizeof(payload));
  assert(g_rx_tail==tail && g_fdcan_rx_ring_drop_count==1);
  assert(rb_pop(&out) && out.std_id==1 && out.data[63]==1);
  memset(payload, 4, sizeof(payload));
  rb_push_drop_newest(4, payload, sizeof(payload));
  for (unsigned i=2; i<=4; ++i) {
    assert(rb_pop(&out) && out.std_id==i && out.len==64);
    for (unsigned j=0; j<64; ++j) assert(out.data[j]==i);
  }
  assert(!rb_pop(&out));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / "ring.c"
            exe = Path(directory) / "ring"
            c.write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_receive_interrupts_enabled_after_ring_initialization(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("#define CAN_BUS_POLLING 0", source)
        init = source.split("void can_bus_init", 1)[1].split(
            "HAL_StatusTypeDef can_bus_subscribe_rx", 1)[0]
        self.assertLess(init.index("g_rx_head = 0"),
                        init.index("HAL_FDCAN_ActivateNotification"))
        self.assertLess(init.index("HAL_FDCAN_Start"),
                        init.index("HAL_FDCAN_ActivateNotification"))
        self.assertIn("FDCAN_IT_RX_FIFO0_MESSAGE_LOST", init)
