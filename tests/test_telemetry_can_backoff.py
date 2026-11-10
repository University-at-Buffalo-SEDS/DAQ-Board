"""Telemetry must not spin on SDK work the CAN TX queue keeps refusing.

With CAN unacknowledged the SDK keeps refused frames queued, so
seds_router_has_queued_work stayed positive and the telemetry thread
(priority 6) relinquished only to its own priority between 1 ms queue
passes. The SD writer (priority 7) starved: optional snapshots waited in
their three slots and 24 rows were dropped per sample, losing the local
record exactly when the network link was down. A pass whose CAN submissions
were refused now sleeps instead.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef unsigned long ULONG;
typedef int SedsRouter;
static SedsRouter router; static struct { SedsRouter *r; } g_router = {&router};
static int queued, sleeps, relinquishes;
static int seds_router_has_queued_work(SedsRouter *r) { (void)r; return queued; }
static void tx_thread_sleep(ULONG t) { (void)t; ++sleeps; }
static void tx_thread_relinquish(void) { ++relinquishes; }
static uint8_t immediate_work_passes;
@WAIT@
int main(void) {
  queued = 3;
  /* Work that is moving gets a few immediate passes, then a sleep. */
  for (int i = 0; i < 5; ++i) wait_for_telemetry_work(50U, false);
  assert(relinquishes == 4 && sleeps == 1);
  /* Work the CAN queue just refused never spins. */
  relinquishes = sleeps = 0;
  for (int i = 0; i < 5; ++i) wait_for_telemetry_work(50U, true);
  assert(relinquishes == 0 && sleeps == 5);
  return 0;
}
'''


class TelemetryCanBackoffTests(unittest.TestCase):
    def test_refused_can_work_sleeps_instead_of_starving_the_sd_writer(self):
        source = (ROOT / 'Core/Src/telemetry_thread.c').read_text()
        wait = _function(source, 'static void wait_for_telemetry_work(')
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / 'wait.c'
            src.write_text(HARNESS.replace('@WAIT@', wait))
            exe = Path(directory) / 'wait'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(src), '-o', str(exe)],
                           check=True)
            subprocess.run([str(exe)], check=True)

    def test_loop_samples_can_refusals_around_each_pass(self):
        source = (ROOT / 'Core/Src/telemetry_thread.c').read_text()
        loop = _function(source, 'void telemetry_thread_entry(')
        before = loop.index('const uint32_t can_rejected = CAN_TX_REFUSALS();')
        work = loop.index('process_all_queues_timeout(1);')
        wait = loop.index('wait_for_telemetry_work(poll_ticks, CAN_TX_REFUSALS() != can_rejected);')
        self.assertLess(before, work)
        self.assertLess(work, wait)


if __name__ == '__main__':
    unittest.main()
