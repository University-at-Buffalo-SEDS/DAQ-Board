"""Exercise the actual analog publisher under bounded startup queue pressure."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class AnalogBackpressureTests(unittest.TestCase):
    def test_retry_preserves_reports_and_shared_deadline_without_allocating(self):
        source = (ROOT / "Core/Src/daq_thread.c").read_text()
        start = source.index("volatile uint32_t g_daq_analog_publish_ok_count;")
        end = source.index("static void daq_publish_analog(", start)
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef unsigned SedsDataType;
typedef int SedsResult;
enum { SEDS_OK=0, SEDS_IO=-14, SEDS_BAD_ARG=-10 };
static uint32_t now;
static unsigned calls, sleeps, refusals, accepted[4];
static int permanent;
static float saved[4][8];
static uint32_t HAL_GetTick(void) { return now; }
static void tx_thread_sleep(unsigned ticks) {
    assert(ticks == (TX_TIMER_TICKS_PER_SECOND+999U)/1000U);
    now += ticks*1000U/TX_TIMER_TICKS_PER_SECOND;
    sleeps++;
}
static SedsResult log_telemetry_asynchronous(unsigned type, const float *p,
                                             size_t count, size_t size) {
    assert(type<4 && count<=8 && size==sizeof(float));
    calls++;
    if (permanent) return permanent;
    if (refusals) { refusals--; return SEDS_IO; }
    accepted[type]++;
    memcpy(saved[type], p, count*size);
    return SEDS_OK;
}
''' + source[start:end] + r'''
int main(void) {
    const float values[8]={1,2,3,4,5,6,7,8};
    refusals=4;
    const uint32_t batch=now;
    for (unsigned type=0; type<4; type++)
        daq_publish_analog_packet(type, values, 8, batch);
    assert(calls==8 && sleeps==4 && now==4);
    assert(g_daq_analog_publish_retry_count==4);
    assert(g_daq_analog_publish_ok_count==4 && !g_daq_analog_publish_fail_count);
    for (unsigned type=0; type<4; type++) {
        assert(accepted[type]==1);
        assert(!memcmp(saved[type],values,sizeof(values)));
    }
    // Persistent pressure expires once for the whole snapshot, including
    // rollover. Later packets still get one attempt, without another wait.
    now=UINT32_MAX-50U;
    const uint32_t wrapped=now;
    const unsigned before=sleeps;
    permanent=SEDS_IO;
    daq_publish_analog_packet(0, values, 8, wrapped);
    assert((uint32_t)(now-wrapped)==100U && sleeps-before==100);
    daq_publish_analog_packet(1, values, 8, wrapped);
    assert((uint32_t)(now-wrapped)==100U && sleeps-before==100);
    assert(g_daq_analog_publish_fail_count==2);
    permanent=SEDS_BAD_ARG;
    const unsigned before_error=sleeps;
    daq_publish_analog_packet(2, values, 8, now);
    assert(sleeps==before_error && g_daq_analog_publish_fail_count==3);
    assert(g_daq_analog_publish_ok_count==4);
}
'''
        for frequency in (1000, 50000):
            with self.subTest(frequency=frequency), tempfile.TemporaryDirectory() as tmp:
                binary = Path(tmp) / "analog-backpressure"
                subprocess.run(
                    ["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                     "-fsanitize=address,undefined",
                     f"-DTX_TIMER_TICKS_PER_SECOND={frequency}U", "-x", "c", "-",
                     "-o", str(binary)], input=harness, text=True, check=True,
                )
                subprocess.run([str(binary)], check=True)
