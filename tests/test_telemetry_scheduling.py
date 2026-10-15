from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TelemetrySchedulingTests(unittest.TestCase):
    def test_acquisition_preempts_network_work_and_io_peers_share_cpu(self):
        source = (ROOT / 'Core/Src/telemetry_thread.c').read_text()
        body = source[source.index('UINT create_telemetry_thread('):]
        stub = r'''
#include <stdint.h>
#include <assert.h>
#include "daq_rates.h"
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef char CHAR;
typedef void VOID;
typedef int TX_BYTE_POOL;
typedef int TX_THREAD;
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_NO_WAIT 0U
#define TX_SUCCESS 0U
#define TX_POOL_ERROR 1U
#define TX_AUTO_START 1U
#define TX_NO_TIME_SLICE 0U
#define TELEMETRY_THREAD_STACK_SIZE 32768U
static TX_THREAD telemetry_thread;
static char stack[TELEMETRY_THREAD_STACK_SIZE];
static UINT priority, threshold, slice, creates, allocation_result;
static void telemetry_thread_entry(ULONG value) {(void)value;}
static UINT tx_byte_allocate(TX_BYTE_POOL *pool, VOID **pointer, ULONG size, UINT wait) {
 (void)pool; (void)wait; assert(size==sizeof(stack)); *pointer=stack; return allocation_result;
}
static UINT tx_thread_create(TX_THREAD *thread, CHAR *name, void (*entry)(ULONG), ULONG arg,
 VOID *pointer, ULONG size, UINT p, UINT t, ULONG s, UINT start) {
 (void)thread; (void)name; (void)entry; (void)arg; (void)pointer; (void)size; (void)start;
 priority=p; threshold=t; slice=s; creates++; return TX_SUCCESS;
}
'''
        main = r'''
int main(void) {
 TX_BYTE_POOL pool=0;
 assert(create_telemetry_thread(&pool)==TX_SUCCESS);
 /* Acquisition unblocks while discovery work is running: ThreadX must
  * permit it to preempt immediately, including the preemption threshold. */
 assert(DAQ_ACQUISITION_THREAD_PRIORITY < priority);
 assert(DAQ_ACQUISITION_THREAD_PRIORITY < threshold);
 /* A ready SD/report worker at the same I/O priority must get a bounded
  * slice even if the network stays busy. */
 assert(priority==DAQ_IO_THREAD_PRIORITY);
 assert(slice>0 && slice<=((TX_TIMER_TICKS_PER_SECOND*DAQ_IO_THREAD_SLICE_MS+999U)/1000U));
 allocation_result=TX_POOL_ERROR;
 assert(create_telemetry_thread(&pool)==TX_POOL_ERROR && creates==1);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'test'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc'),
                            '-x', 'c', '-', '-o', str(exe)],
                           input=stub + body + main, text=True, check=True)
            subprocess.run([str(exe)], check=True, timeout=5)
