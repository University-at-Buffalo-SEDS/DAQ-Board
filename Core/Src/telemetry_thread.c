// telemetry_thread.c
#include "DAQ-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE (16U *1024U)

// How often this node requests a resync from the master:
#define TIMESYNC_REQUEST_PERIOD_MS 2000u // e.g. every 2 seconds

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

#define TIMESYNC_REQUEST_PERIOD_TICKS \
  ((TIMESYNC_REQUEST_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u)


void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    ULONG last_req_ticks = tx_time_get();
    for (;;) {
        /* Poll hardware FIFO and then process reassembly + router queues. */
        // can_bus_poll();
        can_bus_process_rx();
        (void)process_all_queues_timeout(50);

         ULONG now_ticks = tx_time_get();
        if ((ULONG)(now_ticks - last_req_ticks) >= (ULONG)TIMESYNC_REQUEST_PERIOD_TICKS) {
            (void)telemetry_timesync_request();
            last_req_ticks = now_ticks;
        }

        tx_thread_sleep(1);
    }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TELEMETRY_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   pointer,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
