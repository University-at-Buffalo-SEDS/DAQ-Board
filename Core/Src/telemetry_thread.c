// telemetry_thread.c
#include "DAQ-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE 1024u
ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

// How often this node requests a resync from the master:
#define TIMESYNC_REQUEST_PERIOD_MS 2000u   // e.g. every 2 seconds

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_now_ms(void) {
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    const char started_txt[] = "Telemetry thread starting";
    (void)log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,
                                    started_txt,
                                    sizeof(started_txt),
                                    1);

    uint64_t last_req_ms = 0;

    for (;;) {
        can_bus_process_rx();
        (void)process_all_queues_timeout(5);
        can_bus_process_rx();

        const uint64_t now_ms = tx_now_ms();
        if ((uint64_t)(now_ms - last_req_ms) >= (uint64_t)TIMESYNC_REQUEST_PERIOD_MS) {
            (void)telemetry_timesync_request();
            last_req_ms = now_ms;
        }

        tx_thread_sleep(1);
    }
}

void create_telemetry_thread(void)
{
    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   telemetry_thread_stack,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    if (status != TX_SUCCESS) {
        die("Failed to create telemetry thread: %u", (unsigned)status);
    }
}
