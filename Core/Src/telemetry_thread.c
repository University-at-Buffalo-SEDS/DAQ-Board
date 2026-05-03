// telemetry_thread.c
#include "DAQ-Threads.h"
#ifdef TELEMETRY_BOARD_LINK_UART
#include "board_link_uart.h"
#endif
#include "tx_api.h"
#include "telemetry.h"
#ifdef TELEMETRY_CAN_BUS
#include "can_bus.h"
#endif
#include "main.h"

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE (16U * 1024U)
#ifdef TELEMETRY_CAN_BUS
extern FDCAN_HandleTypeDef hfdcan1;
#endif
#ifdef TELEMETRY_BOARD_LINK_UART
extern UART_HandleTypeDef huart1;
#endif

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

#ifdef TELEMETRY_CAN_BUS
    (void)can_bus_init(&hfdcan1);
#endif
#ifdef TELEMETRY_BOARD_LINK_UART
    board_link_uart_init(&huart1);
    (void)board_link_uart_start_rx();
#endif
    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    for (;;)
    {
#ifdef TELEMETRY_BOARD_LINK_UART
        board_link_uart_process();
#endif
#ifdef TELEMETRY_CAN_BUS
        can_bus_process_rx();
#endif
        (void)telemetry_poll_discovery();
        (void)process_all_queues_timeout(50);
#ifdef TELEMETRY_BOARD_LINK_UART
        board_link_uart_process();
#endif
        (void)telemetry_poll_timesync();

        tx_thread_sleep(1);
    }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{

    CHAR *pointer;

    /* Allocate the stack for test  */
    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
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
