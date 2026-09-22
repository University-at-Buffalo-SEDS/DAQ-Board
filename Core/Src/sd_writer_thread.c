#include "DAQ-Threads.h"

#include "sd_card.h"
#include "daq_rates.h"

TX_THREAD sd_writer_thread;

#define SD_WRITER_THREAD_STACK_SIZE (8U * 1024U)

static ULONG g_sd_writer_thread_stack[SD_WRITER_THREAD_STACK_SIZE / sizeof(ULONG)];

void sd_writer_thread_entry(ULONG initial_input)
{
  sd_card_writer_thread_entry(initial_input);
}

UINT create_sd_writer_thread(void)
{
  return tx_thread_create(&sd_writer_thread,
                          "SD Writer",
                          sd_writer_thread_entry,
                          0U,
                          g_sd_writer_thread_stack,
                          sizeof(g_sd_writer_thread_stack),
                          DAQ_IO_THREAD_PRIORITY,
                          DAQ_IO_THREAD_PRIORITY,
                          (TX_TIMER_TICKS_PER_SECOND * DAQ_IO_THREAD_SLICE_MS + 999U) / 1000U,
                          TX_AUTO_START);
}
