#include "DAQ-Threads.h"

#include "sd_card.h"

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
                          4U,
                          4U,
                          TX_NO_TIME_SLICE,
                          TX_AUTO_START);
}
