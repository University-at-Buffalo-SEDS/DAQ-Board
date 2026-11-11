#ifndef SD_PROFILE_H
#define SD_PROFILE_H
/* Bench-only SD writer profile (CMake -DDAQ_SD_PROFILE=ON). DWT CYCCNT runs at
 * the 160 MHz core clock and counts wall cycles, including time the writer is
 * preempted or blocked, so each field is the elapsed time inside that step.
 * Read g_sd_profile over SWD; see docs/sd-write-path.md for the procedure.
 * Every interval is far below the 26.8 s CYCCNT wrap. Normal builds compile
 * these macros away and do not define g_sd_profile. */
#include <stdint.h>

typedef struct {
  uint64_t encode_cycles;      /* raw CSV formatting (sd_format_csv_raw_record) */
  uint64_t file_write_cycles;  /* fx_file_write, including its card I/O */
  uint64_t io_cycles;          /* sd_storage_io_write/read: command, transfer and busy wait */
  uint64_t ready_sleep_cycles; /* busy-poll sleeps (whole ThreadX ticks) waiting for TRAN */
  uint64_t sync_cycles;        /* periodic/requested flush incl. fx_media_flush */
  uint32_t records, file_writes, io_writes, io_reads, io_sectors_written;
  uint32_t ready_sleeps, syncs, started_cycle;
} sd_profile_t;

#if defined(DAQ_SD_PROFILE) && DAQ_SD_PROFILE
#include "main.h"
extern volatile sd_profile_t g_sd_profile;
static inline void sd_profile_start(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_sd_profile.started_cycle = DWT->CYCCNT;
}
#define SD_PROF_BEGIN(name) const uint32_t name = DWT->CYCCNT
#define SD_PROF_END(field, name) (g_sd_profile.field += (uint32_t)(DWT->CYCCNT - (name)))
#define SD_PROF_COUNT(field, n) (g_sd_profile.field += (n))
#else
static inline void sd_profile_start(void) {}
#define SD_PROF_BEGIN(name) do { } while (0)
#define SD_PROF_END(field, name) do { } while (0)
#define SD_PROF_COUNT(field, n) do { } while (0)
#endif
#endif
