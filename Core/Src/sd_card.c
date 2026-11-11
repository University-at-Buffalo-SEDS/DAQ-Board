#include "sd_card.h"
#include "DAQ-Threads.h"
#include "daq_chamber_pressure.h"
#include "daq_rates.h"

#include "fx_api.h"
#include "main.h"
#include "daq_log_time.h"
#ifdef TELEMETRY_ENABLED
#include "daq_log_clock.h"
#endif
#include "sd_float_format.h"
#include "sd_record_format.h"
#include "sd_raw_record_format.h"
#include "sd_file_limit.h"
#include "sd_storage_config.h"
#include "sd_storage_io.h"
#include "daq_timestamp.h"
#include "sd_calendar.h"
#include "sd_log_path.h"
#include "sd_file_stream.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>


extern SD_HandleTypeDef hsd1;
UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition,
                                    ULONG *partition_start,
                                    ULONG *partition_size);

/* Report rows also accumulate while the card stalls; 24 slots overflowed
 * during ordinary flushes even when raw recording kept up. */
#define SD_QUEUE_DEPTH 1024U
#define SD_RAW_QUEUE_DEPTH 1024U
#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
#define SD_LINE_MAX 384U
#define SD_MEDIA_CACHE_SIZE (8U * 512U)
#define SD_TRANSFER_SECTORS 8U
#define SD_SECTOR_SIZE 512U
#define SD_FLAG_FLUSH_REQUEST 0x00000001UL
#define SD_FLAG_POWER_LOSS 0x00000002UL
#define SD_PROVISION_MARKER "SEDSDAQ.ID"

#if defined(__GNUC__)
#define SD_ALIGN_32 __attribute__((aligned(32)))
#else
#define SD_ALIGN_32
#endif

typedef struct
{
  uint16_t len;
  uint32_t route_epoch;
  daq_calibration_t calibration;
  sd_capture_t capture;
  uint64_t session;
  char line[];
} sd_line_slot_t;

typedef struct
{
  uint16_t count, capacity;
  uint32_t route_epoch;
  uint64_t uptime_reference;
  daq_calibration_t calibration;
  sd_capture_t capture;
  uint64_t session;
  sd_raw_adc_record_t samples[];
} sd_raw_slot_t;

/* Compact acquisition rows share immutable calibration/session metadata. The
 * writer expands them to the exact same CSV only after dequeueing ownership. */
#define SD_SCALAR_SLOT_TAG UINT16_MAX
#define SD_SCALAR_BATCH_MAX 32U
#define SD_RAW_SLOT_CAPACITY ((SD_RAW_BATCH_MAX) > 32U ? (SD_RAW_BATCH_MAX) : 32U)
typedef struct {
  uint64_t network_ms;
  uint32_t monotonic_ms, sequence_delta;
  float value;
  uint8_t sensor;
} sd_scalar_record_t;
typedef struct {
  uint16_t len;
  uint32_t route_epoch;
  daq_calibration_t calibration;
  sd_capture_t capture;
  uint64_t session;
  uint16_t count, capacity;
  uint64_t uptime_reference, sequence_base;
  sd_scalar_record_t records[];
} sd_scalar_slot_t;
_Static_assert(offsetof(sd_scalar_slot_t, count) == offsetof(sd_line_slot_t, line),
               "Scalar and CSV slots must share their routing prefix");
/* Every pool allocation is a constant-time fixed-block pop. Reserve padding
 * for8-byte alignment even though ThreadX guarantees only ULONG alignment. */
#define SD_RAW_BLOCK_PAYLOAD (offsetof(sd_raw_slot_t, samples) + SD_RAW_SLOT_CAPACITY * sizeof(sd_raw_adc_record_t))
#define SD_SCALAR_BLOCK_PAYLOAD (offsetof(sd_scalar_slot_t, records) + SD_SCALAR_BATCH_MAX * sizeof(sd_scalar_record_t) + 1U)
#define SD_CSV_BLOCK_PAYLOAD (offsetof(sd_line_slot_t, line) + SD_LINE_MAX + 1U)
#define SD_LINE_BLOCK_PAYLOAD ((SD_SCALAR_BLOCK_PAYLOAD > SD_CSV_BLOCK_PAYLOAD) ? SD_SCALAR_BLOCK_PAYLOAD : SD_CSV_BLOCK_PAYLOAD)
#define SD_POOL_ALIGNMENT_RESERVE (sizeof(void *) + 7U)



/* Static SD-only owners enter the same FIFO as compact and CSV owners.
 * They never hold an acquisition/network work block or consume its pool. */
#define SD_OPTIONAL_SNAPSHOT_TAG (UINT16_MAX - 1U)
#define SD_OPTIONAL_SNAPSHOT_SLOTS 3U
#ifndef SD_OPTIONAL_SNAPSHOT_ROWS
#define SD_OPTIONAL_SNAPSHOT_ROWS 24U
#endif
typedef struct {
  uint16_t len;
  uint32_t route_epoch;
  daq_calibration_t calibration;
  sd_capture_t capture;
  uint64_t session;
  float values[SD_OPTIONAL_SNAPSHOT_ROWS];
  uint64_t network_ms, uptime, sequence_base;
} sd_optional_snapshot_t;
_Static_assert(offsetof(sd_optional_snapshot_t, values) == offsetof(sd_line_slot_t, line),
               "Optional snapshot owners share the ordinary routing prefix");
_Static_assert(offsetof(sd_line_slot_t, line) == 88U, "Capture lease fills line padding");
_Static_assert(offsetof(sd_raw_slot_t, samples) == 96U, "Capture lease fills raw padding");
_Static_assert(offsetof(sd_scalar_slot_t, records) == 112U, "Capture lease fills scalar padding");
_Static_assert(sizeof(sd_optional_snapshot_t) == 208U, "Capture lease fills snapshot padding");
_Static_assert(sizeof(sd_optional_snapshot_t) * SD_OPTIONAL_SNAPSHOT_SLOTS + 64U < 1024U,
               "SD-only optional retention must remain below one KiB");

static TX_QUEUE g_sd_queue;
static ULONG g_sd_queue_storage[SD_QUEUE_DEPTH];
static TX_QUEUE g_sd_raw_queue;
static ULONG g_sd_raw_queue_storage[SD_RAW_QUEUE_DEPTH];
/* Only a queued producer tail may be extended. The writer clears ownership
 * atomically when it removes that exact slot from the queue. */
static sd_raw_slot_t *g_sd_raw_append_tail;
static sd_scalar_slot_t *g_sd_scalar_append_tail;
static const char *const g_sd_scalar_sensors[] = {
  "kg1000_network", "kg50_network", "kg50_p6_network_volts",
  "chamber_pressure_raw_volts_psi", "valve_pressure_psi",
  "input_voltage_v",
  "input_current_a",
  "adc1_aux_v",
  "analog_in_1_v",
  "analog_in_2_v",
  "analog_in_3_v",
  "analog_in_4_v",
  "analog_in_5_v",
  "analog_in_6_v",
  "analog_in_7_v",
  "analog_in_8_v",
  "isense1_v",
  "isense2_v",
  "isense1_a",
  "isense2_a",
  "vmon_v",
  "imon_v",
  "analog_out_1_v",
  "analog_out_2_v",
  "mcp3564r_code",
  "mcp3564r_voltage_v",
  "kg1000_raw_window_average",
  "mcp3564r_temp_c",
  "mcp3564r_ch1_voltage_v"
};
#define SD_REQUIRED_SCALAR_SENSORS 5U
_Static_assert(sizeof(g_sd_scalar_sensors) / sizeof(g_sd_scalar_sensors[0]) ==
    SD_REQUIRED_SCALAR_SENSORS + SD_OPTIONAL_SNAPSHOT_ROWS,
    "Snapshot field order must match the existing optional sensor names");
_Static_assert(sizeof(g_sd_scalar_sensors) / sizeof(g_sd_scalar_sensors[0]) <= UINT8_MAX,
               "Compact scalar sensor indices must fit their existing byte");
/* The receive callback copies one fixed message; CSV and file operations stay
 * in the SD writer. Include the acquisition's session across a launch change. */
typedef struct {
  pressure_sd_batch_t batch;
  /* Route keys (see sd_route_session_locked): the launch key this capture is
   * classified against, and the receive-time route used when it is unknown. */
  uint32_t launch_key, route_key;
  uint32_t route_epoch;
  /* With SD_PRESSURE_ROUTE_CLASSIFIED, bits0-3 mark rows captured inside the
   * launch window and bits8-11 rows captured after it; other rows precede it.
   * Bit4 explicitly records an unknown capture clock; never synthesize UTC. */
  uint32_t capture_route;
} sd_pressure_message_t;
#define SD_PRESSURE_ROUTE_UNKNOWN_UTC 16U
#define SD_PRESSURE_ROUTE_CLASSIFIED 32U
#define SD_PRESSURE_MESSAGE_WORDS ((sizeof(sd_pressure_message_t) + sizeof(ULONG) - 1U) / sizeof(ULONG))
_Static_assert(SD_PRESSURE_MESSAGE_WORDS <= 16U, "ThreadX pressure message bounds");
static TX_QUEUE g_sd_pressure_queue;
/* 800 pressure samples cover3.2s of SD-card programming/flush latency. */
#define SD_PRESSURE_QUEUE_BATCHES 200U
static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];
static uint8_t g_sd_pressure_pending;
static uint64_t g_sd_pressure_pending_key;
static uint8_t g_sd_line_pending;
static uint64_t g_sd_line_pending_key;
volatile uint32_t g_sd_pressure_records_enqueued;
volatile uint32_t g_sd_pressure_records_dropped;
volatile uint32_t g_sd_pressure_records_written;
volatile uint32_t g_sd_pressure_records_flushed;

static TX_EVENT_FLAGS_GROUP g_sd_flags;
/* Numeric acquisition rows use compact batches instead of full CSV strings.
 * Reinvest that RAM in raw acquisition and pressure stall coverage. The two
 * pools gave 24 KiB to the sector-aligned log staging buffers; both still
 * cover three seconds at 2500 raw records/s and 500 rows/s on the target. */
static TX_BLOCK_POOL g_sd_line_pool, g_sd_raw_pool;
static UCHAR g_sd_line_pool_storage[49U * 1024U] SD_ALIGN_32;
static UCHAR g_sd_raw_pool_storage[321U * 1024U] SD_ALIGN_32;
/* Keep at least the acquisition configuration's original minimum reserve.
 * The fixed-block pool additionally covers measured multi-second
 * card stalls at the observed2.5ksamples/s rate when queued tails coalesce. */
_Static_assert(DAQ_CHAMBER_SCAN_MASK == 0U ||
    ((DAQ_SD_RAW_BUFFER_MS * DAQ_ADC_READ_RATE_HZ + 999U) / 1000U) * sizeof(sd_raw_adc_record_t) +
    DAQ_SD_RAW_QUEUE_DEPTH * (offsetof(sd_raw_slot_t, samples) + 16U) +
    SD_RAW_BATCH_MAX * sizeof(sd_raw_adc_record_t) <= sizeof(g_sd_raw_pool_storage),
    "Chamber sample rate exceeds the existing800ms SD raw byte reserve");
_Static_assert(sizeof(g_sd_line_pool_storage) <= 180U * 1024U, "SD telemetry RAM budget");
_Static_assert(sizeof(g_sd_raw_pool_storage) <= 340U * 1024U, "SD raw RAM budget");

static sd_optional_snapshot_t g_sd_optional_snapshots[SD_OPTIONAL_SNAPSHOT_SLOTS];
static uint32_t g_sd_optional_snapshot_used, g_sd_optional_snapshot_deferred;
volatile uint32_t g_sd_optional_snapshot_slots_used;
volatile uint32_t g_sd_optional_snapshots_completed, g_sd_optional_snapshots_deferred;
volatile uint32_t g_sd_optional_snapshot_rows_discarded, g_sd_optional_snapshots_refused;

static FX_MEDIA g_sd_media;
/* Each log stages whole SD sectors. Writes end on a sector boundary, so FileX
 * sends every full sector straight from this 32-byte aligned buffer to IDMA as
 * one multi-block command; only the bytes after the last boundary wait for the
 * next write or the once-per-second flush. */
/* One open log receives every row (raw ADC, telemetry and pressure) of one
 * route key. Two handles exist only so a transition never closes a file while
 * rows that belong to it are still queued: the regular session file and the
 * launch-window file may overlap briefly, then the superseded one closes once.
 * Neither handle rotates for calibration; only the FAT32 part limit splits. */
typedef struct {
  FX_FILE file;
  char name[64];
  _Alignas(32) UCHAR buffer[16U * 1024U];
  size_t len;
  ULONG retry_offset;
  uint64_t key;
  daq_calibration_t calibration;
  uint8_t open, retry_active, reserve_disabled;
} sd_log_t;
static sd_log_t g_sd_regular_log, g_sd_launch_log;
_Static_assert(sizeof(g_sd_regular_log.buffer) % 512U == 0U, "Log staging holds whole sectors");
/* Writer-only: newest key whose file has been finished. A finished launch
 * window never reopens; late rows join the following regular file instead. */
static uint64_t g_sd_regular_closed_key, g_sd_launch_done_key;
static UCHAR g_sd_media_cache[SD_MEDIA_CACHE_SIZE] SD_ALIGN_32;
static UCHAR g_sd_transfer_buffer[SD_TRANSFER_SECTORS * SD_SECTOR_SIZE] SD_ALIGN_32;
volatile uint32_t g_sd_ready = 0U;
/* Debugger/simulator progress: 1 card init, 2 bus setup, 3 mount,
 * 4 open log, 5 ready, 6 formatting. */
volatile uint32_t g_sd_init_stage = 0U;
static uint8_t g_sd_hardware_ready = 0U;
static uint8_t g_sd_media_mounted;
static uint32_t g_sd_mounted_cid[4], g_sd_mounted_blocks;
volatile uint32_t g_sd_card_identity_mismatches;
static uint8_t g_sd_manual_no_format;
static volatile uint32_t g_sd_retry_pending_token, g_sd_retry_active_token;
static uint32_t g_sd_retry_accepted_token, g_sd_retry_completed_token;
static uint32_t g_sd_retry_recent[8], g_sd_retry_recent_next;
static uint64_t g_sd_last_sector_write_ms, g_sd_last_media_flush_ms;
static uint8_t g_sd_write_progress_valid, g_sd_flush_progress_valid;
static daq_calibration_t g_sd_calibration = {1.0f, 0.0f, 1.0f, 0.0f, {0.0f, 1.0f}};
static volatile uint32_t g_sd_calibration_generation = 1U;
static uint8_t g_sd_services_initialized = 0U;
static uint64_t g_sd_session, g_sd_last_launch_session, g_sd_close_unix_ms;
static uint64_t g_sd_close_local_ms;
static uint8_t g_sd_launch_closed, g_sd_deadline_anchored, g_sd_launch_opened;
static uint8_t g_sd_launch_flush_pending;
/* Capture-cohort generations; advanced at every routing transition. */
static uint32_t g_sd_route_epoch;
static void sd_route_epoch_advance(void);

/* The launch file holds exactly T0..T+20 s, where T0 = deadline - 20 s. */
#define SD_LAUNCH_WINDOW_MS 20000U
/* A superseded file stays open this long after its transition so a late VB
 * pressure batch (captured before the transition) still reaches it. */
#define SD_ROUTE_GRACE_MS 1000U
/* Route key: odd = regular session file, even = launch-window file. It starts
 * at1 (boot session) and increments when a window opens (even) and when it
 * ends or is reset (odd), so each key names exactly one file. */
static uint64_t g_sd_route_key = 1U, g_sd_route_change_local;
/* Highest key a raw row was routed to; reset when the launch clock changes. */
static uint64_t g_sd_raw_key_floor;

/* Caller holds the short IRQ lock; only the local extended uptime is read.
 * No router/network-clock lock is acquired from a receive callback. */
static uint64_t sd_route_session_locked(uint64_t local)
{
  if (!g_sd_launch_closed && g_sd_deadline_anchored) {
    if (!g_sd_launch_opened && local < g_sd_close_local_ms &&
        g_sd_close_local_ms - local <= SD_LAUNCH_WINDOW_MS) {
      g_sd_launch_opened = 1U; ++g_sd_route_key; g_sd_route_change_local = local;
      sd_route_epoch_advance();
    }
    if (local >= g_sd_close_local_ms) {
      g_sd_launch_closed = 1U;
      if (g_sd_launch_opened) { ++g_sd_route_key; g_sd_route_change_local = local; }
      sd_route_epoch_advance();
    }
  }
  return g_sd_route_key;
}

/* Caller holds the IRQ lock after sd_route_session_locked. The key of the
 * current session's launch file, or0 when no window is (or will be) logged. */
static uint64_t sd_launch_key_locked(void)
{
  if (g_sd_session == 0U || !g_sd_deadline_anchored) return 0U;
  if (g_sd_launch_opened) return g_sd_launch_closed ? g_sd_route_key - 1U : g_sd_route_key;
  return g_sd_launch_closed ? 0U : g_sd_route_key + 1U;
}

/* Caller holds the IRQ lock after sd_route_session_locked. With both clocks
 * known, a row is classified by its capture UTC exactly like VB pressure, so
 * the launch file holds only rows captured in [T0, T+20 s) whatever the batch
 * or queue boundaries; otherwise the receive-time route applies. */
static uint64_t sd_capture_key_locked(uint64_t route, uint64_t capture_unix_ms)
{
  const uint64_t launch = sd_launch_key_locked();
  if (launch == 0U || g_sd_close_unix_ms < SD_LAUNCH_WINDOW_MS ||
      capture_unix_ms < UINT64_C(315532800000) || capture_unix_ms >= UINT64_C(4354819200000))
    return route;
  if (capture_unix_ms >= g_sd_close_unix_ms) return launch + 1U;
  return capture_unix_ms >= g_sd_close_unix_ms - SD_LAUNCH_WINDOW_MS ? launch : launch - 1U;
}

static uint64_t sd_run_snapshot(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint64_t result = sd_route_session_locked(daq_log_monotonic_ms());
  __set_PRIMASK(mask);
  return result;
}

static uint8_t sd_launch_finished(void)
{
  /* Map UTC once; later loss/correction cannot extend the monotonic cutoff.
   * Without a UTC mapping, ordinary logging continues until the session can
   * be dated honestly. Retained expired sessions never open a launch file. */
  uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint8_t needs_utc = g_sd_close_unix_ms != 0U && !g_sd_deadline_anchored && !g_sd_launch_closed;
  __set_PRIMASK(mask);
  const uint64_t utc = needs_utc ? daq_log_unix_ms() : 0U;
  const uint64_t local = daq_log_monotonic_ms();
  mask = __get_PRIMASK(); __disable_irq();
  if (g_sd_close_unix_ms != 0U && !g_sd_deadline_anchored && !g_sd_launch_closed && utc != 0U) {
    g_sd_close_local_ms = local + (utc < g_sd_close_unix_ms ? g_sd_close_unix_ms - utc : 0U);
    g_sd_deadline_anchored = 1U;
    sd_route_epoch_advance();
  }
  (void)sd_route_session_locked(local);
  const uint8_t finished = g_sd_launch_closed;
  __set_PRIMASK(mask);
  return finished;
}

void sd_card_set_launch_clock(uint64_t session, uint64_t close_unix_ms)
{
  const uint64_t now = session != 0U ? daq_log_unix_ms() : 0U;
  const uint64_t local = daq_log_monotonic_ms();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t prior_session = g_sd_session, prior_close = g_sd_close_unix_ms;
  const uint64_t before = sd_route_session_locked(local);
  /* A reset or newer session ends an open window: the next rows start a new
   * regular session file. A window that never opened leaves it unchanged. */
  const uint8_t ends_window = g_sd_launch_opened && !g_sd_launch_closed &&
      (session == 0U || session > g_sd_last_launch_session);
  if (ends_window) { ++g_sd_route_key; g_sd_route_change_local = local; }
  if (session == 0U) {
    g_sd_session = 0U; g_sd_close_unix_ms = 0U; g_sd_launch_opened = 0U;
    g_sd_close_local_ms = 0U; g_sd_launch_closed = 0U; g_sd_deadline_anchored = 0U;
  } else if (session > g_sd_last_launch_session) {
    g_sd_last_launch_session = session; g_sd_launch_opened = 0U;
    g_sd_session = session; g_sd_close_unix_ms = close_unix_ms;
    g_sd_deadline_anchored = now != 0U;
    g_sd_launch_closed = now != 0U && now >= close_unix_ms;
    g_sd_close_local_ms = now != 0U ? local + (now < close_unix_ms ? close_unix_ms - now : 0U) : 0U;
  } else if (g_sd_session == session && !g_sd_launch_closed && close_unix_ms != g_sd_close_unix_ms) {
    /* Receiver revision ordering has established a genuinely newer T0. Move
     * the existing monotonic mapping by the absolute delta, never by a fresh
     * wall/UTC reading. A closed/tombstoned session never enters this branch. */
    const uint64_t prior = g_sd_close_unix_ms;
    g_sd_close_unix_ms = close_unix_ms;
    if (g_sd_deadline_anchored) {
      if (close_unix_ms < prior) {
        const uint64_t earlier = prior - close_unix_ms;
        g_sd_close_local_ms = g_sd_close_local_ms > earlier ? g_sd_close_local_ms - earlier : 0U;
      } else {
        const uint64_t later = close_unix_ms - prior;
        if (UINT64_MAX - g_sd_close_local_ms < later) g_sd_launch_closed = 1U;
        else g_sd_close_local_ms += later;
      }
    } else if (now != 0U) {
      g_sd_close_local_ms = local + (now < close_unix_ms ? close_unix_ms - now : 0U);
      g_sd_deadline_anchored = 1U;
    }
  }
  const uint64_t after = sd_route_session_locked(local);
  if (before != after) sd_route_epoch_advance();
  /* A new, reset or revised window may legitimately route raw rows lower. */
  if (session != prior_session || close_unix_ms != prior_close) g_sd_raw_key_floor = 0U;
  __set_PRIMASK(mask);
}

volatile uint32_t g_sd_line_drop_count = 0U;
volatile uint32_t g_sd_line_slots_used = 0U;
volatile uint32_t g_sd_line_slots_peak = 0U;
volatile uint32_t g_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_sd_raw_slots_used = 0U;
volatile uint32_t g_sd_raw_slots_peak = 0U;
volatile uint32_t g_sd_max_service_ms = 0U;
volatile uint32_t g_sd_max_read_ms = 0U;
volatile uint32_t g_sd_max_write_ms = 0U;
volatile uint32_t g_sd_max_raw_service_ms = 0U;
volatile uint32_t g_sd_max_telemetry_service_ms = 0U;
volatile uint32_t g_sd_max_flush_ms = 0U;
volatile uint32_t g_sd_write_error_count = 0U;
volatile uint32_t g_sd_flush_count = 0U;
volatile uint32_t g_sd_init_failures = 0U;
volatile uint32_t g_sd_retry_count = 0U;
volatile uint32_t g_sd_warning_publish_count = 0U;
volatile uint32_t g_sd_raw_records_written_count = 0U;
volatile uint32_t g_sd_raw_records_enqueued_count;
volatile uint32_t g_sd_raw_records_dropped_count;
volatile uint32_t g_sd_raw_essential_records_dropped;
volatile uint32_t g_sd_raw_optional_records_dropped;
volatile uint32_t g_sd_optional_telemetry_dropped;
volatile uint32_t g_sd_essential_telemetry_dropped;
volatile uint32_t g_sd_raw_records_coalesced;
volatile uint32_t g_sd_scalar_records_enqueued, g_sd_scalar_records_written, g_sd_scalar_records_coalesced;
volatile uint32_t g_sd_optional_scalar_records_enqueued, g_sd_optional_scalar_records_written, g_sd_optional_scalar_records_coalesced;
volatile uint32_t g_sd_preallocation_attempts;
volatile uint32_t g_sd_preallocation_fallbacks;
volatile uint32_t g_sd_preallocation_errors;
volatile uint32_t g_sd_max_preallocate_ms;
volatile uint32_t g_sd_raw_records_io_lost_count;
volatile uint32_t g_sd_raw_records_flushed_count;
volatile uint32_t g_sd_csv_rows_written_count = 0U;

static uint32_t sd_calibration_snapshot(daq_calibration_t *calibration)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  *calibration = g_sd_calibration;
  const uint32_t generation = g_sd_calibration_generation;
  __set_PRIMASK(saved);
  return generation;
}

/* All producers, reservations and queued/active consumers hold a reference.
 * A seal moves current references into the old cohort in one short RAM step.
 * Neither an empty queue nor a copied boolean can prove these references gone. */
static struct { uint32_t current, old, cutoff, flags; float old_tuple[4]; } g_sd_capture;
#define SD_CAPTURE_SEALED 1U
#define SD_CAPTURE_FAULT 2U
_Static_assert(sizeof(g_sd_capture) == 32U, "Bounded cohort metadata");
static void sd_route_epoch_advance(void)
{
  /* Keep ordinary routing/data behavior. Exhaustion disables only thermal
   * admission; it must never wrap into a trusted cutover owner identity. */
  if (g_sd_route_epoch == UINT32_MAX) g_sd_capture.flags |= SD_CAPTURE_FAULT;
  ++g_sd_route_epoch;
}
static uint8_t sd_capture_old_locked(sd_capture_t capture)
{
  return (g_sd_capture.flags & SD_CAPTURE_SEALED) != 0U &&
      capture < g_sd_capture.cutoff;
}
static void sd_capture_retain_locked(sd_capture_t capture)
{
  uint32_t *count = sd_capture_old_locked(capture) ? &g_sd_capture.old : &g_sd_capture.current;
  if (*count == UINT32_MAX || (sd_capture_old_locked(capture) && *count == 0U))
    g_sd_capture.flags |= SD_CAPTURE_FAULT;
  if (*count != UINT32_MAX) ++*count;
}
void sd_card_capture_retain(sd_capture_t capture)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  sd_capture_retain_locked(capture); __set_PRIMASK(mask);
}
void sd_card_capture_release(sd_capture_t capture)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  uint32_t *count = sd_capture_old_locked(capture) ? &g_sd_capture.old : &g_sd_capture.current;
  if (*count == 0U) g_sd_capture.flags |= SD_CAPTURE_FAULT;
  else --*count;
  __set_PRIMASK(mask);
}
void sd_card_capture_fault(sd_capture_t capture)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (sd_capture_old_locked(capture)) g_sd_capture.flags |= SD_CAPTURE_FAULT;
  __set_PRIMASK(mask);
}
sd_capture_t sd_card_capture_begin(const daq_calibration_t *provided, daq_calibration_t *captured)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const daq_calibration_t actual = daq_calibration_current();
  *captured = provided ? *provided : actual;
  const sd_capture_t capture = g_sd_route_epoch;
  /* Legacy callers without an original lease retain their data, but may not
   * make a sealed transaction look complete with an untracked old snapshot. */
  if ((g_sd_capture.flags & SD_CAPTURE_SEALED) &&
      memcmp(captured, &actual, sizeof(actual)) != 0)
    g_sd_capture.flags |= SD_CAPTURE_FAULT;
  sd_capture_retain_locked(capture);
  __set_PRIMASK(mask); return capture;
}
bool sd_card_thermal_seal(const float old_tuple[4])
{
  if (!old_tuple || __get_PRIMASK() == 0U || g_sd_capture.flags != 0U || g_sd_route_epoch == UINT32_MAX)
    return false;
  sd_route_epoch_advance(); g_sd_capture.cutoff = g_sd_route_epoch;
  g_sd_capture.old = g_sd_capture.current; g_sd_capture.current = 0U;
  memcpy(g_sd_capture.old_tuple, old_tuple, sizeof(g_sd_capture.old_tuple));
  g_sd_capture.flags = SD_CAPTURE_SEALED;
  return true;
}
bool sd_card_thermal_publish_metadata(const float expected[4], const float desired[4])
{
  /* Unlike the legacy whole-calibration setter, this cannot finish a launch,
   * clear its clock/session or perform any SDK/storage work. The service must
   * separately hold the current safe-apply authority; these bytes are not it. */
  if (!expected || !desired || __get_PRIMASK() == 0U ||
      g_sd_capture.flags != SD_CAPTURE_SEALED || g_sd_calibration_generation == UINT32_MAX ||
      memcmp(g_sd_calibration.thermal, expected, sizeof(g_sd_calibration.thermal)) != 0)
    return false;
  if (memcmp(expected, desired, sizeof(g_sd_calibration.thermal)) != 0) {
    memcpy(g_sd_calibration.thermal, desired, sizeof(g_sd_calibration.thermal));
    ++g_sd_calibration_generation;
  }
  return true;
}
bool sd_card_thermal_old_drained(void)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const bool ready = g_sd_capture.flags == SD_CAPTURE_SEALED && g_sd_capture.old == 0U;
  __set_PRIMASK(mask); return ready;
}
void sd_card_thermal_finish(void)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (g_sd_capture.flags == SD_CAPTURE_SEALED && g_sd_capture.old == 0U)
    g_sd_capture.flags = 0U;
  else g_sd_capture.flags |= SD_CAPTURE_FAULT;
  __set_PRIMASK(mask);
}
void sd_card_thermal_cohort_snapshot(uint32_t words[4])
{
  if (!words) return;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  words[0]=g_sd_capture.current; words[1]=g_sd_capture.old;
  words[2]=g_sd_capture.cutoff; words[3]=g_sd_capture.flags;
  __set_PRIMASK(mask);
}

/* Only the storage worker calls these adapters. IDMA owns the aligned
 * bounce buffer until completion or confirmed peripheral quiescence. */
static UINT sd_hal_read(UCHAR *destination, ULONG sector, ULONG sector_count)
{
  const uint32_t started = HAL_GetTick();
  while (sector_count != 0U) {
    const ULONG count = sector_count > SD_TRANSFER_SECTORS ? SD_TRANSFER_SECTORS : sector_count;
    if (sd_storage_io_read(g_sd_transfer_buffer, sector, count) != FX_SUCCESS) {
      if (sd_storage_io_requires_reinit()) g_sd_ready = 0U;
      return FX_IO_ERROR;
    }
    memcpy(destination, g_sd_transfer_buffer, (size_t)count * SD_SECTOR_SIZE);
    destination += count * SD_SECTOR_SIZE; sector += count; sector_count -= count;
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_read_ms) g_sd_max_read_ms = elapsed;
  return FX_SUCCESS;
}

/* FileX passes whole sectors straight from an aligned log buffer or its own
 * aligned cache: IDMA reads those in place, up to one multi-block command per
 * SD_STORAGE_IO_MAX_SECTORS. Any other source goes through the bounce buffer. */
static UINT sd_hal_write(const UCHAR *source, ULONG sector, ULONG sector_count)
{
  const uint32_t started = HAL_GetTick();
  while (sector_count != 0U) {
    const ULONG direct = sector_count > SD_STORAGE_IO_MAX_SECTORS ? SD_STORAGE_IO_MAX_SECTORS : sector_count;
    const uint8_t in_place = sd_storage_io_dma_capable(source, direct);
    const ULONG count = in_place ? direct :
        (sector_count > SD_TRANSFER_SECTORS ? SD_TRANSFER_SECTORS : sector_count);
    if (!in_place) memcpy(g_sd_transfer_buffer, source, (size_t)count * SD_SECTOR_SIZE);
    if (sd_storage_io_write(in_place ? source : g_sd_transfer_buffer, sector, count) != FX_SUCCESS) {
      if (sd_storage_io_requires_reinit()) g_sd_ready = 0U;
      return FX_IO_ERROR;
    }
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    g_sd_last_sector_write_ms = board_uptime_ticks(); g_sd_write_progress_valid = 1U;
    __set_PRIMASK(saved);
    source += count * SD_SECTOR_SIZE; sector += count; sector_count -= count;
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_write_ms) g_sd_max_write_ms = elapsed;
  return FX_SUCCESS;
}

static VOID sd_filex_driver(FX_MEDIA *media)
{
  ULONG sector;
  UINT status = FX_SUCCESS;

  switch (media->fx_media_driver_request)
  {
    case FX_DRIVER_INIT:
      media->fx_media_driver_write_protect = FX_FALSE;
      media->fx_media_driver_free_sector_update = FX_FALSE;
      status = (g_sd_hardware_ready != 0U) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_UNINIT:
    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
    case FX_DRIVER_RELEASE_SECTORS:
      status = FX_SUCCESS;
      break;

    case FX_DRIVER_BOOT_READ:
      status = sd_hal_read(media->fx_media_driver_buffer, 0U,
                           media->fx_media_driver_sectors);
      if (status == FX_SUCCESS)
      {
        ULONG partition_start = 0U;
        ULONG partition_size = 0U;
        if ((_fx_partition_offset_calculate(media->fx_media_driver_buffer, 0U,
                                            &partition_start,
                                            &partition_size) == FX_SUCCESS) &&
            (partition_start != 0U))
        {
          status = sd_hal_read(media->fx_media_driver_buffer, partition_start,
                               media->fx_media_driver_sectors);
        }
      }
      break;

    case FX_DRIVER_BOOT_WRITE:
      status = sd_hal_write(media->fx_media_driver_buffer, media->fx_media_hidden_sectors,
                            media->fx_media_driver_sectors);
      break;

    case FX_DRIVER_READ:
      sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
      status = sd_hal_read(media->fx_media_driver_buffer, sector,
                           media->fx_media_driver_sectors);
      break;

    case FX_DRIVER_WRITE:
      sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
      status = sd_hal_write(media->fx_media_driver_buffer, sector,
                            media->fx_media_driver_sectors);
      break;

    default:
      status = FX_IO_ERROR;
      break;
  }

  media->fx_media_driver_status = status;
}

/* Reserve FAT chains ahead of writes without extending logical CSV length.
 * FileX allocate() appends clusters; its truncate_release() counterpart must
 * release the unused tail on a normal close. An unclean power loss may retain
 * up to2.25MiB of reserved tail per open log, plus cluster rounding, until
 * filesystem repair/reclamation. */
#define SD_FILE_RESERVE_BYTES (2UL * 1024UL * 1024UL)
#define SD_FILE_RESERVE_LOW_WATER (256UL * 1024UL)
#define SD_OPTIONAL_RAW_BACKLOG_LIMIT 256U
/* Optional rows are shed only after the writer has been stalled for the SD
 * specification's worst-case busy time (500 ms write timeout for SDXC; 250 ms
 * for SDSC/SDHC) at the full ADC rate. File creation, preallocation, close and
 * FAT updates therefore never shed a row; the pool guard still reserves a
 * quarter of the raw pool for essential channels. */
#define SD_OPTIONAL_STALL_MS 500U
#define SD_OPTIONAL_RAW_ADMIT_LIMIT ((DAQ_ADC_READ_RATE_HZ * SD_OPTIONAL_STALL_MS + 999U) / 1000U)
#if DAQ_SD_PREALLOCATE_ENABLE && (defined(FX_UPDATE_FILE_SIZE_ON_ALLOCATE) || defined(FX_ENABLE_FAULT_TOLERANT))
#error "Review SD preallocation: this FileX mode extends logical file size on allocate"
#endif
static uint32_t sd_raw_backlog_records(void)
{
  return g_sd_raw_records_enqueued_count - g_sd_raw_records_written_count -
      g_sd_raw_records_io_lost_count;
}

static UINT sd_prepare_file_space(sd_log_t *log, size_t next_bytes)
{
  if (!DAQ_SD_PREALLOCATE_ENABLE) return FX_SUCCESS;
  FX_FILE *file = &log->file;
  uint8_t *disabled = &log->reserve_disabled;
  if (*disabled) return FX_SUCCESS;
  const uint64_t available = file->fx_file_current_available_size;
  const uint64_t offset = file->fx_file_current_file_offset;
  const uint64_t tail = available > offset ? available - offset : 0U;
  if (tail >= (uint64_t)next_bytes + SD_FILE_RESERVE_LOW_WATER) return FX_SUCCESS;
  /* Top up while the protected queue is nearly empty, rather than adding
   * metadata I/O to a backlog. A write that needs space cannot be deferred. */
  if (tail >= next_bytes && sd_raw_backlog_records() > SD_OPTIONAL_RAW_BACKLOG_LIMIT)
    return FX_SUCCESS;
  if (available >= SD_LOG_PART_LIMIT_BYTES) return FX_SUCCESS;
  const uint64_t room = SD_LOG_PART_LIMIT_BYTES - available;
  const ULONG reserve = (ULONG)(room < SD_FILE_RESERVE_BYTES ? room : SD_FILE_RESERVE_BYTES);
  const uint32_t started = HAL_GetTick();
  ++g_sd_preallocation_attempts;
  UINT status = fx_file_allocate(file, reserve);
  /* Allocate may advance its physical cursor at a cluster boundary even
   * when an earlier reserve still has space. Re-seek through zero so the
   * next write uses the original logical offset in the extended FAT chain. */
  if (status == FX_SUCCESS && offset != 0U) {
    status = fx_file_seek(file, 0U);
    if (status == FX_SUCCESS) status = fx_file_seek(file, (ULONG)offset);
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_preallocate_ms) g_sd_max_preallocate_ms = elapsed;
  if (status == FX_NO_MORE_SPACE) {
    /* A fragmented card may not have a consecutive reserve. Preserve ordinary
     * FileX writes; do not retry this performance hint on every4KiB flush. */
    *disabled = 1U; ++g_sd_preallocation_fallbacks;
    return FX_SUCCESS;
  }
  if (status != FX_SUCCESS) ++g_sd_preallocation_errors;
  return status;
}

static UINT sd_release_file_space(FX_FILE *file)
{
  if (file->fx_file_current_available_size <= file->fx_file_current_file_size)
    return FX_SUCCESS;
  /* Ordinary truncate() and close() leave unused allocation attached. */
  return fx_file_truncate_release(file, (ULONG)file->fx_file_current_file_size);
}

static UINT sd_media_flush(void)
{
  const UINT status = fx_media_flush(&g_sd_media);
  if (status == FX_SUCCESS) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    g_sd_last_media_flush_ms = board_uptime_ticks(); g_sd_flush_progress_valid = 1U;
    __set_PRIMASK(saved);
  }
  return status;
}

/* Writer-only FileX calls. No current-directory mutation, historical moves or
 * deletions. An existing regular file cannot masquerade as a log directory. */
static UINT sd_create_log_file(char *filename, size_t filename_size,
                               const char *prefix, uint64_t utc_ms, uint64_t uptime_ms)
{
  if (filename == NULL || filename_size == 0U || prefix == NULL) return FX_PTR_ERROR;
  filename[0] = '\0';
  const char *directory = sd_log_directory(prefix, utc_ms);
  UINT status = fx_directory_create(&g_sd_media, (CHAR *)directory);
  if (status == FX_ALREADY_CREATED) {
    UINT attributes = 0U;
    status = fx_directory_attributes_read(&g_sd_media, (CHAR *)directory, &attributes);
    if (status == FX_SUCCESS && (attributes & FX_DIRECTORY) == 0U) return FX_DIRECTORY_ERROR;
  }
  if (status != FX_SUCCESS) return status;
  sd_calendar_t calendar;
  const uint64_t stamp = sd_calendar_from_unix_ms(utc_ms, &calendar) ? utc_ms : uptime_ms;
  char stamp_text[21];
  sd_format_u64(stamp_text, stamp);
  for (uint32_t suffix = 0U; suffix < 1000U; ++suffix) {
    const int length = snprintf(filename, filename_size, "%s/%s_%s_%03lu.CSV",
                                directory, prefix, stamp_text, (unsigned long)suffix);
    if (length < 0 || (size_t)length >= filename_size) {
      filename[0] = '\0'; return FX_INVALID_NAME;
    }
    status = fx_file_create(&g_sd_media, filename);
    if (status == FX_SUCCESS) return FX_SUCCESS;
    filename[0] = '\0';
    if (status != FX_ALREADY_CREATED) return status;
  }
  return FX_NO_MORE_SPACE;
}
_Static_assert(sizeof("/NO_UTC/DAQ_LAUNCH_18446744073709551615_999.CSV") <= 64U,
               "Log directory plus longest basename must fit FileX and handle storage");

static void sd_update_filesystem_clock(void)
{
  sd_calendar_t now;
  if (sd_calendar_from_unix_ms(daq_log_unix_ms(), &now))
  {
    (void)fx_system_date_set(now.year, now.month, now.day);
    (void)fx_system_time_set(now.hour, now.minute, now.second);
  }
}

static UINT sd_close_file(sd_log_t *log)
{
  sd_update_filesystem_clock();
  UINT status = sd_release_file_space(&log->file);
  if (status != FX_SUCCESS) return status;
  status = fx_file_close(&log->file);
  sd_calendar_t now;
  if (status == FX_SUCCESS && sd_calendar_from_unix_ms(daq_log_unix_ms(), &now) &&
      fx_file_date_time_set(&g_sd_media, log->name, now.year, now.month,
                           now.day, now.hour, now.minute, now.second) != FX_SUCCESS)
    g_sd_write_error_count++;
  /* Timestamp failure must not leave a successfully closed handle marked open. */
  if (status == FX_SUCCESS) log->open = 0U;
  return status;
}

static int sd_format_calibration_line(char *line, size_t size, const daq_calibration_t *calibration)
{
  char coefficients[17][24];
  sd_format_float(coefficients[0], calibration->kg1000_slope);
  sd_format_float(coefficients[1], calibration->kg1000_intercept);
  sd_format_float(coefficients[2], calibration->iadc_slope);
  sd_format_float(coefficients[3], calibration->iadc_intercept);
  for (unsigned i = 0; i < 7; ++i)
    sd_format_float(coefficients[4 + i], calibration->kg50[i]);
  for (unsigned i = 0; i < 4; ++i)
    sd_format_float(coefficients[11 + i], calibration->thermal[i]);
  sd_format_float(coefficients[15], calibration->filter_tau_ms[0]);
  sd_format_float(coefficients[16], calibration->filter_tau_ms[1]);
  const int len = snprintf(
      line, size,
      "# calibration,kg1000_slope=%s,kg1000_intercept=%s,iadc_slope=%s,iadc_intercept=%s,kg50_c0=%s,kg50_c1=%s,kg50_c2=%s,kg50_c3=%s,kg50_c4=%s,kg50_x0=%s,kg50_tare=%s,kg1000_temp_ref=%s,kg1000_raw_per_c=%s,kg50_temp_ref=%s,kg50_raw_per_c=%s,kg1000_filter_tau_ms=%s,kg50_filter_tau_ms=%s\r\n",
      coefficients[0], coefficients[1], coefficients[2], coefficients[3],
      coefficients[4], coefficients[5], coefficients[6], coefficients[7],
      coefficients[8], coefficients[9], coefficients[10],
      coefficients[11], coefficients[12], coefficients[13], coefficients[14], coefficients[15], coefficients[16]);
  return len > 0 && (size_t)len < size ? len : -1;
}

/* Regular files go to /UTC when UTC is known at creation, else /NO_UTC, and
 * stay there for their whole session; launch-window files go to /LAUNCH. */
static UINT sd_open_log(sd_log_t *log, uint64_t key, const daq_calibration_t *calibration)
{
  sd_update_filesystem_clock();
  const uint64_t stamp = daq_log_unix_ms();
  UINT status = sd_create_log_file(log->name, sizeof(log->name), log == &g_sd_launch_log ? "DAQ_LAUNCH" : "DAQ",
                                   stamp, daq_log_monotonic_ms());
  if (status != FX_SUCCESS) return status;

  status = fx_file_open(&g_sd_media, &log->file, log->name, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    return status;
  }

  log->reserve_disabled = 0U; log->len = 0U; log->retry_active = 0U;
  static const char header[] =
      SD_CSV_HEADER
      "# channel=valve_pressure_psi,source_node=VB,datatype=VALVE_PRESSURE_SD_V1,raw_units=ADC_code,converted_units=psi,capture_period_ms=4,sequence_scope=VB_pressure,time_source=network_when_VB_UTC_known_else_local_unknown_UTC\r\n";
  status = sd_prepare_file_space(log, sizeof(header) - 1U);
  if (status == FX_SUCCESS)
    status = fx_file_write(&log->file, (VOID *)header, sizeof(header) - 1U);
  if (status == FX_SUCCESS)
  {
    char calibration_line[768];
    const int len = sd_format_calibration_line(calibration_line, sizeof(calibration_line), calibration);
    status = len > 0 ? fx_file_write(&log->file, calibration_line, (ULONG)len) : FX_IO_ERROR;
    if (status == FX_SUCCESS) {
      const int pressure_len = snprintf(calibration_line, sizeof(calibration_line),
          "# chamber_pressure,p4_adc_channel=%d,model=OMEGA_PX309-1KG5V,raw_units=connector_V,converted_units=psig,divider_ratio=11,zero_V=0,span_V=5,span_psig=1000,noise_floor_millipsi=%u,noise_floor_rule=finite_raw_psig<floor_millipsi/1000:0,noise_floor_zero=disabled,raw_voltage=unchanged,nonfinite=invalid,overflow=preserved\r\n",
          DAQ_CHAMBER_ADC_CHANNEL, (unsigned)DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI);
      status = pressure_len > 0 && (size_t)pressure_len < sizeof(calibration_line)
          ? fx_file_write(&log->file, calibration_line, (ULONG)pressure_len) : FX_IO_ERROR;
    }
  }
  if (status == FX_SUCCESS)
  {
    status = sd_media_flush();
  }
  if (status != FX_SUCCESS) {
    (void)sd_release_file_space(&log->file);
    (void)fx_file_close(&log->file);
    return status;
  }
  log->open = 1U; log->key = key; log->calibration = *calibration;
  return FX_SUCCESS;
}

/* A failed write keeps its bytes and retries from the same file offset. */
static UINT sd_log_flush(sd_log_t *log)
{
  if (log->len == 0U) return FX_SUCCESS;
  if (!log->retry_active) { log->retry_offset = log->file.fx_file_current_file_offset; log->retry_active = 1U; }
  UINT status = sd_prepare_file_space(log, log->len);
  if (status == FX_SUCCESS) status = fx_file_seek(&log->file, log->retry_offset);
  if (status == FX_SUCCESS)
    status = fx_file_write(&log->file, log->buffer, (ULONG)log->len);
  if (status != FX_SUCCESS) { g_sd_write_error_count++; return status; }
  log->retry_active = 0U;
  log->len = 0U;
  return FX_SUCCESS;
}

/* Throughput flush: write the first `bytes` staged bytes and move the rest,
 * unchanged, to the front of the buffer. A failure keeps everything. */
static UINT sd_log_flush_prefix(sd_log_t *log, size_t bytes)
{
  const size_t total = log->len;
  log->len = bytes;
  const UINT status = sd_log_flush(log);
  if (status != FX_SUCCESS) { log->len = total; return status; }
  log->len = total - bytes;
  if (log->len != 0U) memmove(log->buffer, log->buffer + bytes, log->len);
  return FX_SUCCESS;
}

/* Write up to the last sector boundary and keep the partial sector staged.
 * After a durable flush (or the header) left the file offset mid-sector,
 * first complete that one sector on its own; the remaining bytes then start
 * at the aligned buffer front, so FileX hands whole sectors to IDMA in place. */
#define SD_LOG_SECTOR_BYTES 512U
static UINT sd_log_flush_sectors(sd_log_t *log)
{
  uint64_t offset = log->retry_active ? log->retry_offset : log->file.fx_file_current_file_offset;
  const size_t head = (size_t)((SD_LOG_SECTOR_BYTES - offset % SD_LOG_SECTOR_BYTES) % SD_LOG_SECTOR_BYTES);
  if (head != 0U && log->len > head + SD_LOG_SECTOR_BYTES) {
    const UINT status = sd_log_flush_prefix(log, head);
    if (status != FX_SUCCESS) return status;
    offset += head;
  }
  const size_t tail = (size_t)((offset + log->len) % SD_LOG_SECTOR_BYTES);
  if (log->len <= tail) return sd_log_flush(log);
  return sd_log_flush_prefix(log, log->len - tail);
}

static UINT sd_log_write(sd_log_t *log, const void *data, size_t len)
{
  if (log->open == 0U || data == NULL || len == 0U || len > sizeof(log->buffer) - SD_LOG_SECTOR_BYTES)
    return FX_PTR_ERROR;
  if (len > sizeof(log->buffer) - log->len && sd_log_flush_sectors(log) != FX_SUCCESS)
    return FX_IO_ERROR;
  if (len > sizeof(log->buffer) - log->len && sd_log_flush(log) != FX_SUCCESS)
    return FX_IO_ERROR;
  memcpy(log->buffer + log->len, data, len);
  log->len += len;
  return FX_SUCCESS;
}

/* A calibration change is recorded in-stream; it never starts a new file. */
static UINT sd_log_note_calibration(sd_log_t *log, const daq_calibration_t *calibration)
{
  if (memcmp(&log->calibration, calibration, sizeof(*calibration)) == 0) return FX_SUCCESS;
  char line[768];
  const int len = sd_format_calibration_line(line, sizeof(line), calibration);
  if (len <= 0) return FX_IO_ERROR;
  if (len > (int)sizeof(log->buffer)) return FX_IO_ERROR;
  const UINT status = sd_log_write(log, line, (size_t)len);
  if (status == FX_SUCCESS) log->calibration = *calibration;
  return status;
}

/* Flush, close and retire one key's file. Retries keep the open handle. */
static UINT sd_log_finish(sd_log_t *log)
{
  if (!log->open) return FX_SUCCESS;
  if (sd_log_flush(log) != FX_SUCCESS || sd_close_file(log) != FX_SUCCESS) return FX_IO_ERROR;
  if (log == &g_sd_launch_log) { if (log->key > g_sd_launch_done_key) g_sd_launch_done_key = log->key; }
  else if (log->key > g_sd_regular_closed_key) g_sd_regular_closed_key = log->key;
  g_sd_launch_flush_pending = 1U;
  return FX_SUCCESS;
}

/* Select (opening once if needed) the file for one row's route key. Every row
 * of a key goes to one file; only the FAT32 part limit splits it. Rows that
 * arrive after their file was finished go to the newest file instead of
 * reopening it, so no key ever owns two files and no row is discarded. */
static sd_log_t *sd_log_for_key(uint64_t key, const daq_calibration_t *calibration, uint64_t reserve)
{
  sd_log_t *log;
  if ((key & 1U) == 0U) {
    log = &g_sd_launch_log;
    if (!(log->open && log->key == key) && key <= g_sd_launch_done_key) {
      key += 1U; log = &g_sd_regular_log;            /* Late launch-window row. */
    } else if (log->open && log->key != key && sd_log_finish(log) != FX_SUCCESS) {
      return NULL;
    }
  } else {
    log = &g_sd_regular_log;
  }
  if (log == &g_sd_regular_log) {
    if (log->open && log->key < key) {
      if (sd_log_finish(log) != FX_SUCCESS) return NULL;
    } else if (!log->open && key <= g_sd_regular_closed_key && g_sd_launch_log.open) {
      log = &g_sd_launch_log;                         /* Late pre-window row. */
    }
  }
  if (log->open && sd_log_part_needs_rotation((uint64_t)log->file.fx_file_current_file_size + log->len, reserve)) {
    /* FAT32 limit: continue the same key in a new part. */
    if (sd_log_flush(log) != FX_SUCCESS || sd_media_flush() != FX_SUCCESS || sd_close_file(log) != FX_SUCCESS)
      return NULL;
    key = log->key;
  }
  if (!log->open && sd_open_log(log, key, calibration) != FX_SUCCESS) return NULL;
  return sd_log_note_calibration(log, calibration) == FX_SUCCESS ? log : NULL;
}

static UINT sd_write_telemetry_row(const sd_line_slot_t *slot)
{
  sd_log_t *log = sd_log_for_key(slot->session, &slot->calibration, slot->len);
  return log ? sd_log_write(log, slot->line, slot->len) : FX_IO_ERROR;
}

static UINT sd_marker_exists(void)
{
  FX_FILE marker;
  const UINT status = fx_file_open(&g_sd_media, &marker, SD_PROVISION_MARKER,
                                   FX_OPEN_FOR_READ);
  if (status == FX_SUCCESS)
  {
    (void)fx_file_close(&marker);
    return FX_SUCCESS;
  }
  return status;
}

/* FileX formats FAT tables one sector at a time. Coalesce those writes only
 * during provisioning; normal FileX writes retain their synchronous semantics.
 * Copy the buffer because FileX reuses it immediately after each callback. */
static UCHAR g_sd_format_buffer[SD_TRANSFER_SECTORS * SD_SECTOR_SIZE];
static ULONG g_sd_format_start;
static ULONG g_sd_format_count;
static UINT g_sd_format_error;
volatile uint32_t g_sd_format_sectors_written;
volatile uint32_t g_sd_mount_status;
volatile uint32_t g_sd_format_status;

static UINT sd_format_flush(void)
{
  if (g_sd_format_error != FX_SUCCESS) return g_sd_format_error;
  if (g_sd_format_count == 0U) return FX_SUCCESS;
  g_sd_format_error = sd_hal_write(g_sd_format_buffer, g_sd_format_start,
                                  g_sd_format_count);
  if (g_sd_format_error == FX_SUCCESS)
  {
    g_sd_format_sectors_written += g_sd_format_count;
    g_sd_format_count = 0U;
  }
  return g_sd_format_error;
}

static VOID sd_format_driver(FX_MEDIA *media)
{
  if (media->fx_media_driver_request == FX_DRIVER_INIT)
  {
    g_sd_format_count = 0U;
    g_sd_format_error = FX_SUCCESS;
    g_sd_format_sectors_written = 0U;
    sd_filex_driver(media);
    return;
  }
  if (media->fx_media_driver_request == FX_DRIVER_WRITE)
  {
    ULONG sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
    const UCHAR *source = media->fx_media_driver_buffer;
    for (ULONG i = 0U; i < media->fx_media_driver_sectors; ++i, ++sector)
    {
      if (g_sd_format_count != 0U &&
          sector != g_sd_format_start + g_sd_format_count)
        (void)sd_format_flush();
      if (g_sd_format_error != FX_SUCCESS) break;
      if (g_sd_format_count == 0U) g_sd_format_start = sector;
      memcpy(g_sd_format_buffer + g_sd_format_count * SD_SECTOR_SIZE,
             source + i * SD_SECTOR_SIZE, SD_SECTOR_SIZE);
      if (++g_sd_format_count == SD_TRANSFER_SECTORS) (void)sd_format_flush();
    }
    media->fx_media_driver_status = g_sd_format_error;
    return;
  }
  /* Boot writes, reads, flush and uninit must observe all buffered FAT writes. */
  media->fx_media_driver_status = sd_format_flush();
  if (media->fx_media_driver_status == FX_SUCCESS) sd_filex_driver(media);
}

static UINT sd_format_and_mark(void)
{
  HAL_SD_CardInfoTypeDef card_info = {0};
  FX_FILE marker;
  static const char marker_contents[] = "SEDS DAQ FAT volume v1\r\n";

  if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
  {
    return FX_IO_ERROR;
  }

  g_sd_init_stage = 6U;
  UINT status = fx_media_format(&g_sd_media,
                                sd_format_driver,
                                FX_NULL,
                                g_sd_media_cache,
                                sizeof(g_sd_media_cache),
                                "SEDS_DAQ",
                                2U,
                                512U,
                                0U,
                                card_info.LogBlockNbr,
                                SD_SECTOR_SIZE,
                                64U,
                                255U,
                                63U);
  g_sd_format_status = status;
  if (status != FX_SUCCESS)
  {
    return status;
  }
  g_sd_init_stage = 3U;
  status = fx_media_open(&g_sd_media, "DAQ SD", sd_filex_driver, FX_NULL,
                         g_sd_media_cache, sizeof(g_sd_media_cache));
  if (status != FX_SUCCESS)
  {
    return status;
  }
  /* Format supplies the boot-record label; also create the directory label
   * used by desktop volume browsers before marking provisioning complete. */
  status = fx_media_volume_set(&g_sd_media, "SEDS_DAQ");
  if (status != FX_SUCCESS)
  {
    (void)fx_media_close(&g_sd_media);
    return status;
  }
  status = fx_file_create(&g_sd_media, SD_PROVISION_MARKER);
  if ((status != FX_SUCCESS) && (status != FX_ALREADY_CREATED))
  {
    return status;
  }
  status = fx_file_open(&g_sd_media, &marker, SD_PROVISION_MARKER,
                        FX_OPEN_FOR_WRITE);
  if (status == FX_SUCCESS)
  {
    status = fx_file_write(&marker, (VOID *)marker_contents,
                           sizeof(marker_contents) - 1U);
    (void)fx_file_close(&marker);
  }
  if (status == FX_SUCCESS)
  {
    status = sd_media_flush();
  }
  return status;
}

static UINT sd_mount_or_provision(void)
{
  UINT status = fx_media_open(&g_sd_media, "DAQ SD", sd_filex_driver, FX_NULL,
                              g_sd_media_cache, sizeof(g_sd_media_cache));
  g_sd_mount_status = status;
  /* A transport fault is not an unformatted card. Never turn a failed DMA
   * read into filesystem formatting. The worker retries initialization. */
  /* FileX wraps transport failures as FX_BOOT_ERROR/FX_FAT_READ_ERROR.
   * Only a successfully read but invalid filesystem permits provisioning. */
  if (status != FX_SUCCESS && status != FX_MEDIA_INVALID) return status;
  const UINT marker_status = status == FX_SUCCESS ? sd_marker_exists() : FX_NOT_FOUND;
  if (status == FX_SUCCESS && marker_status != FX_SUCCESS && marker_status != FX_NOT_FOUND) {
    (void)fx_media_close(&g_sd_media);
    return marker_status;
  }
  if ((status == FX_SUCCESS) && (marker_status == FX_SUCCESS))
  {
    /* Upgrade cards provisioned before the directory label was added,
     * preserving their files and provisioning marker. */
    status = fx_media_volume_set(&g_sd_media, "SEDS_DAQ");
    if (status == FX_SUCCESS) status = sd_media_flush();
    if (status != FX_SUCCESS) (void)fx_media_close(&g_sd_media);
    return status;
  }
  if (status == FX_SUCCESS) (void)fx_media_close(&g_sd_media);
  /* A remote retry never grants provisioning permission, including later
   * automatic retries in this boot. Existing logs/unknown media stay intact. */
  if (g_sd_manual_no_format) return status == FX_SUCCESS ? FX_NOT_FOUND : status;
  return sd_format_and_mark();
}

/* Transport recovery keeps the original FileX state and pending records. */
static UINT sd_resume_card_media(void)
{
  if (g_sd_media_mounted) {
    if (memcmp(g_sd_mounted_cid, hsd1.CID, sizeof(g_sd_mounted_cid)) != 0 ||
        g_sd_mounted_blocks != hsd1.SdCard.LogBlockNbr) {
      ++g_sd_card_identity_mismatches;
      return FX_IO_ERROR;
    }
    return FX_SUCCESS;
  }
  const UINT status = sd_mount_or_provision();
  if (status == FX_SUCCESS) {
    memcpy(g_sd_mounted_cid, hsd1.CID, sizeof(g_sd_mounted_cid));
    g_sd_mounted_blocks = hsd1.SdCard.LogBlockNbr;
    g_sd_media_mounted = 1U;
  }
  return status;
}

static uint64_t g_sd_raw_pending_key;
static uint8_t g_sd_raw_pending;

/* Pressure batch timestamps are VB capture clocks, not DAQ receipt clocks.
 * Only a valid UTC mapping can split a remote batch across T0/T+20s: bits0-3
 * mark rows inside [T0, T+20s), bits8-11 rows at or after T+20s. */
static uint32_t sd_pressure_capture_route(const pressure_sd_batch_t *batch,
    uint64_t session, uint64_t close_utc)
{
  /* Exact sd_calendar FAT UTC range, checked in constant time: the receive
   * callback holds PRIMASK while snapshotting and publishing this owner. */
  if (batch->unix_ms < UINT64_C(315532800000) ||
      batch->unix_ms >= UINT64_C(4354819200000)) return SD_PRESSURE_ROUTE_UNKNOWN_UTC;
  if (!session || close_utc < SD_LAUNCH_WINDOW_MS) return 0U;
  const uint64_t start = close_utc - SD_LAUNCH_WINDOW_MS;
  uint32_t bits = 0U;
  for (unsigned i = 0U; i < PRESSURE_SD_SAMPLES; ++i) {
    const uint64_t at = batch->unix_ms + i * PRESSURE_SD_PERIOD_MS;
    if (at >= close_utc) bits |= 1U << (8U + i);
    else if (at >= start) bits |= 1U << i;
  }
  return bits;
}

/* Sole-writer durable barrier: queued owner retirement alone is insufficient.
 * FileX media flush updates modified open-file directory entries, FAT/cache and
 * invokes the real media driver. Failed buffered writes retain their retry
 * offset and bytes in the existing helpers; never claim the old cohort durable. */
bool sd_card_thermal_flush_old(void)
{
  if (tx_thread_identify() != &sd_writer_thread || !g_sd_ready ||
      !sd_card_thermal_old_drained()) return false;
  if (sd_log_flush(&g_sd_regular_log) != FX_SUCCESS || sd_log_flush(&g_sd_launch_log) != FX_SUCCESS ||
      sd_media_flush() != FX_SUCCESS) return false;
  return g_sd_regular_log.len == 0U && g_sd_launch_log.len == 0U &&
      sd_card_thermal_old_drained();
}

/* Pressure rows classified against a launch window map to its file (L), the
 * regular file after it (L+1) or the regular file before it (L-1). */
static uint64_t sd_pressure_row_key(const sd_pressure_message_t *message, unsigned index)
{
  if ((message->capture_route & SD_PRESSURE_ROUTE_CLASSIFIED) == 0U) return message->route_key;
  if ((message->capture_route & (1U << index)) != 0U) return message->launch_key;
  if ((message->capture_route & (1U << (8U + index))) != 0U) return (uint64_t)message->launch_key + 1U;
  return (uint64_t)message->launch_key - 1U;
}

static uint64_t sd_pressure_oldest_key(const sd_pressure_message_t *message, unsigned first)
{
  uint64_t oldest = UINT64_MAX;
  for (unsigned i = first; i < PRESSURE_SD_SAMPLES; ++i) {
    const uint64_t key = sd_pressure_row_key(message, i);
    if (key < oldest) oldest = key;
  }
  return oldest;
}

/* Oldest route key still queued or partly written in any SD queue. Each FIFO
 * is in key order, so its head (or retained pending owner) is its oldest. */
static uint64_t sd_oldest_queued_key(void)
{
  uint64_t oldest = UINT64_MAX, key;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (g_sd_raw_pending) oldest = g_sd_raw_pending_key;
  else if (g_sd_raw_queue.tx_queue_enqueued != 0U)
    oldest = ((const sd_raw_slot_t *)(uintptr_t)*g_sd_raw_queue.tx_queue_read)->session;
  if (g_sd_line_pending) key = g_sd_line_pending_key;
  else if (g_sd_queue.tx_queue_enqueued != 0U)
    memcpy(&key, (const char *)(uintptr_t)*g_sd_queue.tx_queue_read + offsetof(sd_line_slot_t, session), sizeof(key));
  else key = UINT64_MAX;
  if (key < oldest) oldest = key;
  if (g_sd_pressure_pending) key = g_sd_pressure_pending_key;
  else if (g_sd_pressure_queue.tx_queue_enqueued != 0U) {
    sd_pressure_message_t head;
    memcpy(&head, (const void *)g_sd_pressure_queue.tx_queue_read, sizeof(head));
    key = sd_pressure_oldest_key(&head, 0U);
  } else key = UINT64_MAX;
  if (key < oldest) oldest = key;
  __set_PRIMASK(mask);
  return oldest;
}

/* No IRQ masking around allocation or file I/O. Only queue pointer ownership
 * changes under PRIMASK, so acquisition cannot append a writer-owned slot. */
static UINT sd_take_line_message(ULONG *message)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_receive(&g_sd_queue, message, TX_NO_WAIT);
  if (result == TX_SUCCESS && g_sd_scalar_append_tail == (sd_scalar_slot_t *)(uintptr_t)*message)
    g_sd_scalar_append_tail = NULL;
  __set_PRIMASK(mask);
  return result;
}

static UINT sd_take_raw_message(ULONG *message)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_receive(&g_sd_raw_queue, message, TX_NO_WAIT);
  if (result == TX_SUCCESS && g_sd_raw_append_tail == (sd_raw_slot_t *)(uintptr_t)*message)
    g_sd_raw_append_tail = NULL;
  __set_PRIMASK(mask);
  return result;
}

static void *sd_pool_allocate(TX_BLOCK_POOL *pool, size_t bytes)
{
  void *allocation = NULL;
  if (bytes + SD_POOL_ALIGNMENT_RESERVE > pool->tx_block_pool_block_size ||
      tx_block_allocate(pool, &allocation, TX_NO_WAIT) != TX_SUCCESS) return NULL;
  const uintptr_t aligned = ((uintptr_t)allocation + sizeof(void *) + 7U) & ~(uintptr_t)7U;
  ((void **)aligned)[-1] = allocation;
  return (void *)aligned;
}

static void sd_pool_release(void *slot)
{
  (void)tx_block_release(((void **)slot)[-1]);
}

static sd_line_slot_t *sd_alloc_slot(uint16_t len)
{
  sd_line_slot_t *slot = sd_pool_allocate(&g_sd_line_pool, offsetof(sd_line_slot_t, line) + len + 1U);
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    if (++g_sd_line_slots_used > g_sd_line_slots_peak) g_sd_line_slots_peak = g_sd_line_slots_used;
    __set_PRIMASK(saved);
  }
  return slot;
}

static void sd_free_slot(sd_line_slot_t *slot)
{
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq(); --g_sd_line_slots_used; __set_PRIMASK(saved);
    sd_pool_release(slot);
  }
}

static sd_raw_slot_t *sd_alloc_raw_slot(uint16_t count)
{
  sd_raw_slot_t *slot = sd_pool_allocate(&g_sd_raw_pool, offsetof(sd_raw_slot_t, samples) +
                                       (size_t)count * sizeof(sd_raw_adc_record_t));
  if (slot != NULL) {
    slot->capacity = count;
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    if (++g_sd_raw_slots_used > g_sd_raw_slots_peak) g_sd_raw_slots_peak = g_sd_raw_slots_used;
    __set_PRIMASK(saved);
  }
  return slot;
}

static void sd_free_raw_slot(sd_raw_slot_t *slot)
{
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq(); --g_sd_raw_slots_used; __set_PRIMASK(saved);
    sd_pool_release(slot);
  }
}

static uint8_t sd_raw_retire_completed(sd_raw_slot_t **pending, uint16_t index)
{
  sd_raw_slot_t *slot = pending ? *pending : NULL;
  if (!slot || index != slot->count) return 0U;
  sd_card_capture_release(slot->capture); sd_free_raw_slot(slot);
  *pending = NULL; g_sd_raw_pending = 0U;
  return 1U;
}

/* Two 500 Hz channels can enqueue two rows per acquisition batch. Drain a
 * bounded burst rather than one row per raw batch, which steadily falls behind
 * whenever raw formatting/SD writes take most of the writer's time slice. */
static uint8_t sd_optional_snapshot_guard(sd_line_slot_t *owner, uint16_t index);
static void sd_release_telemetry_owner(sd_line_slot_t *owner);

static uint16_t sd_telemetry_tag(const sd_line_slot_t *owner)
{
  uint16_t tag; memcpy(&tag, owner, sizeof(tag)); return tag;
}

static UINT sd_write_optional_snapshot_item(const sd_line_slot_t *pending,
    uint16_t *index, uint8_t *complete)
{
  const sd_optional_snapshot_t *owner = (const sd_optional_snapshot_t *)(const void *)pending;
  if (*index >= SD_OPTIONAL_SNAPSHOT_ROWS) return FX_PTR_ERROR;
  union {
    uint64_t alignment;
    UCHAR bytes[offsetof(sd_line_slot_t, line) + SD_LINE_MAX];
  } storage;
  sd_line_slot_t *expanded = (sd_line_slot_t *)(void *)storage.bytes;
  expanded->calibration = owner->calibration; expanded->capture = owner->capture;
  expanded->session = owner->session; expanded->route_epoch = owner->route_epoch;
  char value[24]; sd_format_float(value, owner->values[*index]);
  const int length = sd_format_csv_record(expanded->line, SD_LINE_MAX,
      owner->network_ms, owner->uptime,
      g_sd_scalar_sensors[SD_REQUIRED_SCALAR_SENSORS + *index], value,
      owner->sequence_base + *index);
  if (length <= 0 || (size_t)length >= SD_LINE_MAX) return FX_IO_ERROR;
  expanded->len = (uint16_t)length;
  const UINT result = sd_write_telemetry_row(expanded);
  if (result == FX_SUCCESS) {
    *complete = ++*index == SD_OPTIONAL_SNAPSHOT_ROWS;
    ++g_sd_optional_scalar_records_written;
  }
  return result;
}

static UINT sd_write_telemetry_item(const sd_line_slot_t *pending,
                                    uint16_t *index, uint8_t *complete)
{
  /* Shared prefix reads use bytes: a scalar allocation has a different
   * effective type from the expanded CSV slot under release strict aliasing. */
  uint16_t tag; memcpy(&tag, pending, sizeof(tag));
  if (tag == SD_OPTIONAL_SNAPSHOT_TAG)
    return sd_write_optional_snapshot_item(pending, index, complete);
  if (tag != SD_SCALAR_SLOT_TAG) {
    *complete = 1U;
    return sd_write_telemetry_row(pending);
  }
  const sd_scalar_slot_t *batch = (const sd_scalar_slot_t *)(const void *)pending;
  const sd_scalar_record_t *row = &batch->records[*index];
  union {
    uint64_t alignment;
    UCHAR bytes[offsetof(sd_line_slot_t, line) + SD_LINE_MAX];
  } storage;
  sd_line_slot_t *expanded = (sd_line_slot_t *)(void *)storage.bytes;
  expanded->calibration = batch->calibration; expanded->capture = batch->capture; expanded->session = batch->session;
  expanded->route_epoch = batch->route_epoch;
  char value[24]; sd_format_float(value, row->value);
  const int length = sd_format_csv_record(expanded->line, SD_LINE_MAX,
      row->network_ms, board_uptime_project(batch->uptime_reference, row->monotonic_ms),
      g_sd_scalar_sensors[row->sensor], value, batch->sequence_base + row->sequence_delta);
  if (length <= 0 || (size_t)length >= SD_LINE_MAX) return FX_IO_ERROR;
  expanded->len = (uint16_t)length;
  const UINT result = sd_write_telemetry_row(expanded);
  if (result == FX_SUCCESS) {
    *complete = ++*index == batch->count;
    if (row->sensor < SD_REQUIRED_SCALAR_SENSORS) ++g_sd_scalar_records_written;
    else ++g_sd_optional_scalar_records_written;
  }
  return result;
}

static uint8_t sd_service_telemetry_rows(void)
{
  static sd_line_slot_t *pending;
  static uint16_t scalar_index;
  uint8_t serviced = 0U;
  for (unsigned budget = 0U; budget < 5U; ++budget) {
    if (!pending) {
      ULONG message;
      if (sd_take_line_message(&message) != TX_SUCCESS) break;
      pending = (sd_line_slot_t *)(uintptr_t)message;
      scalar_index = 0U;
      g_sd_line_pending = pending != NULL;
      if (pending) memcpy(&g_sd_line_pending_key,
          (const char *)pending + offsetof(sd_line_slot_t, session), sizeof(g_sd_line_pending_key));
    }
    if (pending && sd_telemetry_tag(pending) == SD_OPTIONAL_SNAPSHOT_TAG) {
      const uint8_t guard = sd_optional_snapshot_guard(pending, scalar_index);
      if (guard == 0U) break; /* Retain this exact owner and cursor. */
      if (guard == 2U) {
        sd_capture_t abandoned; memcpy(&abandoned, (const char *)pending + offsetof(sd_line_slot_t, capture), sizeof(abandoned));
        sd_card_capture_fault(abandoned);
        sd_release_telemetry_owner(pending);
        pending = NULL; g_sd_line_pending = 0U; serviced = 1U;
        continue; /* This discard consumes one of the same five iterations. */
      }
    }
    serviced = 1U;
    if (!pending) continue;
    uint8_t complete;
    if (sd_write_telemetry_item(pending, &scalar_index, &complete) != FX_SUCCESS) {
      ++g_sd_write_error_count; break;
    }
    g_sd_csv_rows_written_count++;
    if (!complete) continue;
    if (sd_telemetry_tag(pending) == SD_OPTIONAL_SNAPSHOT_TAG) ++g_sd_optional_snapshots_completed;
    sd_release_telemetry_owner(pending); pending = NULL; g_sd_line_pending = 0U;
  }
  return serviced;
}

/* Retain a failed row and its batch until the media accepts it. A bounded
 * four-row turn leaves the existing raw and telemetry services runnable. */
static uint8_t sd_service_pressure_rows(void)
{
  static union {
    sd_pressure_message_t message;
    ULONG words[SD_PRESSURE_MESSAGE_WORDS];
  } pending;
  static union {
    uint64_t alignment;
    UCHAR bytes[offsetof(sd_line_slot_t, line) + SD_LINE_MAX];
  } storage;
  static unsigned index;
  sd_line_slot_t *slot = (sd_line_slot_t *)(void *)storage.bytes;
  uint8_t serviced = 0U;
  for (unsigned budget = 0U; budget < PRESSURE_SD_SAMPLES; ++budget) {
    if (!g_sd_pressure_pending) {
      if (tx_queue_receive(&g_sd_pressure_queue, pending.words, TX_NO_WAIT) != TX_SUCCESS) break;
      index = 0U; slot->len = 0U;
      g_sd_pressure_pending_key = sd_pressure_oldest_key(&pending.message, 0U);
      g_sd_pressure_pending = 1U;
      slot->capture = pending.message.route_epoch;
      /* Pressure raw/converted values come from VB; DAQ coefficients are only
       * the common file metadata and never transform a pressure record. */
      const uint32_t snapshot_mask = __get_PRIMASK(); __disable_irq();
      (void)sd_calibration_snapshot(&slot->calibration);
      if (sd_capture_old_locked(pending.message.route_epoch))
        memcpy(slot->calibration.thermal, g_sd_capture.old_tuple, sizeof(g_sd_capture.old_tuple));
      __set_PRIMASK(snapshot_mask);
    }
    slot->session = sd_pressure_row_key(&pending.message, index);
    serviced = 1U;
    const pressure_sd_batch_t *batch = &pending.message.batch;
    if (slot->len == 0U) {
      char raw[12], converted[24];
      snprintf(raw, sizeof(raw), "%u", (unsigned)batch->raw[index]);
      sd_format_float(converted, batch->psi[index]);
      const int n = sd_format_csv_sensor_record(slot->line, SD_LINE_MAX,
          batch->unix_ms ? batch->unix_ms + index * PRESSURE_SD_PERIOD_MS : 0U,
          batch->uptime_ms + index * PRESSURE_SD_PERIOD_MS, "valve_pressure_psi",
          raw, converted, (uint32_t)(batch->first_sequence + index));
      if (n < 0) {
        ++g_sd_pressure_records_dropped; ++g_sd_write_error_count;
        sd_card_capture_fault(pending.message.route_epoch);
        if (++index == PRESSURE_SD_SAMPLES) { sd_card_capture_release(pending.message.route_epoch); g_sd_pressure_pending = 0U; }
        continue;
      }
      slot->len = (uint16_t)n;
    }
    const UINT status = sd_write_telemetry_row(slot);
    if (status != FX_SUCCESS) { ++g_sd_write_error_count; break; }
    ++g_sd_pressure_records_written; ++g_sd_csv_rows_written_count;
    slot->len = 0U;
    if (++index == PRESSURE_SD_SAMPLES) { sd_card_capture_release(pending.message.route_epoch); g_sd_pressure_pending = 0U; }
    else g_sd_pressure_pending_key = sd_pressure_oldest_key(&pending.message, index);
  }
  return serviced;
}

/* Close a superseded file exactly once: after its route transition plus the
 * grace period, and only when no queued row still belongs to it. Failures
 * retain the open handle and retry; a finished launch file never reopens. */
static void sd_close_superseded_logs(void)
{
  (void)sd_launch_finished();
  const uint64_t local = daq_log_monotonic_ms();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t route = sd_route_session_locked(local);
  const uint64_t changed = g_sd_route_change_local;
  __set_PRIMASK(mask);
  if (local - changed >= SD_ROUTE_GRACE_MS) {
    sd_log_t *const logs[] = {&g_sd_launch_log, &g_sd_regular_log};
    uint64_t oldest = 0U;
    uint8_t oldest_known = 0U;
    for (unsigned i = 0U; i < 2U; ++i) {
      sd_log_t *log = logs[i];
      if (!log->open || log->key == route) continue;
      if (!oldest_known) { oldest = sd_oldest_queued_key(); oldest_known = 1U; }
      if (oldest <= log->key) continue;
      if (sd_log_finish(log) != FX_SUCCESS) ++g_sd_write_error_count;
    }
  }
  if (g_sd_launch_flush_pending) {
    if (sd_media_flush() == FX_SUCCESS) g_sd_launch_flush_pending = 0U;
    else ++g_sd_write_error_count;
  }
}

static uint32_t sd_take_manual_retry(void)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint32_t token = g_sd_retry_pending_token;
  if (token != 0U) {
    g_sd_retry_pending_token = 0U; g_sd_retry_active_token = token;
    g_sd_manual_no_format = 1U;
  }
  __set_PRIMASK(saved);
  return token;
}

static void sd_complete_manual_retry(void)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  if (g_sd_retry_active_token != 0U) {
    g_sd_retry_completed_token = g_sd_retry_active_token; g_sd_retry_active_token = 0U;
  }
  __set_PRIMASK(saved);
}

/* Network SD download service. Runs only inside an otherwise idle writer turn
 * (no queued raw, telemetry or pressure rows serviced), never while a launch
 * file is open or its close flush is pending, and only in Idle, Landed or
 * Recovery. Each turn performs at most one sector read or a short bounded
 * directory step into the single 512-byte download block. The read handle is
 * a separate FX_OPEN_FOR_READ owner; it never writes and never alters a log. */
#define SD_DL_IDLE_CLOSE_MS 3000U
#define SD_DL_LIST_SCAN_PER_TURN 32U
/* The read handle lives in the on-demand download workspace (see
 * sd_file_stream.c); it exists only while a station session or open handle does. */
#define g_sd_dl_file (*(FX_FILE *)sd_file_stream_handle())
static uint8_t g_sd_dl_open, g_sd_dl_dir;
size_t sd_card_download_handle_size(void) { return sizeof(FX_FILE); }
static void sd_dl_mark(uint8_t open) { g_sd_dl_open = open; sd_file_stream_handle_open(open != 0U); }
/* Listing borrows the media default directory only inside one writer turn and
 * restores the root before returning; only the iterator position persists. */
static uint8_t g_sd_dl_list_valid, g_sd_dl_list_dir;
static uint16_t g_sd_dl_list_next;
static ULONG g_sd_dl_list_entry;
static uint32_t g_sd_dl_last_ms;
volatile uint32_t g_sd_dl_reads, g_sd_dl_lists, g_sd_dl_errors;

static void sd_dl_release(void)
{
  if (g_sd_dl_open) { (void)fx_file_close(&g_sd_dl_file); sd_dl_mark(0U); }
  g_sd_dl_list_valid = 0U;
}

static uint8_t sd_dl_name_is_open_log(const char *path)
{
  return (g_sd_regular_log.open && strcmp(g_sd_regular_log.name, path) == 0) ||
         (g_sd_launch_log.open && strcmp(g_sd_launch_log.name, path) == 0);
}

/* LIST answers place the name in the download block (unused by LIST). */
static void sd_dl_list(const sd_file_request_t *request, sd_file_result_t *result, uint8_t *block)
{
  result->status = SD_FILE_STATUS_IO_ERROR;
  const UINT set = fx_directory_default_set(&g_sd_media, (CHAR *)sd_file_directory(request->dir));
  if (set != FX_SUCCESS) {
    (void)fx_directory_default_set(&g_sd_media, FX_NULL);
    g_sd_dl_list_valid = 0U;
    result->status = (set == FX_INVALID_PATH || set == FX_NOT_FOUND || set == FX_NOT_DIRECTORY)
        ? SD_FILE_STATUS_END : SD_FILE_STATUS_IO_ERROR;
    return;
  }
  if (!g_sd_dl_list_valid || g_sd_dl_list_dir != request->dir || request->index < g_sd_dl_list_next) {
    g_sd_dl_list_valid = 1U; g_sd_dl_list_dir = request->dir; g_sd_dl_list_next = 0U; g_sd_dl_list_entry = 0U;
  }
  CHAR *name = (CHAR *)block;
  for (uint32_t scanned = 0U; scanned < SD_DL_LIST_SCAN_PER_TURN; ++scanned) {
    UINT attributes = 0U, year, month, day, hour, minute, second;
    ULONG size = 0U;
    g_sd_media.fx_media_default_path.fx_path_current_entry = g_sd_dl_list_entry;
    const UINT status = fx_directory_next_full_entry_find(&g_sd_media, name, &attributes, &size,
                                                          &year, &month, &day, &hour, &minute, &second);
    g_sd_dl_list_entry = g_sd_media.fx_media_default_path.fx_path_current_entry;
    if (status == FX_NO_MORE_ENTRIES) { result->status = SD_FILE_STATUS_END; break; }
    if (status != FX_SUCCESS) { g_sd_dl_list_valid = 0U; break; }
    name[FX_MAX_LONG_NAME_LEN - 1U] = '\0';
    const size_t length = strlen(name);
    if ((attributes & (FX_DIRECTORY | FX_VOLUME | FX_HIDDEN | FX_SYSTEM)) != 0U ||
        !sd_file_name_valid(name, length)) continue;
    if (g_sd_dl_list_next++ != request->index) continue;
    char path[72];
    (void)sd_file_path(path, sizeof path, request->dir, name, length);
    result->status = SD_FILE_STATUS_OK;
    result->file_size = (uint32_t)size;
    result->flags = sd_dl_name_is_open_log(path) ? SD_FILE_ENTRY_OPEN : 0U;
    result->name_len = (uint8_t)length;
    break;
  }
  (void)fx_directory_default_set(&g_sd_media, FX_NULL);
  /* A long skip continues on the station's retry; no turn scans unbounded. */
  if (result->status == SD_FILE_STATUS_IO_ERROR && g_sd_dl_list_valid) result->status = SD_FILE_STATUS_BUSY;
}

static void sd_dl_read(const sd_file_request_t *request, sd_file_result_t *result, uint8_t *block)
{
  char path[72];
  result->status = SD_FILE_STATUS_BAD_MESSAGE;
  if (!sd_file_path(path, sizeof path, request->dir, request->name, request->name_len)) return;
  const uint8_t growing = sd_dl_name_is_open_log(path);
  if (g_sd_dl_open && (g_sd_dl_dir != request->dir ||
      strcmp(request->name, g_sd_dl_file.fx_file_name_buffer) != 0 ||
      (growing && request->offset >= g_sd_dl_file.fx_file_current_file_size))) {
    (void)fx_file_close(&g_sd_dl_file); sd_dl_mark(0U);
  }
  if (!g_sd_dl_open) {
    const UINT status = fx_file_open(&g_sd_media, &g_sd_dl_file, path, FX_OPEN_FOR_READ);
    if (status != FX_SUCCESS) {
      result->status = status == FX_NOT_FOUND ? SD_FILE_STATUS_NOT_FOUND : SD_FILE_STATUS_IO_ERROR;
      return;
    }
    sd_dl_mark(1U); g_sd_dl_dir = request->dir;
  }
  const uint64_t size64 = g_sd_dl_file.fx_file_current_file_size;
  const uint32_t size = size64 > UINT32_MAX ? UINT32_MAX : (uint32_t)size64;
  result->file_size = size; result->offset = request->offset; result->flags = growing ? SD_FILE_ENTRY_OPEN : 0U;
  if (request->offset > size) { result->status = SD_FILE_STATUS_BAD_OFFSET; return; }
  if (request->offset == size) { result->status = SD_FILE_STATUS_END; return; }
  const uint32_t start = request->offset & ~(uint32_t)(SD_FILE_BLOCK_BYTES - 1U);
  const uint32_t want = size - start < SD_FILE_BLOCK_BYTES ? size - start : SD_FILE_BLOCK_BYTES;
  ULONG actual = 0U;
  UINT status = fx_file_seek(&g_sd_dl_file, start);
  if (status == FX_SUCCESS) status = fx_file_read(&g_sd_dl_file, block, want, &actual);
  if (status != FX_SUCCESS || actual <= request->offset - start) {
    (void)fx_file_close(&g_sd_dl_file); sd_dl_mark(0U);
    result->status = SD_FILE_STATUS_IO_ERROR;
    return;
  }
  result->status = SD_FILE_STATUS_OK;
  result->data_start = (uint16_t)(request->offset - start);
  result->data_len = (uint16_t)(actual - result->data_start);
}

static uint8_t sd_file_service_turn(void)
{
  const uint32_t now = HAL_GetTick();
  const uint8_t allowed = sd_file_stream_state_allowed() && g_sd_launch_log.open == 0U &&
                          g_sd_launch_flush_pending == 0U;
  if ((!allowed || !sd_file_stream_session_active() || now - g_sd_dl_last_ms >= SD_DL_IDLE_CLOSE_MS) &&
      (g_sd_dl_open || g_sd_dl_list_valid))
    sd_dl_release();
  sd_file_request_t request;
  uint32_t ticket = 0U;
  if (!sd_file_stream_claim(&request, &ticket)) return 0U;
  sd_file_result_t result;
  memset(&result, 0, sizeof result);
  uint8_t *const block = sd_file_stream_block();
  if (!allowed || block == NULL) {
    result.status = SD_FILE_STATUS_BAD_STATE;
  } else if (request.op == SD_FILE_OP_LIST) {
    sd_dl_list(&request, &result, block); ++g_sd_dl_lists;
  } else {
    sd_dl_read(&request, &result, block); ++g_sd_dl_reads;
  }
  if (result.status == SD_FILE_STATUS_IO_ERROR) ++g_sd_dl_errors;
  g_sd_dl_last_ms = HAL_GetTick();
  sd_file_stream_finish(ticket, &result);
  return 1U;
}

void sd_card_writer_thread_entry(ULONG initial_input)
{
  ULONG message = 0U;
  sd_raw_slot_t *pending_raw = NULL;
  uint16_t pending_raw_index = 0U;
  sd_raw_format_cache_t raw_format_cache = {0};
  ULONG last_flush = tx_time_get();
  /* Attempt the first mount immediately. Subsequent missing-card retries stay
   * at one second, but a healthy card should never delay acquisition logging
   * merely because the board has just booted. */
  ULONG last_retry = (ULONG)(0U - TX_TIMER_TICKS_PER_SECOND);
  uint32_t failed_retries = 0U;
  (void)initial_input;

  for (;;)
  {
    ULONG flags = 0U;
    uint8_t serviced_work = 0U;
    /* This boundary owns no in-flight IDMA buffer or FileX operation. Keep
     * pending records, dirty cache, handles and monotonic launch deadlines. */
    if (sd_take_manual_retry() != 0U) {
      g_sd_ready = 0U;
      last_retry = tx_time_get() - TX_TIMER_TICKS_PER_SECOND;
    }

    if (g_sd_ready == 0U)
    {
      const ULONG now = tx_time_get();
      if ((now - last_retry) >= TX_TIMER_TICKS_PER_SECOND)
      {
        last_retry = now;
        g_sd_retry_count++;
        g_sd_init_stage = 1U;
        /* Init clears ErrorCode only after success, while wide-bus setup
         * checks it earlier. Reset stale errors and controller state first. */
        (void)HAL_SD_DeInit(&hsd1);
        /* Clock gating in MSP deinit does not reset a stuck data-path state
         * machine. Reset the peripheral before the next card initialization. */
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
        if ((HAL_SD_Init(&hsd1) == HAL_OK) &&
            ((g_sd_init_stage = 2U) != 0U) &&
            (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK))
        {
          g_sd_hardware_ready = 1U;
          g_sd_init_stage = 3U;
          const UINT media_status = sd_resume_card_media();
          if (media_status == FX_SUCCESS) {
            g_sd_ready = 1U;
            g_sd_init_stage = 5U;
            failed_retries = 0U;
          }
        }

        sd_complete_manual_retry();
        if (g_sd_ready == 0U)
        {
          g_sd_hardware_ready = 0U;
          g_sd_init_failures++;
          if ((failed_retries == 0U) || ((failed_retries % 60U) == 0U))
          {
#ifdef TELEMETRY_ENABLED
            if (log_telemetry_string_asynchronous(
                    SEDS_DT_WARNING,
                    "DAQ SD card unavailable; acquisition continues without logging") == SEDS_OK)
            {
              g_sd_warning_publish_count++;
            }
#endif
          }
          failed_retries++;
        }
      }
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U);
      continue;
    }

    /* Rotate only when a record with new coefficients arrives. Rotating from
     * the global snapshot while idle creates header-only files and can switch
     * forward then backward across an in-flight old acquisition batch. */
    const uint32_t service_start_ms = HAL_GetTick();

    /* Bound raw work so a continuously replenished ADC queue cannot starve
     * live/replay rows, calibration rotation, or flush requests. */
    if (pending_raw == NULL && sd_take_raw_message(&message) == TX_SUCCESS) {
      pending_raw = (sd_raw_slot_t *)(uintptr_t)message;
      pending_raw_index = 0U;
      g_sd_raw_pending_key = pending_raw->session; g_sd_raw_pending = 1U;
    }
    if (pending_raw != NULL)
    {
      serviced_work = 1U;
      sd_raw_slot_t *slot = pending_raw;
      char line[256];
      if (slot != NULL)
      {
        /* Reserve a worst-case complete batch before admitting its first row. */
        sd_log_t *log = sd_log_for_key(slot->session, &slot->calibration,
                                       (uint64_t)SD_RAW_BATCH_MAX * 256U);
        if (log == NULL)
        {
          g_sd_write_error_count++;
          tx_thread_sleep(1U);
          continue;
        }
        /* Larger allocations improve stall capacity without enlarging the
         * writer turn: telemetry/PT queues retain their original service share. */
        const uint16_t turn_end = slot->count - pending_raw_index > SD_RAW_BATCH_MAX ?
            pending_raw_index + SD_RAW_BATCH_MAX : slot->count;
        for (uint16_t i = pending_raw_index; i < turn_end; ++i)
        {
          const sd_raw_adc_record_t *sample = &slot->samples[i];
          const uint64_t captured_uptime = board_uptime_project(slot->uptime_reference, sample->monotonic_ms);
          const char *sensor = daq_chamber_is_channel(sample->channel) ? "chamber_pressure_raw_volts_psi" : sample->channel < 8U ? (const char *const[]){
              "mcp3564r_raw", "kg50_raw", "mcp3564r_ch2_raw", "mcp3564r_ch3_raw",
              "mcp3564r_ch4_raw", "mcp3564r_ch5_raw", "mcp3564r_ch6_raw", "mcp3564r_ch7_raw"
          }[sample->channel] : sample->channel == 9U ? "mcp3564r_ch2_ch3_gain16_raw" : "invalid_adc_channel";
          const int len = sd_format_csv_raw_record(line, sizeof(line),
              sample->network_unix_ms, captured_uptime, sensor, sample->raw_adc_code,
              sample->raw_value, sample->calibrated_value, sample->adc_temperature_c,
              sample->adc_temperature_code, sample->record_sequence, &raw_format_cache);
          if (len <= 0 || (size_t)len >= sizeof(line)) {
            ++g_sd_write_error_count; ++g_sd_raw_records_io_lost_count;
            sd_card_capture_fault(slot->capture);
            pending_raw_index = i + 1U;
            continue; /* Count an unformattable record; do not wedge the entire logger. */
          }
          if (sd_log_write(log, line, (size_t)len) != FX_SUCCESS) {
            ++g_sd_write_error_count;
            tx_thread_sleep(1U);
            break; /* Retain this exact record and batch for the next service turn. */
          }
          g_sd_raw_records_written_count++;
          pending_raw_index = i + 1U;
        }
        (void)sd_raw_retire_completed(&pending_raw, pending_raw_index);
      }
    }

    const uint32_t raw_ms = HAL_GetTick() - service_start_ms;
    if (raw_ms > g_sd_max_raw_service_ms) g_sd_max_raw_service_ms = raw_ms;
    const uint32_t telemetry_start_ms = HAL_GetTick();
    serviced_work |= sd_service_telemetry_rows();
    serviced_work |= sd_service_pressure_rows();
    const uint32_t telemetry_ms = HAL_GetTick() - telemetry_start_ms;
    if (telemetry_ms > g_sd_max_telemetry_service_ms) g_sd_max_telemetry_service_ms = telemetry_ms;

    if (tx_event_flags_get(&g_sd_flags,
                           SD_FLAG_FLUSH_REQUEST | SD_FLAG_POWER_LOSS,
                           TX_OR_CLEAR, &flags, TX_NO_WAIT) == TX_SUCCESS)
    {
      /* A brownout warning requests an immediate flush, not a permanent
       * recording shutdown. A startup ADC transient must not stop this run. */
    }

    if ((flags != 0U) ||
        ((tx_time_get() - last_flush) >= TX_TIMER_TICKS_PER_SECOND))
    {
      const uint32_t flush_start_ms = HAL_GetTick();
      sd_update_filesystem_clock();
      if ((sd_log_flush(&g_sd_regular_log) == FX_SUCCESS) &&
          (sd_log_flush(&g_sd_launch_log) == FX_SUCCESS) &&
          (sd_media_flush() == FX_SUCCESS))
      {
        g_sd_launch_flush_pending = 0U;
        g_sd_flush_count++;
        g_sd_raw_records_flushed_count = g_sd_raw_records_written_count;
        g_sd_pressure_records_flushed = g_sd_pressure_records_written;
      }
      else
      {
        g_sd_write_error_count++;
      }
      last_flush = tx_time_get();
      const uint32_t flush_ms = HAL_GetTick() - flush_start_ms;
      if (flush_ms > g_sd_max_flush_ms) g_sd_max_flush_ms = flush_ms;
    }

    /* Superseded regular/launch files close once; ordinary SD work continues. */
    sd_close_superseded_logs();

    const uint32_t service_ms = HAL_GetTick() - service_start_ms;
    if (service_ms > g_sd_max_service_ms) g_sd_max_service_ms = service_ms;
    /* Downloads use only turns with no logging work; excluded from service stats. */
    if (serviced_work == 0U) serviced_work = sd_file_service_turn();
    if (serviced_work == 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 1000U);
    }
  }
}

_Static_assert(SD_OPTIONAL_RAW_ADMIT_LIMIT * sizeof(sd_raw_adc_record_t) <= sizeof(g_sd_raw_pool_storage) / 2U,
               "A 500 ms optional stall budget must leave half the raw pool for essential rows");

UINT sd_card_init(TX_BYTE_POOL *byte_pool)
{
  (void)byte_pool;
  if (g_sd_services_initialized != 0U)
  {
    return TX_SUCCESS;
  }
  if (tx_block_pool_create(&g_sd_line_pool, "sd_lines", SD_LINE_BLOCK_PAYLOAD + SD_POOL_ALIGNMENT_RESERVE,
                           g_sd_line_pool_storage, sizeof(g_sd_line_pool_storage)) != TX_SUCCESS ||
      tx_block_pool_create(&g_sd_raw_pool, "sd_raw", SD_RAW_BLOCK_PAYLOAD + SD_POOL_ALIGNMENT_RESERVE,
                           g_sd_raw_pool_storage, sizeof(g_sd_raw_pool_storage)) != TX_SUCCESS)
    return TX_POOL_ERROR;
  const UINT io_status = sd_storage_io_init();
  if (io_status != TX_SUCCESS) return io_status;
  if (tx_queue_create(&g_sd_queue, "sd_queue", TX_1_ULONG,
                      g_sd_queue_storage, sizeof(g_sd_queue_storage)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }
  if (tx_queue_create(&g_sd_raw_queue, "sd_raw_queue", TX_1_ULONG,
                      g_sd_raw_queue_storage,
                      sizeof(g_sd_raw_queue_storage)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }
  if (tx_queue_create(&g_sd_pressure_queue, "sd_pressure", SD_PRESSURE_MESSAGE_WORDS,
                      g_sd_pressure_queue_storage, sizeof(g_sd_pressure_queue_storage)) != TX_SUCCESS)
    return TX_QUEUE_ERROR;
  if (tx_event_flags_create(&g_sd_flags, "sd_flags") != TX_SUCCESS)
  {
    return TX_GROUP_ERROR;
  }

  fx_system_initialize();
  /* Unknown time is explicitly the FAT epoch, never FileX's 2017 default. */
  (void)fx_system_date_set(1980U, 1U, 1U);
  (void)fx_system_time_set(0U, 0U, 0U);
  g_sd_services_initialized = 1U;

  /* All card/media operations belong to the lower-priority writer. */
  return TX_SUCCESS;
}

void sd_card_set_hardware_ready(uint8_t ready)
{
  g_sd_hardware_ready = (ready != 0U) ? 1U : 0U;
}

static sd_card_status_t sd_enqueue_line(sd_line_slot_t *slot, uint64_t capture_unix_ms)
{
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  slot->session = sd_capture_key_locked(sd_run_snapshot(), capture_unix_ms);
  slot->route_epoch = g_sd_route_epoch;
  g_sd_scalar_append_tail = NULL;
  const UINT result = tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT);
  __set_PRIMASK(mask);
  if (result != TX_SUCCESS)
  {
    g_sd_line_drop_count++;
    sd_card_capture_fault(slot->capture);
    sd_card_capture_release(slot->capture); sd_free_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
}

static uint8_t sd_raw_is_essential(const sd_raw_adc_record_t *sample)
{
  return sample->channel == 0U || sample->channel == 1U || sample->channel == 9U ||
      daq_chamber_is_channel(sample->channel);
}

static void sd_raw_count_dropped(const sd_raw_adc_record_t *samples, uint16_t count,
                                  uint8_t only_optional)
{
  for (uint16_t i = 0U; i < count; ++i) {
    if (sd_raw_is_essential(&samples[i])) {
      if (only_optional) continue;
      ++g_sd_raw_essential_records_dropped;
    } else ++g_sd_raw_optional_records_dropped;
    ++g_sd_raw_records_dropped_count;
  }
}

/* These atomic word reads are an early-shedding hint, never allocation
 * authorization. Required loadcells and pressure do not use this gate. */
static uint8_t sd_raw_optional_admitted(void)
{
  return sd_raw_backlog_records() <= SD_OPTIONAL_RAW_ADMIT_LIMIT &&
      g_sd_raw_pool.tx_block_pool_available >= (g_sd_raw_pool.tx_block_pool_total + 3U) / 4U;
}

static uint8_t sd_optional_telemetry_admitted(void)
{
  return sd_raw_optional_admitted() &&
      g_sd_line_pool.tx_block_pool_available >= (g_sd_line_pool.tx_block_pool_total * 3U + 3U) / 4U;
}

/* The original raw guard still gates optional expansion. Retention must not
 * pin the FIFO head until required producers consume their reserved blocks. */
static uint8_t sd_optional_snapshot_guard(sd_line_slot_t *owner, uint16_t index)
{
  if (sd_raw_optional_admitted()) return 1U;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  for (unsigned slot = 0U; slot < SD_OPTIONAL_SNAPSHOT_SLOTS; ++slot) {
    if ((void *)owner != (void *)&g_sd_optional_snapshots[slot]) continue;
    const uint32_t bit = 1UL << slot;
    if ((g_sd_optional_snapshot_deferred & bit) == 0U) {
      g_sd_optional_snapshot_deferred |= bit;
      ++g_sd_optional_snapshots_deferred;
    }
    break;
  }
  const uint8_t discard = g_sd_line_pool.tx_block_pool_available <
      (g_sd_line_pool.tx_block_pool_total * 3U + 3U) / 4U;
  if (discard) {
    const uint32_t remaining = SD_OPTIONAL_SNAPSHOT_ROWS - index;
    g_sd_optional_telemetry_dropped += remaining; g_sd_line_drop_count += remaining;
    g_sd_optional_snapshot_rows_discarded += remaining;
  }
  __set_PRIMASK(mask);
  return discard ? 2U : 0U;
}

static void sd_release_telemetry_owner(sd_line_slot_t *owner)
{
  sd_capture_t capture; memcpy(&capture, (const char *)owner + offsetof(sd_line_slot_t, capture), sizeof(capture));
  sd_card_capture_release(capture);

  for (unsigned slot = 0U; slot < SD_OPTIONAL_SNAPSHOT_SLOTS; ++slot) {
    if ((void *)owner != (void *)&g_sd_optional_snapshots[slot]) continue;
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    g_sd_optional_snapshot_used &= ~(1UL << slot);
    g_sd_optional_snapshot_deferred &= ~(1UL << slot);
    --g_sd_optional_snapshot_slots_used;
    __set_PRIMASK(mask);
    return;
  }
  sd_free_slot(owner);
}

static uint8_t sd_required_sensor_name(const char *name)
{
  return strcmp(name, "kg1000_network") == 0 || strcmp(name, "kg50_network") == 0 ||
      strcmp(name, "kg50_p6_network_volts") == 0 ||
      strcmp(name, "chamber_pressure_raw_volts_psi") == 0 || strcmp(name, "valve_pressure_psi") == 0;
}

static uint64_t sd_row_sequence;
static uint64_t sd_next_row_sequence(void) {
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t value = ++sd_row_sequence; __set_PRIMASK(saved); return value;
}
static int sd_scalar_sensor_index(const char *name)
{
  for (unsigned i = 0U; i < sizeof(g_sd_scalar_sensors) / sizeof(g_sd_scalar_sensors[0]); ++i)
    if (strcmp(name, g_sd_scalar_sensors[i]) == 0) return (int)i;
  return -1;
}

static sd_card_status_t sd_enqueue_scalar(uint8_t sensor, uint64_t network_ms,
    uint64_t uptime, float value, const daq_calibration_t *calibration, sd_capture_t capture)
{
  const uint8_t required = sensor < SD_REQUIRED_SCALAR_SENSORS;
  /* A compact append consumes no new block, but optional work still observes
   * the existing raw-backlog/CPU guard. Required rows keep their prior path. */
  if (!required && !sd_raw_optional_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  const uint64_t sequence = sd_next_row_sequence();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t session = sd_capture_key_locked(sd_run_snapshot(), network_ms);
  const uint32_t epoch = g_sd_route_epoch;
  sd_scalar_slot_t *tail = g_sd_scalar_append_tail;
  if (tail && tail->count < tail->capacity && tail->session == session &&
      tail->route_epoch == epoch && tail->capture == capture && sequence - tail->sequence_base <= UINT32_MAX &&
      memcmp(&tail->calibration, calibration, sizeof(*calibration)) == 0) {
    tail->records[tail->count++] = (sd_scalar_record_t){network_ms, (uint32_t)uptime,
        (uint32_t)(sequence - tail->sequence_base), value, sensor};
    if (uptime > tail->uptime_reference) tail->uptime_reference = uptime;
    if (required) { ++g_sd_scalar_records_enqueued; ++g_sd_scalar_records_coalesced; }
    else { ++g_sd_optional_scalar_records_enqueued; ++g_sd_optional_scalar_records_coalesced; }
    __set_PRIMASK(mask); return SD_CARD_STATUS_OK;
  }
  __set_PRIMASK(mask);
  /* Existing allocator owns slot counters and8-byte alignment. Its len is a
   * byte payload size here; the tag is assigned only after allocation. */
  const size_t bytes = offsetof(sd_scalar_slot_t, records) - offsetof(sd_line_slot_t, line) +
      SD_SCALAR_BATCH_MAX * sizeof(sd_scalar_record_t);
  /* Preserve the full original required reserve for every new optional owner.
   * A matching queued tail above remains bounded by its existing32 rows. */
  if (!required && !sd_optional_telemetry_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  sd_scalar_slot_t *slot = (sd_scalar_slot_t *)(void *)sd_alloc_slot((uint16_t)bytes);
  if (!slot) {
    ++g_sd_line_drop_count;
    if (required) ++g_sd_essential_telemetry_dropped;
    else ++g_sd_optional_telemetry_dropped;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->len = SD_SCALAR_SLOT_TAG; slot->count = 1U; slot->capacity = SD_SCALAR_BATCH_MAX;
  slot->session = session; slot->route_epoch = epoch; slot->calibration = *calibration;
  slot->capture = capture; sd_card_capture_retain(capture);
  slot->sequence_base = sequence; slot->uptime_reference = uptime;
  slot->records[0] = (sd_scalar_record_t){network_ms, (uint32_t)uptime, 0U, value, sensor};
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t queued_mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT);
  if (result == TX_SUCCESS) {
    g_sd_scalar_append_tail = slot;
    if (required) ++g_sd_scalar_records_enqueued;
    else ++g_sd_optional_scalar_records_enqueued;
  }
  __set_PRIMASK(queued_mask);
  if (result != TX_SUCCESS) {
    ++g_sd_line_drop_count;
    if (required) ++g_sd_essential_telemetry_dropped;
    else ++g_sd_optional_telemetry_dropped;
    sd_card_capture_release(capture); sd_free_slot((sd_line_slot_t *)(void *)slot); return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
}

#ifdef TELEMETRY_ENABLED
sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt)
{
  if (!g_sd_ready || !pkt) return SD_CARD_STATUS_BUSY;
  daq_calibration_t captured_calibration;
  const sd_capture_t capture = sd_card_capture_begin(NULL, &captured_calibration);
  if (!sd_optional_telemetry_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    { sd_card_capture_fault(capture); sd_card_capture_release(capture); return SD_CARD_STATUS_BACKPRESSURE; }
  }
  const int32_t want = seds_pkt_to_string_len(pkt);
  char value[SD_LINE_MAX], line[SD_LINE_MAX];
  if (want < 0 || (size_t)want >= sizeof(value) ||
      seds_pkt_to_string(pkt, value, (size_t)want + 1U) != SEDS_OK) { sd_card_capture_fault(capture); sd_card_capture_release(capture); return SD_CARD_STATUS_IO_ERROR; }
  const uint64_t uptime = board_uptime_ticks(), utc = daq_log_unix_ms();
  const int len = sd_format_csv_record(line, sizeof(line), utc, uptime, "seds_packet", value, sd_next_row_sequence());
  if (len < 0) { sd_card_capture_fault(capture); sd_card_capture_release(capture); return SD_CARD_STATUS_IO_ERROR; }
  sd_line_slot_t *slot = sd_alloc_slot((uint16_t)len);
  if (!slot) { g_sd_line_drop_count++; { sd_card_capture_fault(capture); sd_card_capture_release(capture); return SD_CARD_STATUS_BACKPRESSURE; } }
  slot->calibration = captured_calibration;
  slot->capture = capture; sd_card_capture_retain(capture);
  slot->session = sd_run_snapshot(); slot->len = (uint16_t)len;
  memcpy(slot->line, line, (size_t)len + 1U);
  { sd_card_capture_release(capture); return sd_enqueue_line(slot, utc); }
}

#endif

sd_card_status_t sd_card_log_pressure_batch(const pressure_sd_batch_t *batch)
{
  if (!batch) return SD_CARD_STATUS_IO_ERROR;
  /* The writer updates the closed latch at its monotonic launch deadline.
   * Never take router-clock or SD-pool locks from the receive callback. */
  if (!g_sd_services_initialized) {
    g_sd_pressure_records_dropped += PRESSURE_SD_SAMPLES;
    return SD_CARD_STATUS_BUSY;
  }
  union {
    sd_pressure_message_t message;
    ULONG words[SD_PRESSURE_MESSAGE_WORDS];
  } queued = {0};
  queued.message.batch = *batch;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t route = sd_run_snapshot();
  const uint64_t launch = sd_launch_key_locked();
  uint32_t capture = sd_pressure_capture_route(batch, launch, g_sd_close_unix_ms);
  /* Classify by VB capture time when both clocks are known; otherwise use the
   * receive-time route, exactly like DAQ-captured rows. */
  if (launch != 0U && launch <= UINT32_MAX && (capture & SD_PRESSURE_ROUTE_UNKNOWN_UTC) == 0U)
    capture |= SD_PRESSURE_ROUTE_CLASSIFIED;
  else capture &= SD_PRESSURE_ROUTE_UNKNOWN_UTC;
  queued.message.capture_route = capture;
  queued.message.launch_key = (uint32_t)launch;
  queued.message.route_key = route <= UINT32_MAX ? (uint32_t)route : UINT32_MAX;
  queued.message.route_epoch = g_sd_route_epoch;
  const UINT result = tx_queue_send(&g_sd_pressure_queue, queued.words, TX_NO_WAIT);
  if (result == TX_SUCCESS) sd_capture_retain_locked(queued.message.route_epoch);
  else sd_card_capture_fault(queued.message.route_epoch);
  __set_PRIMASK(mask);
  if (result != TX_SUCCESS) {
    g_sd_pressure_records_dropped += PRESSURE_SD_SAMPLES;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  g_sd_pressure_records_enqueued += PRESSURE_SD_SAMPLES;
  return SD_CARD_STATUS_OK;
}

static sd_card_status_t sd_card_enqueue_csv_row_captured_impl(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value,
                                         const daq_calibration_t *calibration, sd_capture_t capture)
{
  /* The bounded row queue can accept startup records while the writer mounts
   * the card. Do not discard the first network row merely because mounting
   * runs in another task. A missing/slow card still produces backpressure
   * when the fixed block pool fills; the general-purpose heap is not used. */
  if ((g_sd_services_initialized == 0U) || (sensor_name == NULL))
  {
    return SD_CARD_STATUS_BUSY;
  }
  const int scalar_sensor = sd_scalar_sensor_index(sensor_name);
  if (scalar_sensor < 0 && !sd_required_sensor_name(sensor_name) && !sd_optional_telemetry_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  char line[SD_LINE_MAX];
  daq_calibration_t captured_calibration;
  if (calibration != NULL) captured_calibration = *calibration;
  else (void)sd_calibration_snapshot(&captured_calibration);
  const uint64_t session = sd_run_snapshot();
  char value_text[24];
  const uint64_t age = (uint32_t)(daq_log_monotonic_ms() - timestamp_ms);
  const uint64_t now = board_uptime_ticks();
  const uint64_t uptime = now >= age ? now - age : 0U;
  const uint64_t network_ms = daq_sample_network_ms(daq_log_unix_ms(), now, uptime);
  if (scalar_sensor >= 0)
    return sd_enqueue_scalar((uint8_t)scalar_sensor, network_ms, uptime, value, &captured_calibration, capture);
  sd_format_float(value_text, value);
  const int len = sd_format_csv_record(line, sizeof(line), network_ms, uptime,
                                       sensor_name, value_text, sd_next_row_sequence());
  if ((len <= 0) || ((size_t)len >= sizeof(line)))
  {
    return SD_CARD_STATUS_IO_ERROR;
  }
  sd_line_slot_t *slot = sd_alloc_slot((uint16_t)len);
  if (slot == NULL) {
    ++g_sd_line_drop_count;
    if (sd_required_sensor_name(sensor_name)) ++g_sd_essential_telemetry_dropped;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->calibration = captured_calibration;
  slot->capture = capture; sd_card_capture_retain(capture);
  slot->session = session;
  memcpy(slot->line, line, (size_t)len + 1U);
  slot->len = (uint16_t)len;
  const sd_card_status_t result = sd_enqueue_line(slot, network_ms);
  if (result != SD_CARD_STATUS_OK && sd_required_sensor_name(sensor_name))
    ++g_sd_essential_telemetry_dropped;
  return result;
}

static sd_card_status_t sd_card_enqueue_optional_snapshot_captured_impl(
    const float values[SD_OPTIONAL_SNAPSHOT_ROWS], uint64_t timestamp_ms,
    const daq_calibration_t *calibration, sd_capture_t capture)
{
  if (!values || !calibration) return SD_CARD_STATUS_IO_ERROR;
  unsigned slot = SD_OPTIONAL_SNAPSHOT_SLOTS;
  uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (g_sd_services_initialized) {
    for (unsigned i = 0U; i < SD_OPTIONAL_SNAPSHOT_SLOTS; ++i) {
      if ((g_sd_optional_snapshot_used & (1UL << i)) != 0U) continue;
      slot = i; g_sd_optional_snapshot_used |= 1UL << slot;
      ++g_sd_optional_snapshot_slots_used; break;
    }
  }
  if (slot == SD_OPTIONAL_SNAPSHOT_SLOTS) {
    g_sd_optional_telemetry_dropped += SD_OPTIONAL_SNAPSHOT_ROWS;
    g_sd_line_drop_count += SD_OPTIONAL_SNAPSHOT_ROWS;
    ++g_sd_optional_snapshots_refused;
  }
  const uint8_t initialized = g_sd_services_initialized;
  __set_PRIMASK(mask);
  if (slot == SD_OPTIONAL_SNAPSHOT_SLOTS)
    return initialized ? SD_CARD_STATUS_BACKPRESSURE : SD_CARD_STATUS_BUSY;

  sd_optional_snapshot_t *owner = &g_sd_optional_snapshots[slot];
  /* No formatting, heap/FileX or blocking waits in this producer path. */
  memcpy(owner->values, values, sizeof(owner->values));
  owner->calibration = *calibration; owner->capture = capture;
  sd_card_capture_retain(capture); owner->len = SD_OPTIONAL_SNAPSHOT_TAG;
  const uint64_t age = (uint32_t)(daq_log_monotonic_ms() - timestamp_ms);
  const uint64_t now = board_uptime_ticks();
  owner->uptime = now >= age ? now - age : 0U;
  owner->network_ms = daq_sample_network_ms(daq_log_unix_ms(), now, owner->uptime);
  const ULONG message = (ULONG)(uintptr_t)owner;
  mask = __get_PRIMASK(); __disable_irq();
  /* Stamp routing/sequence in the SAME transaction as FIFO admission: a
   * concurrent launch transition cannot enqueue an old epoch behind a new one. */
  owner->session = sd_capture_key_locked(sd_run_snapshot(), owner->network_ms);
  owner->route_epoch = g_sd_route_epoch;
  owner->sequence_base = sd_row_sequence + 1U;
  const UINT result = tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT);
  if (result == TX_SUCCESS) {
    sd_row_sequence += SD_OPTIONAL_SNAPSHOT_ROWS;
    g_sd_scalar_append_tail = NULL;
    g_sd_optional_scalar_records_enqueued += SD_OPTIONAL_SNAPSHOT_ROWS;
    g_sd_optional_scalar_records_coalesced += SD_OPTIONAL_SNAPSHOT_ROWS - 1U;
  } else {
    sd_card_capture_release(capture);
    g_sd_optional_snapshot_used &= ~(1UL << slot);
    --g_sd_optional_snapshot_slots_used;
    g_sd_optional_telemetry_dropped += SD_OPTIONAL_SNAPSHOT_ROWS;
    g_sd_line_drop_count += SD_OPTIONAL_SNAPSHOT_ROWS;
    ++g_sd_optional_snapshots_refused;
  }
  __set_PRIMASK(mask);
  return result == TX_SUCCESS ? SD_CARD_STATUS_OK : SD_CARD_STATUS_BACKPRESSURE;
}

/* Caller holds the IRQ lock. Raw rows arrive in capture order, but about 0.7%
 * carry a capture UTC 1-2 ms earlier than the row before. The raw stream never
 * returns to an earlier key, so the boundary falls at one sequence number (the
 * first row whose UTC crosses) and each file holds one contiguous raw run. */
static uint64_t sd_raw_key_locked(uint64_t route, uint64_t capture_unix_ms, uint8_t commit)
{
  uint64_t key = sd_capture_key_locked(route, capture_unix_ms);
  if (key < g_sd_raw_key_floor) key = g_sd_raw_key_floor;
  if (commit) g_sd_raw_key_floor = key;
  return key;
}

static sd_card_status_t sd_card_enqueue_raw_adc_samples_captured_impl(const sd_raw_adc_record_t *samples,
                                                 uint16_t count,
                                                 const daq_calibration_t *calibration, sd_capture_t capture)
{
  if ((g_sd_services_initialized == 0U) || !samples || !calibration || count == 0U || count > SD_RAW_BATCH_MAX) {
    if (samples && count && count <= SD_RAW_BATCH_MAX) sd_raw_count_dropped(samples, count, 0U);
    return SD_CARD_STATUS_BUSY;
  }
  /* A batch straddling T0 or T+20 s is split so each sample lands in the file
   * of its own capture time; samples are in capture order. */
  if (count > 1U) {
    uint16_t split = 1U;
    const uint32_t split_mask = __get_PRIMASK(); __disable_irq();
    const uint64_t route = sd_run_snapshot();
    const uint64_t first = sd_raw_key_locked(route, samples[0].network_unix_ms, 0U);
    while (split < count && sd_raw_key_locked(route, samples[split].network_unix_ms, 0U) <= first) ++split;
    __set_PRIMASK(split_mask);
    if (split < count) {
      const sd_card_status_t head = sd_card_enqueue_raw_adc_samples_captured_impl(samples, split, calibration, capture);
      const sd_card_status_t tail = sd_card_enqueue_raw_adc_samples_captured_impl(samples + split, (uint16_t)(count - split), calibration, capture);
      return head != SD_CARD_STATUS_OK ? head : tail;
    }
  }
  const uint8_t optional = sd_raw_optional_admitted();
  uint16_t retained = 0U;
  for (uint16_t i = 0U; i < count; ++i)
    if (optional || sd_raw_is_essential(&samples[i])) ++retained;
  if (!optional) sd_raw_count_dropped(samples, count, 1U);
  if (retained == 0U) return SD_CARD_STATUS_BACKPRESSURE;

  uint16_t index = 0U, appended = 0U;
  const uint64_t uptime = board_uptime_ticks();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t session = sd_raw_key_locked(sd_run_snapshot(), samples[0].network_unix_ms, 1U);
  const uint32_t epoch = g_sd_route_epoch;
  sd_raw_slot_t *tail = g_sd_raw_append_tail;
  if (tail && tail->session == session && tail->route_epoch == epoch && tail->capture == capture &&
      memcmp(&tail->calibration, calibration, sizeof(*calibration)) == 0) {
    while (index < count && tail->count < tail->capacity) {
      if (optional || sd_raw_is_essential(&samples[index])) {
        tail->samples[tail->count++] = samples[index]; ++appended;
      }
      ++index;
    }
    /* Existing records project backwards from the newest appended uptime,
     * including a32-bit HAL tick wrap. Every sample keeps its original time. */
    if (appended) tail->uptime_reference = uptime;
    g_sd_raw_records_coalesced += appended;
    g_sd_raw_records_enqueued_count += appended;
  }
  __set_PRIMASK(mask);
  if (appended == retained) return retained == count ? SD_CARD_STATUS_OK : SD_CARD_STATUS_BACKPRESSURE;

  /* Reserve a full batch only when a new queued tail is needed. Under a
   * flush backlog, later acquisition calls fill it without repeating the
   * calibration/session/pool header or allocating another tiny slot. */
  sd_raw_slot_t *slot = sd_alloc_raw_slot(SD_RAW_SLOT_CAPACITY);
  if (!slot) {
    ++g_sd_raw_batch_drop_count;
    for (uint16_t i = index; i < count; ++i)
      if (optional || sd_raw_is_essential(&samples[i])) sd_raw_count_dropped(&samples[i], 1U, 0U);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->uptime_reference = uptime;
  slot->calibration = *calibration;
  slot->capture = capture; sd_card_capture_retain(capture);
  slot->session = session; slot->route_epoch = epoch; slot->count = 0U;
  for (; index < count; ++index)
    if (optional || sd_raw_is_essential(&samples[index])) slot->samples[slot->count++] = samples[index];
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t queued_mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_send(&g_sd_raw_queue, (VOID *)&message, TX_NO_WAIT);
  if (result == TX_SUCCESS) {
    g_sd_raw_append_tail = slot;
    g_sd_raw_records_enqueued_count += slot->count;
  }
  __set_PRIMASK(queued_mask);
  if (result != TX_SUCCESS) {
    ++g_sd_raw_batch_drop_count;
    sd_raw_count_dropped(slot->samples, slot->count, 0U);
    sd_card_capture_release(capture); sd_free_raw_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return retained == count ? SD_CARD_STATUS_OK : SD_CARD_STATUS_BACKPRESSURE;
}

sd_card_status_t sd_card_request_retry(uint32_t token)
{
  if (token == 0U) return SD_CARD_STATUS_IO_ERROR;
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  for (unsigned i = 0U; i < 8U; ++i) if (g_sd_retry_recent[i] == token) {
    __set_PRIMASK(saved); return SD_CARD_STATUS_OK;
  }
  if (g_sd_retry_pending_token || g_sd_retry_active_token) {
    __set_PRIMASK(saved); return SD_CARD_STATUS_BUSY;
  }
  g_sd_retry_recent[g_sd_retry_recent_next++ % 8U] = token;
  g_sd_retry_accepted_token = token; g_sd_retry_pending_token = token;
  __set_PRIMASK(saved);
  return SD_CARD_STATUS_OK;
}

static uint32_t sd_progress_age(uint64_t now, uint64_t last, uint8_t valid)
{
  if (!valid) return UINT32_MAX;
  const uint64_t age = now >= last ? now - last : 0U;
  return age >= UINT32_MAX ? UINT32_MAX - 1U : (uint32_t)age;
}

void sd_card_health_snapshot(uint32_t words[8])
{
  if (!words) return;
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t now = board_uptime_ticks();
  words[0] = (g_sd_services_initialized ? 1U : 0U) | (g_sd_ready ? 2U : 0U) |
      (g_sd_launch_closed ? 4U : 0U) |
      ((g_sd_retry_pending_token || g_sd_retry_active_token) ? 8U : 0U);
  words[1] = sd_progress_age(now, g_sd_last_sector_write_ms, g_sd_write_progress_valid);
  words[2] = sd_progress_age(now, g_sd_last_media_flush_ms, g_sd_flush_progress_valid);
  words[3] = g_sd_init_stage; words[4] = g_sd_retry_count;
  words[5] = g_sd_retry_accepted_token; words[6] = g_sd_retry_completed_token;
  words[7] = g_sd_init_failures;
  __set_PRIMASK(saved);
}

/* Read-only diagnostics: never query SDK UTC or advance launch routing here. */
void sd_card_launch_log_snapshot(uint32_t words[7])
{
  if (!words) return;
#ifdef TELEMETRY_ENABLED
  /* Read revision before the pair: an interleaved update can only produce an
   * older revision with a newer pair, never claim a revision before its apply. */
  const uint64_t revision = daq_log_clock_revision();
#else
  const uint64_t revision = 0U;
#endif
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t session = g_sd_session, deadline = g_sd_close_unix_ms;
  words[0] = (uint32_t)session; words[1] = (uint32_t)(session >> 32U);
  words[2] = (uint32_t)deadline; words[3] = (uint32_t)(deadline >> 32U);
  words[4] = (uint32_t)revision; words[5] = (uint32_t)(revision >> 32U);
  words[6] = (g_sd_services_initialized ? 1U : 0U) | (g_sd_ready ? 2U : 0U) |
      (g_sd_deadline_anchored ? 4U : 0U) | (g_sd_launch_closed ? 8U : 0U) |
      ((session && g_sd_launch_log.open) ? 48U : 0U) |
      (g_sd_launch_flush_pending ? 64U : 0U);
  __set_PRIMASK(saved);
}

UINT sd_card_request_flush(void)
{
  return tx_event_flags_set(&g_sd_flags, SD_FLAG_FLUSH_REQUEST, TX_OR);
}

UINT sd_card_notify_power_loss(void)
{
  return tx_event_flags_set(&g_sd_flags, SD_FLAG_POWER_LOSS, TX_OR);
}

UINT sd_card_is_ready(void)
{
  return (UINT)g_sd_ready;
}

void sd_card_set_calibration(const daq_calibration_t *calibration)
{
  if (calibration == NULL) return;
  const uint8_t finished = sd_launch_finished();
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint8_t changed = memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0;
  if (changed) { g_sd_calibration = *calibration; ++g_sd_calibration_generation; }
  __set_PRIMASK(saved);
  if (changed && finished) sd_card_set_launch_clock(0U, 0U);
}

/* Refusal belongs to the producer owner, not to callers remembering an
 * extra notification. A failed old-cohort handoff cannot become a successful
 * drain merely because its final reservation was correctly retired. */
sd_card_status_t sd_card_enqueue_csv_row_captured(const char *name, uint64_t timestamp,
    float value, const daq_calibration_t *calibration, sd_capture_t capture)
{
  const sd_card_status_t result = sd_card_enqueue_csv_row_captured_impl(name, timestamp, value, calibration, capture);
  if (result != SD_CARD_STATUS_OK) sd_card_capture_fault(capture);
  return result;
}
sd_card_status_t sd_card_enqueue_raw_adc_samples_captured(const sd_raw_adc_record_t *samples,
    uint16_t count, const daq_calibration_t *calibration, sd_capture_t capture)
{
  const sd_card_status_t result = sd_card_enqueue_raw_adc_samples_captured_impl(samples, count, calibration, capture);
  if (result != SD_CARD_STATUS_OK) sd_card_capture_fault(capture);
  return result;
}
sd_card_status_t sd_card_enqueue_optional_snapshot_captured(const float values[SD_OPTIONAL_SNAPSHOT_ROWS],
    uint64_t timestamp, const daq_calibration_t *calibration, sd_capture_t capture)
{
  const sd_card_status_t result = sd_card_enqueue_optional_snapshot_captured_impl(values, timestamp, calibration, capture);
  if (result != SD_CARD_STATUS_OK) sd_card_capture_fault(capture);
  return result;
}
/* Compatibility entrypoints acquire a lease before formatting/allocation. */
sd_card_status_t sd_card_enqueue_csv_row(const char *name, uint64_t timestamp, float value,
                                        const daq_calibration_t *provided)
{
  daq_calibration_t calibration;
  const sd_capture_t capture = sd_card_capture_begin(provided, &calibration);
  const sd_card_status_t result = sd_card_enqueue_csv_row_captured(name, timestamp, value, &calibration, capture);
  sd_card_capture_release(capture); return result;
}
sd_card_status_t sd_card_enqueue_raw_adc_samples(const sd_raw_adc_record_t *samples, uint16_t count,
                                                 const daq_calibration_t *provided)
{
  daq_calibration_t calibration;
  const sd_capture_t capture = sd_card_capture_begin(provided, &calibration);
  const sd_card_status_t result = sd_card_enqueue_raw_adc_samples_captured(samples, count, &calibration, capture);
  sd_card_capture_release(capture); return result;
}
sd_card_status_t sd_card_enqueue_optional_snapshot(const float values[SD_OPTIONAL_SNAPSHOT_ROWS], uint64_t timestamp,
                                                   const daq_calibration_t *provided)
{
  daq_calibration_t calibration;
  const sd_capture_t capture = sd_card_capture_begin(provided, &calibration);
  const sd_card_status_t result = sd_card_enqueue_optional_snapshot_captured(values, timestamp, &calibration, capture);
  sd_card_capture_release(capture); return result;
}
