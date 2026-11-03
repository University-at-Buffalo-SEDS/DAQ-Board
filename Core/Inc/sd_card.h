#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "sedsnet_config.h"
#include "tx_api.h"
#include "daq_calibration.h"
#include "pressure_sd_wire.h"

typedef enum
{
  SD_CARD_STATUS_OK = 0,
  SD_CARD_STATUS_BUSY = 1,
  SD_CARD_STATUS_BACKPRESSURE = 2,
  SD_CARD_STATUS_IO_ERROR = 3
} sd_card_status_t;

/* A retained calibration capture reference. Internal producers transfer/clone
 * it into existing work/SD owners; it is not a timestamp or network identity. */
sd_capture_t sd_card_capture_begin(const daq_calibration_t *provided, daq_calibration_t *captured);
void sd_card_capture_retain(sd_capture_t capture);
void sd_card_capture_release(sd_capture_t capture);
void sd_card_capture_fault(sd_capture_t capture);
/* Sole writer/control adapter only; callers hold the short RAM gate. */
bool sd_card_thermal_seal(const float old_tuple[4]);
/* RAM publication only; requires the existing short owner gate and a sealed
 * cohort. This low-level operation is not permission to activate a request. */
bool sd_card_thermal_publish_metadata(const float expected[4], const float desired[4]);
bool sd_card_thermal_old_drained(void);
bool sd_card_thermal_flush_old(void);
void sd_card_thermal_finish(void);
void sd_card_thermal_cohort_snapshot(uint32_t words[4]);
typedef struct
{
  uint64_t network_unix_ms;
  uint32_t monotonic_ms;
  uint32_t record_sequence;
  int32_t raw_adc_code;
  float raw_value;
  float calibrated_value;
  float adc_temperature_c;
  int32_t adc_temperature_code;
  uint8_t channel;
} sd_raw_adc_record_t;

UINT sd_card_init(TX_BYTE_POOL *byte_pool);
void sd_card_set_hardware_ready(uint8_t ready);
void sd_card_writer_thread_entry(ULONG initial_input);
#ifdef TELEMETRY_ENABLED
sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt);
#endif
/* Copies a fixed batch without CSV, allocation or media I/O. OK means queued;
 * writer/flushed counters separately report storage progress. */
sd_card_status_t sd_card_log_pressure_batch(const pressure_sd_batch_t *batch);
sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value,
                                         const daq_calibration_t *calibration);
/* Nonblocking, bounded SD-only ownership of all24 optional snapshot rows.
 * Capture clocks/calibration/session/sequence once. OK means retained, not
 * persisted. A full ring refuses/counts24 rows once; network owners are free. */
#define SD_OPTIONAL_SNAPSHOT_ROWS 24U
sd_card_status_t sd_card_enqueue_optional_snapshot(
    const float values[SD_OPTIONAL_SNAPSHOT_ROWS], uint64_t timestamp_ms,
    const daq_calibration_t *calibration);
sd_card_status_t sd_card_enqueue_raw_adc_samples_captured(
    const sd_raw_adc_record_t *samples, uint16_t count,
    const daq_calibration_t *calibration, sd_capture_t capture);
sd_card_status_t sd_card_enqueue_csv_row_captured(
    const char *sensor_name, uint64_t timestamp_ms, float value,
    const daq_calibration_t *calibration, sd_capture_t capture);
sd_card_status_t sd_card_enqueue_optional_snapshot_captured(
    const float values[SD_OPTIONAL_SNAPSHOT_ROWS], uint64_t timestamp_ms,
    const daq_calibration_t *calibration, sd_capture_t capture);
sd_card_status_t sd_card_enqueue_raw_adc_samples(
    const sd_raw_adc_record_t *samples, uint16_t count,
    const daq_calibration_t *calibration);
/* Remote retry is deferred, deduplicated across the eight most recent tokens,
 * and never permits formatting. Completion means one attempt, not a good card. */
sd_card_status_t sd_card_request_retry(uint32_t token);
/* Flags, write/flush ages, init stage/retries, accepted/completed token, failures.
 * UINT32_MAX age means no successful progress observed in this boot. */
void sd_card_health_snapshot(uint32_t words[8]);
/* Actual storage session/deadline and receiver revision (each lo/hi), flags.
 * Read-only snapshot; no SDK calls or routing transitions under the IRQ lock. */
void sd_card_launch_log_snapshot(uint32_t words[7]);
UINT sd_card_request_flush(void);
UINT sd_card_notify_power_loss(void);
UINT sd_card_is_ready(void);
void sd_card_set_calibration(const daq_calibration_t *calibration);
/* Apply only after DAQ_LOG_CLOCK receiver revision validation. Local reset
 * may use (0,0); accepted pre-cutoff T0 revisions can adjust either direction. */
void sd_card_set_launch_clock(uint64_t session, uint64_t close_unix_ms);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
