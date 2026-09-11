#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sedsnet_config.h"
#include "tx_api.h"
#include "daq_calibration.h"

typedef enum
{
  SD_CARD_STATUS_OK = 0,
  SD_CARD_STATUS_BUSY = 1,
  SD_CARD_STATUS_BACKPRESSURE = 2,
  SD_CARD_STATUS_IO_ERROR = 3
} sd_card_status_t;

typedef struct
{
  uint64_t network_unix_ms;
  uint32_t monotonic_ms;
  int32_t raw_adc_code;
  float raw_value;
  float calibrated_value;
} sd_raw_adc_record_t;

UINT sd_card_init(TX_BYTE_POOL *byte_pool);
void sd_card_set_hardware_ready(uint8_t ready);
void sd_card_writer_thread_entry(ULONG initial_input);
sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt);
sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value,
                                         const daq_calibration_t *calibration);
sd_card_status_t sd_card_enqueue_raw_adc_samples(
    const sd_raw_adc_record_t *samples, uint16_t count,
    const daq_calibration_t *calibration);
UINT sd_card_request_flush(void);
UINT sd_card_notify_power_loss(void);
UINT sd_card_is_ready(void);
void sd_card_set_calibration(const daq_calibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
