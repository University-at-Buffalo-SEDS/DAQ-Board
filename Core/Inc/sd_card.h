#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sedsnet_config.h"
#include "tx_api.h"

typedef enum
{
  SD_CARD_STATUS_OK = 0,
  SD_CARD_STATUS_BUSY = 1,
  SD_CARD_STATUS_BACKPRESSURE = 2,
  SD_CARD_STATUS_IO_ERROR = 3
} sd_card_status_t;

UINT sd_card_init(TX_BYTE_POOL *byte_pool);
void sd_card_writer_thread_entry(ULONG initial_input);
sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt);
sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value);
UINT sd_card_request_flush(void);
UINT sd_card_notify_power_loss(void);
UINT sd_card_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
