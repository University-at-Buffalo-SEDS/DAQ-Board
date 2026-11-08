#ifndef SD_FILE_STREAM_H
#define SD_FILE_STREAM_H
/* Low-priority SD file download service. The router callback only copies one
 * validated request into a single RAM slot; the SD writer performs at most one
 * bounded FileX operation for it during an otherwise idle writer turn; the
 * telemetry thread then paces the answer frames out, one frame at a time and
 * only while the router has no other queued work (or after a long gap). */
#include "sd_file_wire.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t status;
    uint8_t flags;          /* LIST: SD_FILE_ENTRY_* */
    uint32_t file_size;
    uint32_t offset;        /* READ: absolute offset of data[0] */
    uint16_t data_len;      /* READ: bytes in the shared block buffer */
    uint16_t data_start;    /* READ: first valid byte in the block buffer */
    uint8_t name_len;       /* LIST: name bytes at the start of the block buffer */
} sd_file_result_t;

/* SD writer side. Claim moves a pending request to busy; finish publishes the
 * answer only if the stream session that asked is still current. */
bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket);
uint8_t *sd_file_stream_block(void);
/* Writer-owned read handle storage inside the same on-demand workspace. The
 * writer reports while it holds that handle open; storage is never released
 * while a handle is open or a request is BUSY. */
void *sd_file_stream_handle(void);
void sd_file_stream_handle_open(bool open);
/* Provided by the SD writer: bytes of its read-handle object. */
size_t sd_card_download_handle_size(void);
void sd_file_stream_finish(uint32_t ticket, const sd_file_result_t *result);
/* True while a session exists; lets the writer close its read handle early. */
bool sd_file_stream_session_active(void);
/* Flight-state gate shared by the router callback and the SD writer. */
bool sd_file_stream_state_allowed(void);

#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
SedsResult sd_file_stream_init(SedsRouter *router);
void sd_file_stream_poll(void);
#endif
#endif
