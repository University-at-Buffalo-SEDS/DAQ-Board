#ifndef SD_FILE_STREAM_H
#define SD_FILE_STREAM_H
/* Low-priority SD transfer service v2 (GroundStation26
 * docs/backend/sd-transfer.md). The router callback queues one LIST/OPEN/HASH
 * request in a RAM slot and applies ACKs; the SD writer answers requests and
 * builds DATA messages (sd_xfer.c) from the card, one bounded step per
 * otherwise idle writer turn; the telemetry thread sends answers and paces
 * DATA out at the station's rate, behind any other queued router work. */
#include "sd_file_wire.h"
#include "sd_xfer.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* SD writer side. Claim moves a pending request to busy; answer publishes a
 * frame only if the stream session that asked is still current. The borrowed
 * area (512-byte block, read handle, transfer state and workspace) exists
 * only while a session or an open handle does; the writer touches it only
 * while it owns a BUSY request, holds the handle open or runs a transfer. */
bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket);
bool sd_file_stream_ticket_current(uint32_t ticket);
uint8_t *sd_file_stream_block(void);
void *sd_file_stream_handle(void);
void *sd_file_stream_scratch(void);          /* SHA-256 context while hashing */
/* The writer reports while it holds that handle open; storage is never
 * released while a handle is open or a request is BUSY. */
void sd_file_stream_handle_open(bool open);
/* Provided by the SD writer: bytes of its read-handle object. */
size_t sd_card_download_handle_size(void);
void sd_file_stream_answer(uint32_t ticket, const uint8_t *frame, size_t len);
/* OPEN: starts the transfer under the lock and answers it. */
void sd_file_stream_open(uint32_t ticket, const sd_xfer_open_t *open, uint32_t file_size, bool growing);
bool sd_file_stream_transfer_active(void);
/* Builds the next DATA message if one is due and the send slot is free;
 * true if it produced one. A read error ends the transfer. */
bool sd_file_stream_produce(sd_xfer_read_fn read, void *ctx, uint32_t now_ms);
/* True while a session exists; lets the writer close its read handle early. */
bool sd_file_stream_session_active(void);
/* Flight-state gate shared by the router callback and the SD writer. */
bool sd_file_stream_state_allowed(void);

#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
SedsResult sd_file_stream_init(SedsRouter *router);
/* Telemetry thread, router held: at most one frame per call. */
void sd_file_stream_poll(void);
#endif
#endif
