#include "sd_file_stream.h"

#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"
#include <string.h>
#if defined(TELEMETRY_ENABLED) && defined(TELEMETRY_USE_TLSF)
#include "telemetry_tlsf.h"
#endif

/* Single slot, three owners in turn: router callback (IDLE->REQUESTED),
 * SD writer (REQUESTED->BUSY->READY) and telemetry poll (READY->IDLE).
 * Transitions use a short IRQ mask; no FileX, SDK or clock calls under it. */
typedef enum { SLOT_IDLE = 0, SLOT_REQUESTED, SLOT_BUSY, SLOT_READY } sd_file_slot_t;

/* Pace answer frames: send immediately only when the router is otherwise
 * idle, otherwise wait out a long gap so a busy bus still makes progress.
 * The minimum spacing caps the service at roughly 50 KB/s of payload. */
#define SD_FILE_MIN_FRAME_MS 2U
#define SD_FILE_BUSY_FRAME_MS 25U

static struct {
    volatile sd_file_slot_t slot;
    uint32_t generation;           /* advanced on every session change */
    uint32_t request_generation;
    sd_file_request_t request;
    sd_file_result_t result;
    uint16_t sent;                 /* READ bytes already framed */
#ifdef TELEMETRY_ENABLED
    SedsRouter *router;
    uint32_t stream_id, peer_stream_id, peer_address;
    uint16_t peer_port;
    char peer_name[16];
    size_t peer_name_len;
    bool connected;
    uint32_t last_frame_ms;
#endif
} s;
/* The block exists only while a station session exists (Idle/Landed/Recovery
 * downloads); it is borrowed from the SEDSnet heap with its normal admission
 * reserve rather than adding static RAM. The writer touches it only while it
 * owns a BUSY request; it is released once no session or BUSY owner remains. */
static uint8_t *s_block;
static volatile bool s_handle_open;
#define SD_FILE_HANDLE_OFFSET SD_FILE_BLOCK_BYTES

volatile uint32_t g_sd_file_requests, g_sd_file_frames, g_sd_file_refused, g_sd_file_aborts;

static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }

bool sd_file_stream_state_allowed(void)
{
    return sd_file_state_allowed(flight_state_cache_has_value(), flight_state_cache_value());
}

bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket)
{
    if (!request || !ticket) return false;
    const uint32_t p = lock();
    const bool claimed = s.slot == SLOT_REQUESTED;
    if (claimed) {
        *request = s.request; *ticket = s.request_generation; s.slot = SLOT_BUSY;
    }
    unlock(p);
    return claimed;
}

uint8_t *sd_file_stream_block(void) { return s_block; }
void *sd_file_stream_handle(void) { return s_block ? s_block + SD_FILE_HANDLE_OFFSET : NULL; }
void sd_file_stream_handle_open(bool open) { s_handle_open = open; }

void sd_file_stream_finish(uint32_t ticket, const sd_file_result_t *result)
{
    const uint32_t p = lock();
    if (s.slot == SLOT_BUSY) {
        const bool current = result && ticket == s.generation && ticket == s.request_generation;
        if (current) {
            s.result = *result; s.sent = 0U; s.slot = SLOT_READY;
        } else {
            s.slot = SLOT_IDLE;
        }
    }
    unlock(p);
}

bool sd_file_stream_session_active(void)
{
#ifdef TELEMETRY_ENABLED
    return s.connected;
#else
    return false;
#endif
}

#ifdef TELEMETRY_ENABLED
static bool acquire_block(void)
{
    if (s_block) return true;
#ifdef TELEMETRY_USE_TLSF
    const size_t bytes = SD_FILE_HANDLE_OFFSET + sd_card_download_handle_size();
    if (!telemetry_tlsf_admit(bytes + 4096U, bytes)) return false;
    s_block = telemetry_tlsf_malloc(bytes);
#endif
    return s_block != NULL;
}

/* Telemetry thread only: no session can create a request, so a non-BUSY slot
 * has no other owner of the block. */
static void release_block_if_unused(void)
{
    if (!s_block || s.connected) return;
    const uint32_t p = lock();
    uint8_t *block = s.slot == SLOT_BUSY || s_handle_open ? NULL : s_block;
    if (block) s_block = NULL;
    unlock(p);
#ifdef TELEMETRY_USE_TLSF
    if (block) telemetry_tlsf_free(block);
#endif
}

/* Drops any queued answer; a busy writer turn finishes into IDLE. */
static void end_session_locked(void)
{
    s.connected = false;
    ++s.generation;
    if (s.slot != SLOT_BUSY) s.slot = SLOT_IDLE;
}

static SedsResult send_frame(const uint8_t *frame, size_t len)
{
    if (!len) return SEDS_ERR;
    const SedsResult r = seds_router_send_p2p_stream(s.router, s.stream_id, frame, len);
    if (r == SEDS_OK) { ++g_sd_file_frames; s.last_frame_ms = HAL_GetTick(); }
    return r;
}

static SedsResult reply_status(uint32_t stream, uint8_t op, uint8_t status)
{
    uint8_t frame[SD_FILE_DATA_HEADER];
    size_t len;
    if (op == SD_FILE_OP_READ) len = sd_file_encode_data(frame, sizeof frame, status, SD_FILE_DATA_LAST, 0U, 0U, NULL, 0U);
    else if (op == SD_FILE_OP_LIST) len = sd_file_encode_list(frame, sizeof frame, status, 0U, 0U, 0U, 0U, NULL, 0U);
    else len = sd_file_encode_status(frame, sizeof frame, op, status);
    if (status != SD_FILE_STATUS_OK) ++g_sd_file_refused;
    return seds_router_send_p2p_stream(s.router, stream, frame, len);
}

static bool authorized(const SedsP2pStreamEventView *e)
{
    return e && e->stream_id && e->local_port == SD_FILE_STREAM_PORT && e->peer_hostname &&
           e->peer_hostname_len && e->peer_hostname_len <= sizeof s.peer_name &&
           seds_router_sender_matches_hostname(s.router, e->peer_hostname, e->peer_hostname_len, "GS", 2U);
}

static bool same_session(const SedsP2pStreamEventView *e)
{
    return s.connected && e->stream_id == s.stream_id && e->peer_stream_id == s.peer_stream_id &&
           e->peer_address == s.peer_address && e->peer_port == s.peer_port &&
           e->peer_hostname_len == s.peer_name_len && !memcmp(e->peer_hostname, s.peer_name, s.peer_name_len);
}

static SedsResult sd_file_stream_event(const SedsP2pStreamEventView *e, void *user)
{
    (void)user;
    if (!authorized(e)) return SEDS_BAD_ARG;
    if (e->kind == SEDS_P2P_STREAM_ACCEPTED) {
        if (!sd_file_stream_state_allowed() || !acquire_block()) { ++g_sd_file_refused; return SEDS_ERR; }
        const uint32_t p = lock();
        end_session_locked();
        s.stream_id = e->stream_id; s.peer_stream_id = e->peer_stream_id;
        s.peer_address = e->peer_address; s.peer_port = e->peer_port;
        memcpy(s.peer_name, e->peer_hostname, e->peer_hostname_len); s.peer_name_len = e->peer_hostname_len;
        s.connected = true;
        unlock(p);
        return SEDS_OK;
    }
    if (!same_session(e)) return SEDS_ERR;
    if (e->kind == SEDS_P2P_STREAM_CLOSED || e->kind == SEDS_P2P_STREAM_RESET) {
        const uint32_t p = lock(); end_session_locked(); unlock(p);
        return SEDS_OK;
    }
    if (e->kind != SEDS_P2P_STREAM_DATA || !e->payload || !e->payload_len) return SEDS_BAD_ARG;
    sd_file_request_t request;
    const uint8_t status = sd_file_parse_request(e->payload, e->payload_len, &request);
    const uint8_t op = e->payload[0];
    if (status != SD_FILE_STATUS_OK) return reply_status(e->stream_id, op, status);
    if (request.op == SD_FILE_OP_CANCEL) {
        const uint32_t p = lock();
        ++s.generation;                       /* stale any busy writer answer */
        if (s.slot != SLOT_BUSY) s.slot = SLOT_IDLE;
        unlock(p);
        return reply_status(e->stream_id, op, SD_FILE_STATUS_OK);
    }
    if (!sd_file_stream_state_allowed()) return reply_status(e->stream_id, op, SD_FILE_STATUS_BAD_STATE);
    const uint32_t p = lock();
    const bool idle = s.slot == SLOT_IDLE;
    if (idle) {
        s.request = request; s.request_generation = s.generation; s.slot = SLOT_REQUESTED;
    }
    unlock(p);
    if (!idle) return reply_status(e->stream_id, op, SD_FILE_STATUS_BUSY);
    ++g_sd_file_requests;
    return SEDS_OK;
}

SedsResult sd_file_stream_init(SedsRouter *router)
{
    if (!router) return SEDS_BAD_ARG;
    const uint32_t p = lock();
    s.router = router; end_session_locked();
    unlock(p);
    return seds_router_bind_p2p_stream_port(router, SD_FILE_STREAM_PORT, sd_file_stream_event, NULL);
}


/* Telemetry thread only. At most one frame per call. */
void sd_file_stream_poll(void)
{
    if (!s.router) return;
    if (s.connected && !sd_file_stream_state_allowed()) {
        /* Leaving Idle/Landed/Recovery aborts at once; the station resumes later. */
        const uint32_t stream = s.stream_id;
        const uint32_t p = lock(); end_session_locked(); unlock(p);
        ++g_sd_file_aborts;
        (void)seds_router_reset_p2p_stream(s.router, stream);
        return;
    }
    if (!s.connected) { release_block_if_unused(); return; }
    if (s.slot != SLOT_READY || !s_block) return;
    const uint32_t now = HAL_GetTick();
    const uint32_t gap = now - s.last_frame_ms;
    if (gap < SD_FILE_MIN_FRAME_MS) return;
    if (seds_router_has_queued_work(s.router) != 0 && gap < SD_FILE_BUSY_FRAME_MS) return;

    uint8_t frame[SD_FILE_FRAME_MAX];
    const sd_file_result_t *r = &s.result;
    bool done = true;
    size_t len;
    if (s.request.op == SD_FILE_OP_LIST) {
        len = sd_file_encode_list(frame, sizeof frame, r->status, s.request.dir, s.request.index,
                                  r->file_size, r->flags, (const char *)s_block, r->name_len);
    } else if (r->status != SD_FILE_STATUS_OK || r->data_len == 0U) {
        len = sd_file_encode_data(frame, sizeof frame, r->status, SD_FILE_DATA_LAST, r->offset, r->file_size, NULL, 0U);
    } else {
        const uint16_t remaining = (uint16_t)(r->data_len - s.sent);
        const uint16_t chunk = remaining > SD_FILE_DATA_MAX ? SD_FILE_DATA_MAX : remaining;
        done = chunk == remaining;
        len = sd_file_encode_data(frame, sizeof frame, r->status, done ? SD_FILE_DATA_LAST : 0U,
                                  r->offset + s.sent, r->file_size, s_block + r->data_start + s.sent, chunk);
        if (send_frame(frame, len) != SEDS_OK) return;  /* retry this frame next poll */
        s.sent = (uint16_t)(s.sent + chunk);
        if (done) { const uint32_t p = lock(); if (s.slot == SLOT_READY) s.slot = SLOT_IDLE; unlock(p); }
        return;
    }
    if (send_frame(frame, len) != SEDS_OK) return;
    const uint32_t p = lock(); if (s.slot == SLOT_READY) s.slot = SLOT_IDLE; unlock(p);
}
#endif
