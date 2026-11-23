/* Core/Src/sd_file_stream.c: DAQ SD transfer service v2, router/telemetry
 * side (GroundStation26 docs/backend/sd-transfer.md). Same wire as the FC. */
#include "sd_file_stream.h"

#include "flight_state_cache.h"
#include "launchcore/image.h"
#include "main.h"
#include "sedsnet_config.h"
#include <string.h>
#if defined(TELEMETRY_ENABLED) && defined(TELEMETRY_USE_TLSF)
#include "telemetry_tlsf.h"
#endif

/* Control slot, three owners in turn: router callback (IDLE->REQUESTED),
 * SD writer (REQUESTED->BUSY->READY) and telemetry poll (READY->IDLE).
 * Transitions use a short IRQ mask; no FileX, SDK or clock calls under it. */
typedef enum { SLOT_IDLE = 0, SLOT_REQUESTED, SLOT_BUSY, SLOT_READY } sd_file_slot_t;

/* Frames go out at once while the router is otherwise idle, else after a
 * 25 ms gap, so commands, telemetry and fill traffic go first. DATA is also
 * held to the station's rate (sd_xfer.c). */
#define SD_FILE_MIN_FRAME_MS 2U
#define SD_FILE_BUSY_FRAME_MS 25U

typedef union {
    sd_xfer_work_t xfer;
    launchcore_sha256_ctx_t sha;
} sd_file_work_t;

/* Borrowed from the SEDSnet heap (with its normal admission reserve) only
 * while a station session exists: a 512-byte block, the transfer state, its
 * workspace (or the HASH context) and the writer's FX_FILE read handle
 * (sd_card_download_handle_size() bytes, after the fixed part). */
typedef struct {
    uint8_t block[SD_FILE_BLOCK_BYTES];
    sd_xfer_t x;
    sd_file_work_t work;
    uint64_t handle[];
} sd_file_area_t;

static struct {
    volatile sd_file_slot_t slot;
    volatile uint8_t msg_ready;    /* DATA message handed to the telemetry thread */
    uint8_t answer_len, msg_xfer;
    uint16_t msg_len;
    uint32_t generation;           /* advanced on every session change */
    uint32_t request_generation;
    sd_file_request_t request;
    uint8_t answer[SD_FILE_ANSWER_MAX];
    const uint8_t *msg;
#ifdef TELEMETRY_ENABLED
    SedsRouter *router;
    uint32_t stream_id, peer_stream_id, peer_address;
    uint16_t peer_port;
    char peer_name[16];
    size_t peer_name_len;
    bool connected;
    uint32_t last_frame_ms, next_data_ms;
#endif
} s;
/* The SD writer touches the area only while it owns a BUSY request, holds
 * its handle open or runs the transfer; it is released once no session, BUSY
 * owner or open handle remains. */
static sd_file_area_t *s_area;
static volatile bool s_handle_open;

volatile uint32_t g_sd_file_requests, g_sd_file_frames, g_sd_file_refused, g_sd_file_aborts;
volatile uint32_t g_sd_file_data, g_sd_file_acks;

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

bool sd_file_stream_ticket_current(uint32_t ticket) { return ticket == s.generation && s.slot == SLOT_BUSY; }
uint8_t *sd_file_stream_block(void) { return s_area ? s_area->block : NULL; }
void *sd_file_stream_handle(void) { return s_area ? (void *)s_area->handle : NULL; }
void *sd_file_stream_scratch(void) { return s_area ? (void *)&s_area->work.sha : NULL; }
void sd_file_stream_handle_open(bool open) { s_handle_open = open; }
bool sd_file_stream_transfer_active(void) { return s_area && s_area->x.id; }

void sd_file_stream_answer(uint32_t ticket, const uint8_t *frame, size_t len)
{
    const uint32_t p = lock();
    if (s.slot == SLOT_BUSY) {
        if (frame && ticket == s.generation && ticket == s.request_generation && len && len <= sizeof s.answer) {
            memcpy(s.answer, frame, len); s.answer_len = (uint8_t)len; s.slot = SLOT_READY;
        } else {
            s.slot = SLOT_IDLE;
        }
    }
    unlock(p);
}

void sd_file_stream_open(uint32_t ticket, const sd_xfer_open_t *open, uint32_t file_size, bool growing)
{
    uint8_t ans[SD_XFER_OPEN_ANSWER] = {0};
#ifdef TELEMETRY_ENABLED
    const uint32_t now = HAL_GetTick();
    const uint32_t p = lock();
    if (s_area && open && ticket == s.generation && ticket == s.request_generation) {
        s.msg_ready = 0U;
        (void)sd_xfer_begin(&s_area->x, open, file_size, growing, now, ans);
    }
    unlock(p);
#else
    (void)open; (void)file_size; (void)growing;
#endif
    sd_file_stream_answer(ticket, ans, sizeof ans);
}

bool sd_file_stream_produce(sd_xfer_read_fn read, void *ctx, uint32_t now_ms)
{
#ifdef TELEMETRY_ENABLED
    if (!s_area || !s_area->x.id || s.msg_ready) return false;
    uint32_t p = lock();
    const sd_xfer_flow_t flow = s_area->x.flow;
    const uint8_t xfer = s_area->x.id;
    unlock(p);
    const uint8_t *out = NULL;
    const size_t n = sd_xfer_produce(&s_area->x, &s_area->work.xfer, &flow, read, ctx, now_ms, &out);
    if (n == 0U || n == SIZE_MAX) return false;
    p = lock();
    s.msg = out; s.msg_len = (uint16_t)n; s.msg_xfer = xfer; s.msg_ready = 1U;
    unlock(p);
    return true;
#else
    (void)read; (void)ctx; (void)now_ms;
    return false;
#endif
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
static bool acquire_area(void)
{
    if (s_area) return true;
#ifdef TELEMETRY_USE_TLSF
    const size_t bytes = sizeof(sd_file_area_t) + sd_card_download_handle_size();
    if (!telemetry_tlsf_admit(bytes + 4096U, bytes)) return false;
    s_area = telemetry_tlsf_malloc(bytes);
    if (s_area) s_area->x.id = 0U;
#endif
    return s_area != NULL;
}

/* Telemetry thread only: without a session no request can be created, so a
 * non-BUSY slot without an open handle leaves the area unowned. */
static void release_area_if_unused(void)
{
    if (!s_area || s.connected) return;
    const uint32_t p = lock();
    sd_file_area_t *area = s.slot == SLOT_BUSY || s_handle_open ? NULL : s_area;
    if (area) s_area = NULL;
    unlock(p);
#ifdef TELEMETRY_USE_TLSF
    if (area) telemetry_tlsf_free(area);
#endif
}

/* Drops any queued answer or message and ends the transfer; a busy writer
 * turn finishes into IDLE. Caller locks. */
static void end_transfer_locked(void)
{
    ++s.generation;
    if (s.slot != SLOT_BUSY) s.slot = SLOT_IDLE;
    s.msg_ready = 0U;
    if (s_area) s_area->x.id = 0U;
}

static void end_session_locked(void)
{
    s.connected = false;
    end_transfer_locked();
}

static SedsResult send_frame(const uint8_t *frame, size_t len)
{
    const SedsResult r = seds_router_send_p2p_stream(s.router, s.stream_id, frame, len);
    if (r == SEDS_OK) { ++g_sd_file_frames; s.last_frame_ms = HAL_GetTick(); }
    return r;
}

static SedsResult reply_status(uint32_t stream, const uint8_t *request, size_t len, uint8_t status)
{
    uint8_t frame[SD_FILE_ANSWER_MAX];
    const size_t n = sd_file_encode_status(frame, request[0], status);
    if (request[0] == SD_FILE_OP_OPEN && len > 1U) frame[2] = request[1];   /* xfer */
    if (status != SD_FILE_STATUS_OK) ++g_sd_file_refused;
    return seds_router_send_p2p_stream(s.router, stream, frame, n);
}

/* The GroundStation only (over the fill network). One session at a time: a
 * new ACCEPTED replaces the previous session. */
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
        if (!sd_file_stream_state_allowed() || !acquire_area()) { ++g_sd_file_refused; return SEDS_ERR; }
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
    const uint8_t op = e->payload[0];
    if (op == SD_FILE_OP_ACK) {
        uint8_t xfer;
        sd_xfer_flow_t flow;
        if (!sd_xfer_parse_ack(e->payload, e->payload_len, &xfer, &flow)) return SEDS_BAD_ARG;
        const uint32_t now = HAL_GetTick();
        const uint32_t p = lock();
        const bool applied = s_area && sd_xfer_ack(&s_area->x, xfer, &flow, now);
        unlock(p);
        if (applied) ++g_sd_file_acks;
        return SEDS_OK;
    }
    sd_file_request_t request;
    const uint8_t status = sd_file_parse_request(e->payload, e->payload_len, &request);
    if (status != SD_FILE_STATUS_OK) return reply_status(e->stream_id, e->payload, e->payload_len, status);
    if (op == SD_FILE_OP_CANCEL) {
        const uint32_t p = lock(); end_transfer_locked(); unlock(p);   /* stales a busy writer answer */
        return reply_status(e->stream_id, e->payload, e->payload_len, SD_FILE_STATUS_OK);
    }
    if (!sd_file_stream_state_allowed())
        return reply_status(e->stream_id, e->payload, e->payload_len, SD_FILE_STATUS_BAD_STATE);
    const uint32_t p = lock();
    const bool idle = s.slot == SLOT_IDLE;
    if (idle) { s.request = request; s.request_generation = s.generation; s.slot = SLOT_REQUESTED; }
    unlock(p);
    if (!idle) return reply_status(e->stream_id, e->payload, e->payload_len, SD_FILE_STATUS_BUSY);
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
    if (!s.connected) { release_area_if_unused(); return; }
    const uint32_t now = HAL_GetTick();
    const uint32_t gap = now - s.last_frame_ms;
    if (gap < SD_FILE_MIN_FRAME_MS) return;
    if (seds_router_has_queued_work(s.router) != 0 && gap < SD_FILE_BUSY_FRAME_MS) return;
    if (s.slot == SLOT_READY) {
        if (send_frame(s.answer, s.answer_len) != SEDS_OK) return;  /* retry next poll */
        const uint32_t p = lock(); if (s.slot == SLOT_READY) s.slot = SLOT_IDLE; unlock(p);
        return;
    }
    if (!s.msg_ready) return;
    if (!s_area || s.msg_xfer != s_area->x.id) { s.msg_ready = 0U; return; }  /* cancelled */
    const uint32_t rate = s_area->x.flow.rate;
    if (!rate || (int32_t)(now - s.next_data_ms) < 0) return;
    if (send_frame(s.msg, s.msg_len) != SEDS_OK) return;          /* retry next poll */
    s.next_data_ms = now + (uint32_t)(((uint64_t)s.msg_len * 1000U + rate - 1U) / rate);
    ++g_sd_file_data;
    s.msg_ready = 0U;
}
#endif
