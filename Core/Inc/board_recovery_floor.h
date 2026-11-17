#ifndef BOARD_RECOVERY_FLOOR_H
#define BOARD_RECOVERY_FLOOR_H
#include <stdint.h>
/* No implicit clear. Corrupt/storage-failed floor rejects commands; QUERY lives.
 * Caller must separately validate freshness and source. Every time here is a
 * station stamp as a board_stamp key (time_epoch, timestamp): the persisted
 * floor record keeps its 16-byte layout with the epoch in the top 16 bits of
 * the stamp, so a record written before epochs existed reads as epoch 0. */
uint8_t board_recovery_floor_active(void);
/* May perform storage access: never call under control-owner mutex. */
uint8_t board_recovery_floor_read_cutoff(uint64_t *cutoff_stamp);
uint8_t board_recovery_floor_allows_recovery_time(uint64_t issuer_session, uint64_t source_stamp);
uint8_t board_recovery_floor_is_healthy(void);
uint8_t board_recovery_command_time_allowed(uint64_t source_stamp);
/* Explicit authorized RECOVER only, after boot/epoch/nonce/fresh-time checks.
 * Repairs require a strictly later (epoch, timestamp) than every verified/
 * attempted stamp, across all issuers: a request after a network time step
 * (a newer epoch) is later whatever its timestamp, and a replay stamped before
 * the floor never is; changing issuer never clears or lowers the floor.
 * Returns 1 only after persistent write + exact readback; must precede latches. */
uint8_t board_recovery_floor_commit(uint64_t issuer_session, uint64_t source_stamp);
uint8_t board_recovery_floor_accept_bootstrap(uint64_t original_source_stamp);
uint8_t board_recovery_floor_note_bootstrap_attempt(uint64_t original_source_stamp);
uint8_t board_recovery_floor_prime_boot(void);
uint8_t board_recovery_floor_cached_health(void);
uint8_t board_recovery_floor_cached_cutoff(uint64_t *cutoff);
#endif
