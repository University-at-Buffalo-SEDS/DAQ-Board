#ifndef SD_FILE_SHA256_H
#define SD_FILE_SHA256_H
/* Software SHA-256 for the SD transfer HASH answer: LaunchCore's
 * bootloader/src/sha256.c compiled a second time under these names
 * (Core/Src/sd_file_sha256.c). The application's launchcore_sha256_* is the
 * HASH peripheral (Bootloader/sha256_hardware.c), one global hardware context
 * that OTA image checks and SEDSnet crypto re-initialise from the telemetry
 * thread; it cannot carry a file hash across SD writer turns. */
#include "launchcore/image.h"
#include <stddef.h>
#include <stdint.h>

void sd_file_sha256_init(launchcore_sha256_ctx_t *ctx);
void sd_file_sha256_update(launchcore_sha256_ctx_t *ctx, const void *data, size_t len);
void sd_file_sha256_final(launchcore_sha256_ctx_t *ctx, uint8_t out[32]);
#endif
