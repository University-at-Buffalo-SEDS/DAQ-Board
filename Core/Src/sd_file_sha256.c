/* LaunchCore's software SHA-256 under private names (see sd_file_sha256.h).
 * CMake puts LaunchCore's bootloader/src on this file's include path. */
#include "sd_file_sha256.h"
#define launchcore_sha256_init sd_file_sha256_init
#define launchcore_sha256_update sd_file_sha256_update
#define launchcore_sha256_final sd_file_sha256_final
#define launchcore_sha256 sd_file_sha256
void sd_file_sha256(const void *data, size_t len, uint8_t out[32]);
#include "sha256.c"
