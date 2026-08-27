#pragma once

#include "sedsnet_config.h"
#include <stddef.h>
#include <stdint.h>

/* Registers the HASH-backed provider with SEDSNet. Keys remain board-owned and
 * must be provisioned by startup code or a secure provisioning workflow. */
SedsResult sedsnet_hardware_crypto_init(void);
SedsResult sedsnet_hardware_crypto_register_key(uint32_t key_id,
                                                const uint8_t key[32]);
void sedsnet_hardware_crypto_clear_keys(void);

