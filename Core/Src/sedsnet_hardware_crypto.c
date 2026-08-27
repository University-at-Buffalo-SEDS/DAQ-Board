#include "sedsnet_hardware_crypto.h"

#include "launchcore/image.h"
#include <stdbool.h>
#include <string.h>

#define CRYPTO_KEY_SLOTS 4U
#define CRYPTO_TAG_SIZE 16U
#define CRYPTO_BLOCK_SIZE 32U

typedef struct {
  uint32_t id;
  uint8_t key[32];
  bool used;
} crypto_key_t;

static crypto_key_t keys[CRYPTO_KEY_SLOTS];

static const uint8_t *find_key(uint32_t id)
{
  for (size_t i = 0; i < CRYPTO_KEY_SLOTS; ++i)
    if (keys[i].used && keys[i].id == id) return keys[i].key;
  return NULL;
}

static void sha_chunks(const uint8_t *const *chunks, const size_t *lengths,
                       size_t count, uint8_t out[32])
{
  launchcore_sha256_ctx_t ctx;
  launchcore_sha256_init(&ctx);
  for (size_t i = 0; i < count; ++i)
    launchcore_sha256_update(&ctx, chunks[i], lengths[i]);
  launchcore_sha256_final(&ctx, out);
}

static void hmac_sha256(const uint8_t key[32],
                        const uint8_t *const *chunks, const size_t *lengths,
                        size_t count, uint8_t out[32])
{
  uint8_t ipad[64];
  uint8_t opad[64];
  uint8_t inner[32];
  memset(ipad, 0x36, sizeof(ipad));
  memset(opad, 0x5c, sizeof(opad));
  for (size_t i = 0; i < 32U; ++i) {
    ipad[i] ^= key[i];
    opad[i] ^= key[i];
  }

  launchcore_sha256_ctx_t ctx;
  launchcore_sha256_init(&ctx);
  launchcore_sha256_update(&ctx, ipad, sizeof(ipad));
  for (size_t i = 0; i < count; ++i)
    launchcore_sha256_update(&ctx, chunks[i], lengths[i]);
  launchcore_sha256_final(&ctx, inner);

  const uint8_t *outer_chunks[] = {opad, inner};
  const size_t outer_lengths[] = {sizeof(opad), sizeof(inner)};
  sha_chunks(outer_chunks, outer_lengths, 2U, out);
}

static SedsResult crypt_stream(uint32_t key_id, const uint8_t *nonce,
                               size_t nonce_len, const uint8_t *aad,
                               size_t aad_len, const uint8_t *input,
                               size_t input_len, uint8_t *output,
                               size_t output_cap)
{
  const uint8_t *key = find_key(key_id);
  if (key == NULL || nonce == NULL || (aad == NULL && aad_len != 0U) ||
      (input == NULL && input_len != 0U) || output == NULL ||
      output_cap < input_len)
    return SEDS_BAD_ARG;

  static const uint8_t label[] = "SEDS-HMAC-STREAM";
  uint8_t key_id_le[4] = {(uint8_t)key_id, (uint8_t)(key_id >> 8),
                          (uint8_t)(key_id >> 16), (uint8_t)(key_id >> 24)};
  uint64_t counter = 0U;
  for (size_t offset = 0; offset < input_len; offset += CRYPTO_BLOCK_SIZE) {
    uint8_t counter_le[8];
    for (size_t i = 0; i < sizeof(counter_le); ++i)
      counter_le[i] = (uint8_t)(counter >> (8U * i));
    const uint8_t *chunks[] = {label, key_id_le, nonce, aad, counter_le};
    const size_t lengths[] = {sizeof(label) - 1U, sizeof(key_id_le), nonce_len,
                              aad_len, sizeof(counter_le)};
    uint8_t block[32];
    hmac_sha256(key, chunks, lengths, 5U, block);
    size_t take = input_len - offset;
    if (take > sizeof(block)) take = sizeof(block);
    for (size_t i = 0; i < take; ++i)
      output[offset + i] = input[offset + i] ^ block[i];
    ++counter;
  }
  return SEDS_OK;
}

static void make_tag(const uint8_t key[32], uint32_t key_id,
                     const uint8_t *nonce, size_t nonce_len,
                     const uint8_t *aad, size_t aad_len,
                     const uint8_t *ciphertext, size_t ciphertext_len,
                     uint8_t tag[32])
{
  static const uint8_t label[] = "SEDS-HMAC-TAG";
  uint8_t key_id_le[4] = {(uint8_t)key_id, (uint8_t)(key_id >> 8),
                          (uint8_t)(key_id >> 16), (uint8_t)(key_id >> 24)};
  const uint8_t *chunks[] = {label, key_id_le, nonce, aad, ciphertext};
  const size_t lengths[] = {sizeof(label) - 1U, sizeof(key_id_le), nonce_len,
                            aad_len, ciphertext_len};
  hmac_sha256(key, chunks, lengths, 5U, tag);
}

static SedsResult seal(uint32_t key_id, const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *plaintext, size_t plaintext_len,
                       uint8_t *ciphertext, size_t ciphertext_cap,
                       size_t *ciphertext_len, uint8_t *tag, size_t tag_cap,
                       size_t *tag_len, void *user)
{
  (void)user;
  const uint8_t *key = find_key(key_id);
  if (key == NULL || ciphertext_len == NULL || tag == NULL ||
      tag_len == NULL || tag_cap < CRYPTO_TAG_SIZE)
    return SEDS_BAD_ARG;
  SedsResult result = crypt_stream(key_id, nonce, nonce_len, aad, aad_len,
                                   plaintext, plaintext_len, ciphertext,
                                   ciphertext_cap);
  if (result != SEDS_OK) return result;
  uint8_t full_tag[32];
  make_tag(key, key_id, nonce, nonce_len, aad, aad_len, ciphertext,
           plaintext_len, full_tag);
  memcpy(tag, full_tag, CRYPTO_TAG_SIZE);
  *ciphertext_len = plaintext_len;
  *tag_len = CRYPTO_TAG_SIZE;
  return SEDS_OK;
}

static SedsResult open(uint32_t key_id, const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ciphertext, size_t ciphertext_len,
                       const uint8_t *tag, size_t tag_len,
                       uint8_t *plaintext, size_t plaintext_cap,
                       size_t *plaintext_len, void *user)
{
  (void)user;
  const uint8_t *key = find_key(key_id);
  if (key == NULL || tag == NULL || tag_len != CRYPTO_TAG_SIZE ||
      plaintext_len == NULL)
    return SEDS_BAD_ARG;
  uint8_t expected[32];
  make_tag(key, key_id, nonce, nonce_len, aad, aad_len, ciphertext,
           ciphertext_len, expected);
  uint8_t diff = 0U;
  for (size_t i = 0; i < CRYPTO_TAG_SIZE; ++i) diff |= tag[i] ^ expected[i];
  if (diff != 0U) return SEDS_ERR;
  SedsResult result = crypt_stream(key_id, nonce, nonce_len, aad, aad_len,
                                   ciphertext, ciphertext_len, plaintext,
                                   plaintext_cap);
  if (result == SEDS_OK) *plaintext_len = ciphertext_len;
  return result;
}

SedsResult sedsnet_hardware_crypto_init(void)
{
  return seds_crypto_register_provider(seal, open, NULL);
}

SedsResult sedsnet_hardware_crypto_register_key(uint32_t id,
                                                const uint8_t key[32])
{
  if (key == NULL) return SEDS_BAD_ARG;
  crypto_key_t *free_slot = NULL;
  for (size_t i = 0; i < CRYPTO_KEY_SLOTS; ++i) {
    if (keys[i].used && keys[i].id == id) {
      memcpy(keys[i].key, key, 32U);
      return SEDS_OK;
    }
    if (!keys[i].used && free_slot == NULL) free_slot = &keys[i];
  }
  if (free_slot == NULL) return SEDS_ERR;
  free_slot->id = id;
  memcpy(free_slot->key, key, 32U);
  free_slot->used = true;
  return SEDS_OK;
}

void sedsnet_hardware_crypto_clear_keys(void)
{
  memset(keys, 0, sizeof(keys));
}

