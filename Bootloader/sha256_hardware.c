#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_hash.h"
#include "stm32u5xx_hal_hash_ex.h"
#include <stdbool.h>
#include "launchcore/image.h"
#include <string.h>

#define HASH_TIMEOUT_MS 1000U

static HASH_HandleTypeDef hhash;
static bool hash_failed;

static void feed_blocks(launchcore_sha256_ctx_t *ctx, const uint8_t *data, size_t len)
{
    while (len != 0U) {
        size_t take = sizeof(ctx->block) - ctx->used;
        if (take > len) take = len;
        memcpy(&ctx->block[ctx->used], data, take);
        ctx->used += take;
        data += take;
        len -= take;
        if (ctx->used == sizeof(ctx->block)) {
            if (HAL_HASHEx_SHA256_Accmlt(&hhash, ctx->block,
                                               sizeof(ctx->block)) != HAL_OK)
                hash_failed = true;
            ctx->used = 0U;
        }
    }
}

void HAL_HASH_MspInit(HASH_HandleTypeDef *handle)
{
    (void)handle;
    __HAL_RCC_HASH_CLK_ENABLE();
}

void launchcore_sha256_init(launchcore_sha256_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    memset(&hhash, 0, sizeof(hhash));
    hhash.Init.DataType = HASH_DATATYPE_8B;
    hash_failed = HAL_HASH_Init(&hhash) != HAL_OK;
}

void launchcore_sha256_update(launchcore_sha256_ctx_t *ctx,
                              const void *data, size_t len)
{
    if (ctx == NULL || (data == NULL && len != 0U) || hash_failed) {
        hash_failed = true;
        return;
    }
    ctx->len += len;
    feed_blocks(ctx, (const uint8_t *)data, len);
}

void launchcore_sha256_final(launchcore_sha256_ctx_t *ctx, uint8_t out[32])
{
    if (ctx == NULL || out == NULL || hash_failed ||
        HAL_HASHEx_SHA256_Accmlt_End(&hhash, ctx->block, ctx->used, out,
                                     HASH_TIMEOUT_MS) != HAL_OK)
        memset(out, 0, 32U);
    (void)HAL_HASH_DeInit(&hhash);
}

void launchcore_sha256(const void *data, size_t len, uint8_t out[32])
{
    launchcore_sha256_ctx_t ctx;
    launchcore_sha256_init(&ctx);
    launchcore_sha256_update(&ctx, data, len);
    launchcore_sha256_final(&ctx, out);
}

