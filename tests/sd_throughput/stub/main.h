/* Host model of the STM32 HAL surface used by the SD path (virtual time). */
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { HAL_SD_CARD_READY = 1, HAL_SD_CARD_TRANSFER = 4, HAL_SD_CARD_RECEIVING = 6,
  HAL_SD_CARD_PROGRAMMING = 7, HAL_SD_CARD_ERROR = 0xFF } HAL_SD_CardStateTypeDef;
typedef enum { HAL_SD_STATE_RESET = 0, HAL_SD_STATE_READY = 1, HAL_SD_STATE_BUSY = 3 } HAL_SD_StateTypeDef;
#define HAL_SD_ERROR_NONE 0U
#define HAL_SD_ERROR_TIMEOUT 0x80000000U
#define SD_CONTEXT_NONE 0U
#define SDMMC_DISABLE_IDMA 0U
#define SDMMC_STATIC_FLAGS 0U
#define SDMMC_BUS_WIDE_4B 1U
typedef struct { volatile uint32_t MASK, IDMACTRL, DCTRL, ICR; } SDMMC_TypeDef;
typedef struct { uint32_t LogBlockNbr, LogBlockSize; } HAL_SD_CardInfoTypeDef;
typedef struct {
  SDMMC_TypeDef *Instance; volatile uint32_t ErrorCode; volatile HAL_SD_StateTypeDef State;
  volatile uint32_t Context; uint32_t CID[4]; HAL_SD_CardInfoTypeDef SdCard;
} SD_HandleTypeDef;
typedef enum { SDMMC1_IRQn = 78 } IRQn_Type;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *h, uint32_t wide);
HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *h, HAL_SD_CardInfoTypeDef *info);
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *h, uint8_t *data, uint32_t sector, uint32_t count);
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *h, uint8_t *data, uint32_t sector, uint32_t count);
void HAL_SD_IRQHandler(SD_HandleTypeDef *h);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq);
#define __HAL_SD_CLEAR_FLAG(h, f) ((void)(h), (void)(f))
#define __HAL_RCC_SDMMC1_FORCE_RESET() ((void)0)
#define __HAL_RCC_SDMMC1_RELEASE_RESET() ((void)0)
#define __DSB() ((void)0)
#define __DMB() ((void)0)
extern uint32_t g_model_primask;
static inline uint32_t __get_PRIMASK(void) { return g_model_primask; }
static inline void __disable_irq(void) { g_model_primask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) { g_model_primask = mask; }
#endif
