/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "sedsnet_config.h"
#include "telemetry.h"
#include "DAQ-Threads.h"
#if (DISABLE_SD_CARD == 0U)
#include "sd_card.h"
#endif
#include "tx_api.h"
/* Provide telemetry_set_byte_pool so rust hooks use the app memory pool */
extern void telemetry_set_byte_pool(TX_BYTE_POOL *pool);
extern void telemetry_init_lock(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  telemetry_set_byte_pool(byte_pool);
  /* Initialize telemetry lock used by Rust (telemetry_lock/telemetry_unlock). */
  telemetry_init_lock();
#if (DISABLE_SD_CARD == 0U)
  ret = sd_card_init(byte_pool);
  if (ret == TX_SUCCESS)
  {
    ret = create_sd_writer_thread();
    if (ret != TX_SUCCESS)
    {
      Error_Handler();
    }
  }
#endif
  ret = create_telemetry_thread(byte_pool);
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }
  ret = create_daq_thread();
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
