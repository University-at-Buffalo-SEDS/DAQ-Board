#pragma once

#include "sedsprintf.h"
#include "stm32u5xx_hal.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*board_link_uart_rx_cb_t)(const uint8_t *data, size_t len, void *user);

void board_link_uart_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef board_link_uart_start_rx(void);
void board_link_uart_process(void);

SedsResult board_link_uart_tx_send(const uint8_t *bytes, size_t len, void *user);
HAL_StatusTypeDef board_link_uart_subscribe_rx(board_link_uart_rx_cb_t cb, void *user);
HAL_StatusTypeDef board_link_uart_unsubscribe_rx(board_link_uart_rx_cb_t cb, void *user);

void board_link_uart_handle_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void board_link_uart_handle_error(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
