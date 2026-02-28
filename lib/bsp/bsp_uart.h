/**
 * @file       bsp_uart.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-08
 * @author     Hai Tran
 *
 * @brief      Generic UART BSP driver (DMA IDLE interrupt)
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef BSP_UART_H
#define BSP_UART_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "driver/uart.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef void (*uart_callback_t)(uart_port_t uart_num, uint8_t *data, size_t len);
typedef struct
{
  uart_port_t     port;
  int             tx_pin;
  int             rx_pin;
  int             baudrate;  // Chuyển thành int để khớp với uart_config_t
  uart_callback_t callback;
} bsp_uart_config_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_uart_init(bsp_uart_config_t *config);
void bsp_uart_send(uart_port_t uart_num, const uint8_t *data, size_t len);

#endif /*End file BSP_UART_H*/

/* End of file -------------------------------------------------------- */
