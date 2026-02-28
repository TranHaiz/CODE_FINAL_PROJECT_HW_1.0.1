/**
 * @file       main.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-28
 * @author     Hai Tran
 *
 * @brief     Main entry point for the project
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"
#include "common_type.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
void my_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len);

/* Function definitions ----------------------------------------------- */
void setup()
{
  Serial.begin(115200);
  bsp_uart_config_t uart2_cfg = { .port     = UART_NUM_2,
                                  .tx_pin   = 17,
                                  .rx_pin   = 16,
                                  .baudrate = 115200,
                                  .callback = my_uart_callback };
  bsp_uart_init(&uart2_cfg);

  bsp_uart_config_t uart1_cfg = { .port     = UART_NUM_1,
                                  .tx_pin   = 8,
                                  .rx_pin   = 18,
                                  .baudrate = 115200,
                                  .callback = my_uart_callback };
  bsp_uart_init(&uart1_cfg);

  Serial.println("BSP UART System Started!");
}

void loop()
{
  // put your main code here, to run repeatedly:
}

/* Private definitions ----------------------------------------------- */
void my_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len)
{
  Serial.printf("Received on UART%d: ", uart_num);
  Serial.write(data, len);
}

/* End of file -------------------------------------------------------- */
