/**
 * @file       bsp_uart.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-08
 * @author     Hai Tran
 *
 * @brief      Generic UART BSP driver (DMA IDLE interrupt)
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"

/* Private defines ---------------------------------------------------- */
#define RX_BUF_SIZE 2048

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
// Uart buffer queue handles
static QueueHandle_t bsp_uart_queues[UART_NUM_MAX];

/* Private function prototypes ---------------------------------------- */
static void uart_event_task(void *pvParameters);

/* Function definitions ----------------------------------------------- */
void bsp_uart_init(bsp_uart_config_t *config)
{
  uart_config_t uart_config = {
    .baud_rate           = config->baudrate,
    .data_bits           = UART_DATA_8_BITS,
    .parity              = UART_PARITY_DISABLE,
    .stop_bits           = UART_STOP_BITS_1,
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk          = UART_SCLK_APB,  // S3 sử dụng DEFAULT thay vì APB cũ
  };

  // 1. Cài đặt Driver
  uart_driver_install(config->port, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 20, &bsp_uart_queues[config->port], 0);
  uart_param_config(config->port, &uart_config);
  uart_set_pin(config->port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // 2. Kích hoạt Idle Detection (Timeout)
  // Sau khi nhận byte cuối, nếu im lặng 10 symbol sẽ bắn sự kiện UART_DATA
  uart_set_rx_timeout(config->port, 10);

  // 3. Tạo Task xử lý (Sử dụng heap để truyền config vào task an toàn)
  bsp_uart_config_t *task_cfg = new bsp_uart_config_t(*config);

  char task_name[20];
  snprintf(task_name, sizeof(task_name), "uart_task_%d", config->port);
  xTaskCreate(uart_event_task, task_name, 4096, (void *) task_cfg, 12, NULL);
}

void bsp_uart_write(uart_port_t uart_num, const char *data, size_t len)
{
  uart_write_bytes(uart_num, (const char *) data, len);
}

/* Private definitions ----------------------------------------------- */
static void uart_event_task(void *pvParameters)
{
  // Copy config vào local để đảm bảo an toàn vùng nhớ
  bsp_uart_config_t cfg = *(bsp_uart_config_t *) pvParameters;
  delete (bsp_uart_config_t *) pvParameters;  // Giải phóng vùng nhớ tạm đã malloc ở init

  uart_event_t event;
  uint8_t     *dtmp = (uint8_t *) malloc(RX_BUF_SIZE);

  for (;;)
  {
    // Đợi sự kiện IDLE/Timeout (Tương đương ngắt DMA Idle của STM32)
    if (xQueueReceive(bsp_uart_queues[cfg.port], (void *) &event, portMAX_DELAY))
    {
      switch (event.type)
      {
      case UART_DATA:
        if (event.size > 0)
        {
          // Đọc dữ liệu từ Hardware Buffer
          int len = uart_read_bytes(cfg.port, dtmp, event.size, 0);
          if (len > 0 && cfg.callback != NULL)
          {
            cfg.callback(cfg.port, dtmp, (size_t) len);
          }
        }
        break;

      case UART_FIFO_OVF:
        uart_flush_input(cfg.port);
        xQueueReset(bsp_uart_queues[cfg.port]);
        break;

      case UART_BUFFER_FULL: uart_flush_input(cfg.port); break;

      default: break;
      }
    }
  }
  free(dtmp);
  vTaskDelete(NULL);
}

/* End of file -------------------------------------------------------- */
