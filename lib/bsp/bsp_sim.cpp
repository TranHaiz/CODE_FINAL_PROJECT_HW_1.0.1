/**
 * @file       bsp_sim.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      BSP driver for SIM module (UART DMA Idle)
 *              - Init SIM module
 *              - API contact SMS
 *              - API call via phone number
 *              - API contact with firebase
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_sim.h"

#include "a7680c.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
#define SIM_SEND_CMD_RETRY (3)
#define DEBUG_SIM

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define SIM_SEND(data) (bsp_uart_write(SIM_UART_HANDLER, (data), strlen((data))))
#ifdef DEBUG_SIM
#define LOG(char)      Serial.println(char)
#define LOG_DATA(data) Serial.println(data)
#else
#define LOG(char)
#define LOG_DATA(data)
#endif

/* Public variables --------------------------------------------------- */
uint8_t sim_raw_data[SIM_RAW_RSP_SIZE];

/* Private variables -------------------------------------------------- */
static bool    is_sim_rsp = false;
static uint8_t sim_rx_buffer[SIM_RX_BUFFER_SIZE];
static char    json_data_buffer[128];
static char    request_buffer[64];

/* Private function prototypes ---------------------------------------- */
/**
 * @brief  Send command and wait for specific response or timeout
 */
static bool bsp_sim_send_and_wait_response(const char *cmd, const char *resp, size_t timeout);

/**
 * @brief Find raw data response from DMA RX buffer
 */
static status_function_t
bsp_sim_parse_raw_data(uint8_t *source, uint16_t source_len, uint8_t *dest, uint16_t *dest_size);

/**
 * @brief  SIM response callback
 */
static void bsp_sim_rsp_callback(uart_port_t uart_num, uint8_t *data, size_t len);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_sim_init(void)
{
  bsp_uart_config_t uart2_cfg = { .port     = SIM_UART_HANDLER,
                                  .tx_pin   = SIM_UART_TX,
                                  .rx_pin   = SIM_UART_RX,
                                  .baudrate = SIM_UART_BAUDRATE,
                                  .callback = bsp_sim_rsp_callback };
  bsp_uart_init(&uart2_cfg);

  SIM_SEND("ATE0\r\n");
  OS_DELAY_MS(50);

  // Tắt các URC không cần thiết
  bsp_sim_send_and_wait_response("AT*URCMODE=0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CFUN=1\r\n", "OK", 2000);
  bsp_sim_send_and_wait_response("AT+CIURC=0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CNMI=0,0,0,0,0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CGEREP=0,0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CREG=0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CGREG=0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+STSF=0\r\n", "OK", 1000);

  bsp_sim_send_and_wait_response("AT\r\n", "OK", 2000);
  bsp_sim_send_and_wait_response("AT+CPIN?\r\n", "+CPIN: READY", 2000);
  bsp_sim_send_and_wait_response("AT+CREG?\r\n", "+CREG: 0,1", 3000);
  bsp_sim_send_and_wait_response("AT+CGATT?\r\n", "+CGATT: 1", 3000);
  bsp_sim_send_and_wait_response("AT+CGDCONT=1,\"IP\",\"v-internet\"\r\n", "OK", 2000);
  bsp_sim_send_and_wait_response("AT+CGACT=1,1\r\n", "OK", 3000);
  bool res = bsp_sim_send_and_wait_response("AT+HTTPINIT\r\n", "OK", 5000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  return STATUS_OK;
}

status_function_t bsp_sim_reset_http(void)
{
  for (uint8_t i = 0; i < SIM_SEND_CMD_RETRY; i++)
  {
    if (bsp_sim_send_and_wait_response("AT+HTTPTERM\r\n", "OK", 1000) != STATUS_OK)
    {
      OS_YIELD();
    }
    else
    {
      break;
    }
  }
  if (bsp_sim_send_and_wait_response("AT+HTTPINIT\r\n", "OK", 1000) == false)
  {
    return STATUS_ERROR;
  }
  return STATUS_OK;
}

status_function_t bsp_sim_send_data_firebase(firebase_data_t *data)
{
  assert_param(data != NULL);

  char json_data[128];
  // clang-format off
  sprintf(json_data,
          "{\"Battery level\":\"%d\",\"Position\":\"%.6f,%.6f\",\"Speed\":\"%.2f\"}",
          data->batt_level,
          data->position.latitude,
          data->position.longitude,
          data->speed);
  // clang-format on
  uint8_t data_len = strlen(json_data);

  char request[64];
  sprintf(request, "AT+HTTPDATA=%d,10000\r\n", data_len);

  bool res = false;

  res = bsp_sim_send_and_wait_response("AT+HTTPPARA=\"SSLCFG\",0\r\n", "OK", 100);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  res = bsp_sim_send_and_wait_response(
    "AT+HTTPPARA=\"URL\",\"https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json\"\r\n", "OK",
    2000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  res = bsp_sim_send_and_wait_response("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", "OK", 1000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  res = bsp_sim_send_and_wait_response(request, "DOWNLOAD", 1000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  res = bsp_sim_send_and_wait_response(json_data, "OK", 1000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
  res = bsp_sim_send_and_wait_response("AT+HTTPACTION=4\r\n", "+HTTPACTION: ", 15000);
  res = bsp_sim_send_and_wait_response("AT+HTTPREAD=0,100\r\n", "}", 1000);

  return STATUS_OK;
}

status_function_t bsp_sim_get_raw_data_firebase(uint8_t *raw_data_buffer, uint16_t *size)
{
  assert_param(raw_data_buffer != NULL);
  bool res = false;

  res = bsp_sim_send_and_wait_response("AT+HTTPPARA=\"SSLCFG\",0\r\n", "OK", 100);
  res = bsp_sim_send_and_wait_response(
    "AT+HTTPPARA=\"URL\",\"https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json\"\r\n", "OK",
    2000);

  res = bsp_sim_send_and_wait_response("AT+HTTPACTION=0\r\n", "+HTTPACTION: ", 15000);
  if (res == false)
  {
    return STATUS_ERROR;
  }

  res = bsp_sim_send_and_wait_response("AT+HTTPREAD=0,500\r\n", "{", 15000);
  if (res == false)
  {
    return STATUS_ERROR;
  }

  // Copy response data to output buffer
  bsp_sim_parse_raw_data(sim_rx_buffer, SIM_RX_BUFFER_SIZE, raw_data_buffer, size);

  return STATUS_OK;
}

/* Private definitions ----------------------------------------------- */
static bool bsp_sim_send_and_wait_response(const char *cmd, const char *resp, size_t timeout)
{
  SIM_SEND(cmd);
  size_t start_tick = OS_GET_TICK();
  while ((OS_GET_TICK() - start_tick) < timeout)
  {
    if (is_sim_rsp)
    {
      is_sim_rsp = false;
      if (strstr((const char *) sim_rx_buffer, resp))
      {
        return true;
      }
    }
    OS_YIELD();
  }
  return false;
}

static status_function_t
bsp_sim_parse_raw_data(uint8_t *source, uint16_t source_len, uint8_t *dest, uint16_t *dest_size)
{
  bool    is_found = false;
  uint8_t start    = 0;
  for (uint8_t i = 0; i < source_len; i++)
  {
    if (source[i] == '{')
    {
      is_found = true;
      start    = i;
      break;
    }
  }

  if (!is_found)
  {
    return STATUS_ERROR;
  }

  // Find ending '}'
  uint8_t end           = 0;
  is_found              = false;
  uint8_t remaining_len = source_len - start + 1;
  for (uint8_t i = 0; i < remaining_len; i++)
  {
    if (source[i] == '}')
    {
      is_found = true;
      end      = i;
      break;
    }
  }

  // Not found '}'
  if (!is_found)
  {
    return STATUS_ERROR;
  }

  // Calculate length both '{' and '}'
  uint8_t length = (end - start) + 1;

  memcpy(dest, &source[start], length);
  dest[length] = '\0';
  *dest_size   = length;

  return STATUS_OK;
}

static void bsp_sim_rsp_callback(uart_port_t uart_num, uint8_t *data, size_t len)
{
  // Copy data to SIM RX buffer
  if (len < SIM_RX_BUFFER_SIZE)
  {
    memcpy(sim_rx_buffer, data, len);
    sim_rx_buffer[len] = '\0';  // Null-terminate
  }
  else
  {
    memcpy(sim_rx_buffer, data, SIM_RX_BUFFER_SIZE - 1);
    sim_rx_buffer[SIM_RX_BUFFER_SIZE - 1] = '\0';
  }
  is_sim_rsp = true;
}

/* End of file -------------------------------------------------------- */
