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
#include "log_service.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_sim, LOG_LEVEL_INFO)
#define SIM_SEND_CMD_RETRY (3)
#define DEBUG_SIM

#if (CONFIG_MQTT_SERVER == true)
#define MQTT_MAX_TOPIC_LEN   (128u)
#define MQTT_MAX_PAYLOAD_LEN (1024u)
#define MQTT_CLIENT_ID_LEN   (32u)

#define MQTT_BROKER_HOST     "broker.hivemq.com"
#define MQTT_BROKER_PORT     (1883)

#endif  // CONFIG_MQTT_SERVER

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
static bool    is_sim_ready = false;

#if (CONFIG_MQTT_SERVER == true)
static bsp_sim_mqtt_callback_t sim_mqtt_cb = NULL;
static uint8_t                 sim_mqtt_buf[512];
static uint16_t                sim_mqtt_buf_len              = 0;
static char                    client_id[MQTT_CLIENT_ID_LEN] = "001";

#endif  // CONFIG_MQTT_SERVER

/* Private function prototypes ---------------------------------------- */
/**
 * @brief  Send command and wait for specific response or timeout
 */
static bool bsp_sim_send_and_wait_response(const char *cmd, const char *resp, size_t timeout);

#if (CONFIG_FIREBASE_SERVER == true)
/**
 * @brief Find raw data response from DMA RX buffer
 */
static status_function_t
bsp_sim_parse_raw_data(uint8_t *source, uint16_t source_len, uint8_t *dest, uint16_t *dest_size);
#endif

/**
 * @brief  SIM response callback
 */
static void bsp_sim_rsp_callback(uart_port_t uart_num, uint8_t *data, size_t len);

#if (CONFIG_MQTT_SERVER == true)
static void build_client_id(char *buf, size_t buf_size);

#endif  // CONFIG_MQTT_SERVER

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

  if (!bsp_sim_send_and_wait_response("AT+CPIN?\r\n", "+CPIN: READY", 2000))
  {
    LOG_ERR("SIM card not ready: %s", sim_rx_buffer);
    is_sim_ready = false;
    return STATUS_ERROR;
  }

  bsp_sim_send_and_wait_response("AT+CREG?\r\n", "+CREG: 0,1", 3000);
  bsp_sim_send_and_wait_response("AT+CGATT?\r\n", "+CGATT: 1", 3000);

  if (!bsp_sim_send_and_wait_response("AT+CGDCONT=1,\"IP\",\"v-internet\"\r\n", "OK", 2000))
  {
    is_sim_ready = false;
    return STATUS_ERROR;
  }

  bsp_sim_send_and_wait_response("AT+CGACT=1,1\r\n", "OK", 3000);

#if (CONFIG_FIREBASE_SERVER == true)
  bool res = true;
  res      = bsp_sim_send_and_wait_response("AT+HTTPINIT\r\n", "OK", 5000);
  if (res == false)
  {
    return STATUS_ERROR;
  }
#endif

  is_sim_ready = true;
  return STATUS_OK;
}

bool bsp_sim_is_ready(void)
{
  if (!bsp_sim_send_and_wait_response("AT+CPIN?\r\n", "+CPIN: READY", 2000))
  {
    LOG_ERR("SIM card not ready: %s", sim_rx_buffer);
    return false;
  }
  else if (!bsp_sim_send_and_wait_response("AT+CREG?\r\n", "+CREG: 0,1", 3000))
  {
    LOG_ERR("No network register: %s", sim_rx_buffer);
    is_sim_ready = false;
  }
  else if (!bsp_sim_send_and_wait_response("AT+CGATT?\r\n", "+CGATT: 1", 3000))
  {
    LOG_ERR("Not attached to network: %s", sim_rx_buffer);
    is_sim_ready = false;
  }
  else
  {
    is_sim_ready = true;
  }

  return is_sim_ready;
}

#if (CONFIG_FIREBASE_SERVER == true)
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
#endif

#if (CONFIG_MQTT_SERVER == true)
status_function_t bsp_sim_mqtt_init(void)
{
  char cmd[128];

  /* 1. Start MQTT Service */
  if (bsp_sim_send_and_wait_response("AT+CMQTTSTART\r\n", "+CMQTTSTART: 0", 5000) == false)
  {
    LOG_ERR("Failed to start MQTT service: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  /* 2. Acquire MQTT Client — client ID động theo thiết bị */
  build_client_id(client_id, sizeof(client_id));
  snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"\r\n", client_id);
  if (bsp_sim_send_and_wait_response(cmd, "OK", 3000) == false)
  {
    LOG_ERR("Failed to acquire MQTT client: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  /* 3. Connect to Broker */
  snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1\r\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  if (bsp_sim_send_and_wait_response(cmd, "+CMQTTCONNECT: 0,0", 15000) == false)
  {
    LOG_ERR("Failed to connect MQTT broker: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_pub(mqtt_message_t *msg)
{
  if (msg == NULL || msg->topic == NULL || msg->payload == NULL)
  {
    return STATUS_ERROR;
  }

  char   cmd[256];
  size_t topic_len   = strlen(msg->topic);
  size_t payload_len = strlen(msg->payload);

  if (topic_len == 0 || topic_len > MQTT_MAX_TOPIC_LEN)
  {
    LOG_ERR("MQTT topic length out of range: %u", (unsigned) topic_len);
    return STATUS_ERROR;
  }
  if (payload_len == 0 || payload_len > MQTT_MAX_PAYLOAD_LEN)
  {
    LOG_ERR("MQTT payload length out of range: %u", (unsigned) payload_len);
    return STATUS_ERROR;
  }

  // Step 1: Set Topic
  snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d\r\n", (int) topic_len);
  if (bsp_sim_send_and_wait_response(cmd, ">", 2000) == false)
  {
    LOG_ERR("Failed to set MQTT topic: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }
  if (bsp_sim_send_and_wait_response(msg->topic, "OK", 2000) == false)
  {
    LOG_ERR("Failed to send MQTT topic: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // Step 2: Set Payload
  snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d\r\n", (int) payload_len);
  if (bsp_sim_send_and_wait_response(cmd, ">", 2000) == false)
  {
    LOG_ERR("Failed to set MQTT payload: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }
  if (bsp_sim_send_and_wait_response(msg->payload, "OK", 2000) == false)
  {
    LOG_ERR("Failed to send MQTT payload: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // Step 3: Publish
  snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,1,60\r\n");
  if (bsp_sim_send_and_wait_response(cmd, "+CMQTTPUB: 0,0", 10000) == false)
  {
    LOG_ERR("Failed to publish MQTT message: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_sub(const char *topic, bsp_sim_mqtt_callback_t cb)
{
  if (topic == NULL)
  {
    return STATUS_ERROR;
  }

  char   cmd[192];
  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > MQTT_MAX_TOPIC_LEN)
  {
    LOG_ERR("MQTT topic length out of range: %u", (unsigned) topic_len);
    return STATUS_ERROR;
  }

  // Step 1: Set topic length
  snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d\r\n", (int) topic_len);
  if (bsp_sim_send_and_wait_response(cmd, ">", 2000) == false)
  {
    LOG_ERR("Failed to set MQTT topic for subscription: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // Step 2: Send topic string
  if (bsp_sim_send_and_wait_response(topic, "OK", 2000) == false)
  {
    LOG_ERR("Failed to send MQTT topic for subscription: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // Step 3: Subscribe — wait for "+CMQTTSUB: 0," regardless of error code,
  //         then parse the code manually
  snprintf(cmd, sizeof(cmd), "AT+CMQTTSUB=0,1\r\n");
  if (bsp_sim_send_and_wait_response(cmd, "+CMQTTSUB: 0,", 5000) == false)
  {
    LOG_ERR("No response to CMQTTSUB: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // Parse error code from "+CMQTTSUB: 0,<code>"
  int err_code = -1;
  if (sscanf((char *) sim_rx_buffer, "+CMQTTSUB: 0,%d", &err_code) != 1)
  {
    LOG_ERR("Failed to parse CMQTTSUB response: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // 0  = subscribed successfully
  // 18 = already subscribed from a previous session — acceptable
  if (err_code == 0)
  {
    LOG_DBG("MQTT subscribed to topic: %s", topic);
  }
  else if (err_code == 18)
  {
    LOG_WRN("Topic already subscribed (code 18) — reusing existing subscription: %s", topic);
  }
  else
  {
    LOG_ERR("MQTT subscribe failed with code %d: %s", err_code, sim_rx_buffer);
    return STATUS_ERROR;
  }

  sim_mqtt_cb = cb;

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_get(uint8_t *out_buf, uint16_t *out_size)
{
  if (out_buf == NULL || out_size == NULL)
    return STATUS_ERROR;

  if (sim_mqtt_buf_len == 0)
  {
    *out_size = 0;
    return STATUS_OK;
  }

  if (sim_mqtt_buf_len > *out_size)
  {
    LOG_ERR("Output buffer too small: need %u, got %u", (unsigned) sim_mqtt_buf_len, (unsigned) *out_size);
    return STATUS_ERROR;
  }

  memcpy(out_buf, sim_mqtt_buf, sim_mqtt_buf_len);
  *out_size        = (uint16_t) sim_mqtt_buf_len;
  sim_mqtt_buf_len = 0;

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_deinit(void)
{
  bool ret = true;

  // Disconnect from broker
  if (bsp_sim_send_and_wait_response("AT+CMQTTDISC=0,60\r\n", "OK", 10000) == false)
  {
    LOG_ERR("Failed to disconnect MQTT client: %s", sim_rx_buffer);
    ret = false;
    /* Tiếp tục để release client và stop service */
  }

  // Release client
  if (bsp_sim_send_and_wait_response("AT+CMQTTREL=0\r\n", "OK", 3000) == false)
  {
    LOG_ERR("Failed to release MQTT client: %s", sim_rx_buffer);
    ret = false;
  }

  if (bsp_sim_send_and_wait_response("AT+CMQTTSTOP\r\n", "OK", 5000) == false)
  {
    LOG_ERR("Failed to stop MQTT service: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  return ret ? STATUS_OK : STATUS_ERROR;
}

#endif
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

#if (CONFIG_MQTT_SERVER == true)
  // Detect MQTT incoming message markers
  if (strstr((const char *) sim_rx_buffer, "+CMQTTRXSTART"))
  {
    // Best-effort parser for CMQTT URCs
    const char *topic_marker   = strstr((const char *) sim_rx_buffer, "+CMQTTRXTOPIC:");
    const char *payload_marker = strstr((const char *) sim_rx_buffer, "+CMQTTRXPAYLOAD:");
    const char *end_marker     = strstr((const char *) sim_rx_buffer, "+CMQTTRXEND:");

    if (topic_marker && payload_marker && end_marker)
    {
      // Extract topic
      char        topic[128]  = { 0 };
      const char *topic_start = strchr(topic_marker, '\n');
      if (topic_start)
      {
        topic_start++;  // Move past '\n'
        const char *topic_end = strstr(topic_start, "\r\n+CMQTTRXPAYLOAD");
        if (topic_end)
        {
          size_t topic_len = topic_end - topic_start;
          if (topic_len < sizeof(topic))
          {
            memcpy(topic, topic_start, topic_len);
          }
        }
      }

      // Extract payload
      const char *payload_start = strchr(payload_marker, '\n');
      if (payload_start)
      {
        payload_start++;  // Move past '\n'
        const char *payload_end = strstr(payload_start, "\r\n+CMQTTRXEND");
        if (payload_end)
        {
          size_t payload_len = payload_end - payload_start;
          if (payload_len < sizeof(sim_mqtt_buf))
          {
            memcpy(sim_mqtt_buf, payload_start, payload_len);
            sim_mqtt_buf[payload_len] = '\0';
            sim_mqtt_buf_len          = (uint16_t) payload_len;

            // call user callback if set
            if (sim_mqtt_cb != NULL)
            {
              sim_mqtt_cb(topic, sim_mqtt_buf, sim_mqtt_buf_len);
            }
          }
        }
      }
    }
  }
#endif  // CONFIG_MQTT_SERVER
}

#if (CONFIG_FIREBASE_SERVER == true)
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
#endif

#if (CONFIG_MQTT_SERVER == true)
static void build_client_id(char *buf, size_t buf_size)
{
  const char *dev_id = "1";
  if (dev_id == NULL || dev_id[0] == '\0')
  {
    snprintf(buf, buf_size, "haq-trk-unknown");
  }
  else
  {
    snprintf(buf, buf_size, "haq-trk-%.16s", dev_id);
  }
}

#endif

/* End of file -------------------------------------------------------- */
