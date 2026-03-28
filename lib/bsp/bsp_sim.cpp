/**
 * @file       bsp_sim.c
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-27
 * @author     Hai Tran
 *
 * @brief      BSP for SIM module — ported to Quectel EG800K
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_sim.h"

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
#define MQTT_PUBLISH_QOS     (1)
#define MQTT_SUB_QOS         (2)

#define MQTT_BROKER_HOST     "test.mosquitto.org"
#define MQTT_BROKER_PORT     (1883)

// EG800K: MQTT context index (0–5)
#define MQTT_CTX             (0)
#endif

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
static volatile bool is_sim_rsp = false;
static uint8_t       sim_rx_buffer[SIM_RX_BUFFER_SIZE];
static char          json_data_buffer[128];
static char          request_buffer[64];
static bool          is_sim_ready = false;

#if (CONFIG_MQTT_SERVER == true)
static bsp_sim_mqtt_callback_t sim_mqtt_cb = NULL;
static uint8_t                 sim_mqtt_buf[512];
static uint16_t                sim_mqtt_buf_len              = 0;
static char                    client_id[MQTT_CLIENT_ID_LEN] = "001";
#endif

/* Private function prototypes ---------------------------------------- */
static bool bsp_sim_send_and_wait_response(const char *cmd, const char *resp, size_t timeout);

#if (CONFIG_FIREBASE_SERVER == true)
static status_function_t
bsp_sim_parse_raw_data(uint8_t *source, uint16_t source_len, uint8_t *dest, uint16_t *dest_size);
#endif

static void bsp_sim_rsp_callback(uart_port_t uart_num, uint8_t *data, size_t len);

#if (CONFIG_MQTT_SERVER == true)
static void bsp_sim_build_client_id(char *buf, size_t buf_size);
#endif

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

  bsp_sim_send_and_wait_response("AT+CFUN=1\r\n", "OK", 2000);
  bsp_sim_send_and_wait_response("AT+CNMI=0,0,0,0,0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CGEREP=0,0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CREG=0\r\n", "OK", 1000);
  bsp_sim_send_and_wait_response("AT+CGREG=0\r\n", "OK", 1000);

  bsp_sim_send_and_wait_response("AT\r\n", "OK", 2000);

  if (!bsp_sim_send_and_wait_response("AT+CPIN?\r\n", "+CPIN: READY", 2000))
  {
    LOG_ERR("SIM card not ready: %s", sim_rx_buffer);
    is_sim_ready = false;
    return STATUS_ERROR;
  }

  bsp_sim_send_and_wait_response("AT+CREG?\r\n", "+CREG: 0,1", 3000);
  bsp_sim_send_and_wait_response("AT+CGATT?\r\n", "+CGATT: 1", 3000);

  if (!bsp_sim_send_and_wait_response("AT+QICSGP=1,1,\"v-internet\",\"\",\"\",1\r\n", "OK", 2000))
  {
    is_sim_ready = false;
    return STATUS_ERROR;
  }

  bsp_sim_send_and_wait_response("AT+QIACT=1\r\n", "OK", 5000);

#if (CONFIG_FIREBASE_SERVER == true)
  if (!bsp_sim_send_and_wait_response("AT+HTTPINIT\r\n", "OK", 5000))
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
    if (bsp_sim_send_and_wait_response("AT+HTTPTERM\r\n", "OK", 1000))
      break;
    OS_YIELD();
  }
  if (!bsp_sim_send_and_wait_response("AT+HTTPINIT\r\n", "OK", 1000))
    return STATUS_ERROR;
  return STATUS_OK;
}

status_function_t bsp_sim_send_data_firebase(firebase_data_t *data)
{
  assert_param(data != NULL);

  char json_data[128];
  sprintf(json_data, "{\"Battery level\":\"%d\",\"Position\":\"%.6f,%.6f\",\"Speed\":\"%.2f\"}", data->batt_level,
          data->position.latitude, data->position.longitude, data->speed);
  uint8_t data_len = strlen(json_data);

  char request[64];
  sprintf(request, "AT+HTTPDATA=%d,10000\r\n", data_len);

  if (!bsp_sim_send_and_wait_response("AT+HTTPPARA=\"SSLCFG\",0\r\n", "OK", 100))
    return STATUS_ERROR;
  if (!bsp_sim_send_and_wait_response(
        "AT+HTTPPARA=\"URL\",\"https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json\"\r\n", "OK",
        2000))
    return STATUS_ERROR;
  if (!bsp_sim_send_and_wait_response("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", "OK", 1000))
    return STATUS_ERROR;
  if (!bsp_sim_send_and_wait_response(request, "DOWNLOAD", 1000))
    return STATUS_ERROR;
  if (!bsp_sim_send_and_wait_response(json_data, "OK", 1000))
    return STATUS_ERROR;

  bsp_sim_send_and_wait_response("AT+HTTPACTION=4\r\n", "+HTTPACTION: ", 15000);
  bsp_sim_send_and_wait_response("AT+HTTPREAD=0,100\r\n", "}", 1000);

  return STATUS_OK;
}

status_function_t bsp_sim_get_raw_data_firebase(uint8_t *raw_data_buffer, uint16_t *size)
{
  assert_param(raw_data_buffer != NULL);

  bsp_sim_send_and_wait_response("AT+HTTPPARA=\"SSLCFG\",0\r\n", "OK", 100);
  bsp_sim_send_and_wait_response(
    "AT+HTTPPARA=\"URL\",\"https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json\"\r\n", "OK",
    2000);

  if (!bsp_sim_send_and_wait_response("AT+HTTPACTION=0\r\n", "+HTTPACTION: ", 15000))
    return STATUS_ERROR;
  if (!bsp_sim_send_and_wait_response("AT+HTTPREAD=0,500\r\n", "{", 15000))
    return STATUS_ERROR;

  bsp_sim_parse_raw_data(sim_rx_buffer, SIM_RX_BUFFER_SIZE, raw_data_buffer, size);
  return STATUS_OK;
}
#endif  // CONFIG_FIREBASE_SERVER

#if (CONFIG_MQTT_SERVER == true)

status_function_t bsp_sim_mqtt_init(void)
{
  char cmd[128];

  // 1. Config MQTT receive mode: URC with topic + payload (mode 1,0,0,1)
  if (!bsp_sim_send_and_wait_response("AT+QMTCFG=\"recv/mode\",0,0,1\r\n", "OK", 2000))
  {
    LOG_ERR("Failed to set MQTT recv mode: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // 2. MQTT version 3.1.1
  if (!bsp_sim_send_and_wait_response("AT+QMTCFG=\"version\",0,4\r\n", "OK", 2000))
  {
    LOG_ERR("Failed to set MQTT version: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // 3. Keepalive 60s
  bsp_sim_send_and_wait_response("AT+QMTCFG=\"keepalive\",0,60\r\n", "OK", 2000);

  // 4. Open MQTT
  snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=%d,\"%s\",%d\r\n", MQTT_CTX, MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  if (!bsp_sim_send_and_wait_response(cmd, "+QMTOPEN: 0,0", 15000))
  {
    LOG_ERR("Failed to open MQTT connection: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  // 5. CONNECT broker
  bsp_sim_build_client_id(client_id, sizeof(client_id));
  snprintf(cmd, sizeof(cmd), "AT+QMTCONN=%d,\"%s\",\"\",\"\"\r\n", MQTT_CTX, client_id);
  if (!bsp_sim_send_and_wait_response(cmd, "+QMTCONN: 0,0,0", 15000))
  {
    LOG_ERR("Failed to connect MQTT broker: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_pub(mqtt_message_t *msg)
{
  if (msg == NULL || msg->topic == NULL || msg->payload == NULL)
    return STATUS_ERROR;

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

#if (MQTT_PUBLISH_QOS == 0)
  snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=%d,0,%d,0,\"%s\",%d\r\n", MQTT_CTX, MQTT_PUBLISH_QOS, msg->topic,
           (int) payload_len);
#elif (MQTT_PUBLISH_QOS == 1)
  snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=%d,1,%d,0,\"%s\",%d\r\n", MQTT_CTX, MQTT_PUBLISH_QOS, msg->topic,
           (int) payload_len);
#endif

  if (!bsp_sim_send_and_wait_response(cmd, ">", 5000))
  {
    LOG_ERR("Failed to get > prompt for MQTT publish: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }

#if (MQTT_PUBLISH_QOS == 0)
  if (!bsp_sim_send_and_wait_response(msg->payload, "+QMTPUBEX: 0,0,0", 10000))
  {
    LOG_ERR("Failed to publish MQTT message: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }
#elif (MQTT_PUBLISH_QOS == 1)
  if (!bsp_sim_send_and_wait_response(msg->payload, "+QMTPUBEX: 0,1,0", 10000))
  {
    LOG_ERR("Failed to publish MQTT message: %s", sim_rx_buffer);
    return STATUS_ERROR;
  }
#endif

  return STATUS_OK;
}

status_function_t bsp_sim_mqtt_sub(const char *topic, bsp_sim_mqtt_callback_t cb)
{
  if (topic == NULL)
    return STATUS_ERROR;

  char   cmd[192];
  char   resp[128];
  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > MQTT_MAX_TOPIC_LEN)
  {
    LOG_ERR("MQTT topic length out of range: %u", (unsigned) topic_len);
    return STATUS_ERROR;
  }

  snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%d,1,\"%s\",%d\r\n", MQTT_CTX, topic, MQTT_SUB_QOS);
  snprintf(resp, sizeof(resp), "+QMTSUB: 0,1,0,%d", MQTT_SUB_QOS);
  if (!bsp_sim_send_and_wait_response(cmd, resp, 5000))
  {
    int err_code = -1;
    sscanf((char *) sim_rx_buffer, "+QMTSUB: 0,1,%d", &err_code);

    if (err_code == 0)
    {
      LOG_DBG("MQTT subscribed to topic: %s", topic);
    }
    else
    {
      LOG_ERR("MQTT subscribe failed with code %d: %s", err_code, sim_rx_buffer);
      return STATUS_ERROR;
    }
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
  char cmd[64];

  // Disconnect
  snprintf(cmd, sizeof(cmd), "AT+QMTDISC=%d\r\n", MQTT_CTX);
  if (!bsp_sim_send_and_wait_response(cmd, "+QMTDISC: 0,0", 10000))
  {
    LOG_ERR("Failed to disconnect MQTT: %s", sim_rx_buffer);
    ret = false;
  }

  // Close TCP
  snprintf(cmd, sizeof(cmd), "AT+QMTCLOSE=%d\r\n", MQTT_CTX);
  if (!bsp_sim_send_and_wait_response(cmd, "+QMTCLOSE: 0,0", 5000))
  {
    LOG_ERR("Failed to close MQTT connection: %s", sim_rx_buffer);
    ret = false;
  }

  return ret ? STATUS_OK : STATUS_ERROR;
}

#endif  // CONFIG_MQTT_SERVER

static bool bsp_sim_send_and_wait_response(const char *cmd, const char *resp, size_t timeout)
{
  if (cmd && cmd[0] != '\0')
    SIM_SEND(cmd);

  if (resp == NULL || resp[0] == '\0')
    return true;

  size_t start_tick = OS_GET_TICK();
  while ((OS_GET_TICK() - start_tick) < timeout)
  {
    if (is_sim_rsp)
    {
      is_sim_rsp = false;
      if (strstr((const char *) sim_rx_buffer, resp))
        return true;
    }
    OS_YIELD();
  }
  return false;
}

static void bsp_sim_rsp_callback(uart_port_t uart_num, uint8_t *data, size_t len)
{
  if (len < SIM_RX_BUFFER_SIZE)
  {
    memcpy(sim_rx_buffer, data, len);
    sim_rx_buffer[len] = '\0';
  }
  else
  {
    memcpy(sim_rx_buffer, data, SIM_RX_BUFFER_SIZE - 1);
    sim_rx_buffer[SIM_RX_BUFFER_SIZE - 1] = '\0';
  }
  is_sim_rsp = true;

#if (CONFIG_MQTT_SERVER == true)
  const char *recv_marker = strstr((const char *) sim_rx_buffer, "+QMTRECV:");
  if (recv_marker)
  {
    char topic[128] = { 0 };

    const char *p = strchr(recv_marker, '"');
    if (p)
    {
      p++;
      const char *topic_end = strchr(p, '"');
      if (topic_end)
      {
        size_t tlen = topic_end - p;
        if (tlen < sizeof(topic))
          memcpy(topic, p, tlen);

        const char *q = topic_end + 1;
        if (*q == ',')
          q++;
        while (*q && *q != ',') q++;
        if (*q == ',')
          q++;

        if (*q == '"')
          q++;
        const char *payload_end = q;
        while (*payload_end && *payload_end != '"' && *payload_end != '\r' && *payload_end != '\n') payload_end++;

        size_t plen = payload_end - q;
        if (plen > 0 && plen < sizeof(sim_mqtt_buf))
        {
          memcpy(sim_mqtt_buf, q, plen);
          sim_mqtt_buf[plen] = '\0';
          sim_mqtt_buf_len   = (uint16_t) plen;

          if (sim_mqtt_cb != NULL)
            sim_mqtt_cb(topic, sim_mqtt_buf, sim_mqtt_buf_len);
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
    return STATUS_ERROR;

  uint8_t end           = 0;
  is_found              = false;
  uint8_t remaining_len = source_len - start;
  for (uint8_t i = 0; i < remaining_len; i++)
  {
    if (source[start + i] == '}')
    {
      is_found = true;
      end      = start + i;
      break;
    }
  }
  if (!is_found)
    return STATUS_ERROR;

  uint8_t length = (end - start) + 1;
  memcpy(dest, &source[start], length);
  dest[length] = '\0';
  *dest_size   = length;

  return STATUS_OK;
}
#endif

#if (CONFIG_MQTT_SERVER == true)
static void bsp_sim_build_client_id(char *buf, size_t buf_size)
{
  const char *dev_id = "1";
  if (dev_id == NULL || dev_id[0] == '\0')
    snprintf(buf, buf_size, "haq-trk-unknown");
  else
    snprintf(buf, buf_size, "haq-trk-%.16s", dev_id);
}
#endif

/* End of file -------------------------------------------------------- */