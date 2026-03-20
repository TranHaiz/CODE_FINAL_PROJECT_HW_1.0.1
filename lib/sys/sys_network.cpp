/**
 * @file       sys_network.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      System Network Layer - Implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network.h"

#include "bsp_rtc.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_network, LOG_LEVEL_NONE)
#define MQTT_CLIENT_ID           "haq-trk-001"
#define MQTT_PUB_TOPIC           "haq-trk-001/data"
#define MQTT_TOPIC_COMMAND       "haq-trk-001/cmd"
#define MQTT_KEEPALIVE_S         (60)
#define MQTT_QOS                 (1)

#define SIM_READY_TIMEOUT_MS     (10000)
#define SIM_READY_POLL_MS        (500)
#define SIM_HARD_RESET_DELAY_MS  (2000)

#define MQTT_MESSAGE_MAX_LEN     (512)
#define MQTT_INIT_TIMEOUT_MS     (15000)
#define MQTT_PUBLISH_INTERVAL_MS (1000)
#define MQTT_KEEPALIVE_MS        (MQTT_KEEPALIVE_S * 1000UL)

#define BACKOFF_BASE_MS          (2000)
#define BACKOFF_MAX_MS           (32000)
#define RETRY_MAX_BEFORE_RESET   (3)

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  NETWORK_STATE_SIM_INIT = 0,
  NETWORK_STATE_SIM_WAIT_READY,
  NETWORK_STATE_MQTT_INIT,
  NETWORK_STATE_ONLINE,
  NETWORK_STATE_ERROR,
  NETWORK_STATE_SIM_RESET,
} net_state_t;

typedef struct
{
  net_state_t state;
  net_state_t prev_state;

  uint8_t  retry_count;
  uint32_t state_enter_ms;
  uint32_t last_publish_ms;
  uint32_t last_keepalive_ms;
  uint32_t last_poll_ms;
  uint32_t last_log_ms;

  bool sim_ready;
  bool mqtt_ready;
} net_ctx_t;

/* Private macros ----------------------------------------------------- */
#define COUNT_MS(since_ms) ((uint32_t) (millis() - (since_ms)))

/* Public variables --------------------------------------------------- */
bool is_data_network_ready = false;

/* Private variables -------------------------------------------------- */
static net_ctx_t        network_ctx;
static char             mqtt_payload_buffer[MQTT_MESSAGE_MAX_LEN];
static sys_input_data_t last_published_data;

/* Private function prototypes ---------------------------------------- */
static void        net_transition(net_state_t new_state);
static uint32_t    net_backoff_ms(uint8_t retry);
static const char *net_state_name(net_state_t state);

static void net_run_sim_init(void);
static void net_run_sim_wait_ready(void);
static void net_run_mqtt_init(void);
static void net_run_online(void);
static void net_run_error_backoff(void);
static void net_run_sim_hard_reset(void);

static bool sys_network_build_payload(sys_input_data_t *data);
static void net_on_mqtt_message(const char *topic, const uint8_t *data, size_t len);
static bool net_publish_telemetry(void);
static void net_build_payload(const sys_input_data_t *data, char *buf, size_t buf_len);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  memset(&network_ctx, 0, sizeof(network_ctx));
  network_ctx.state      = NETWORK_STATE_SIM_INIT;
  network_ctx.prev_state = NETWORK_STATE_SIM_INIT;

  LOG_DBG("Network layer initialized");
}

void sys_network_process(void)
{
  switch (network_ctx.state)
  {
  case NETWORK_STATE_SIM_INIT:
  {
    net_run_sim_init();
    break;
  }
  case NETWORK_STATE_SIM_WAIT_READY:
  {
    net_run_sim_wait_ready();
    break;
  }
  case NETWORK_STATE_MQTT_INIT:
  {
    net_run_mqtt_init();
    break;
  }
  case NETWORK_STATE_ONLINE:
  {
    net_run_online();
    break;
  }
  case NETWORK_STATE_ERROR:
  {
    net_run_error_backoff();
    break;
  }
  case NETWORK_STATE_SIM_RESET:
  {
    net_run_sim_hard_reset();
    break;
  }
  default:
  {
    LOG_ERR("Unknown state %d — resetting", network_ctx.state);
    net_transition(NETWORK_STATE_SIM_INIT);
    break;
  }
  }
}

/* Private definitions ----------------------------------------------- */
static void net_transition(net_state_t new_state)
{
  LOG_DBG("net: %d → %d  (retry=%d)", network_ctx.state, new_state, network_ctx.retry_count);

  network_ctx.prev_state     = network_ctx.state;
  network_ctx.state          = new_state;
  network_ctx.state_enter_ms = OS_GET_TICK();

  network_ctx.last_poll_ms = 0;
  network_ctx.last_log_ms  = 0;
}

static uint32_t net_backoff_ms(uint8_t retry)
{
  uint8_t  shift   = (retry < 4) ? retry : 4;
  uint32_t backoff = (uint32_t) BACKOFF_BASE_MS << shift;
  return (backoff > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : backoff;
}

static void net_run_sim_init(void)
{
  LOG_DBG("Triggering SIM init");
  network_ctx.sim_ready  = false;
  network_ctx.mqtt_ready = false;

  if (bsp_sim_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize SIM");
    net_transition(NETWORK_STATE_SIM_WAIT_READY);
    return;
  }

  net_transition(NETWORK_STATE_MQTT_INIT);
}

static void net_run_sim_wait_ready(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) >= SIM_READY_TIMEOUT_MS)
  {
    LOG_WRN("SIM not ready after %d ms", SIM_READY_TIMEOUT_MS);
    net_transition(NETWORK_STATE_ERROR);
    return;
  }

  if (COUNT_MS(network_ctx.last_poll_ms) < SIM_READY_POLL_MS)
  {
    return;
  }
  network_ctx.last_poll_ms = millis();

  if (bsp_sim_is_ready())
  {
    LOG_DBG("SIM ready");
    network_ctx.sim_ready = true;
    net_transition(NETWORK_STATE_MQTT_INIT);
  }
  else
  {
    LOG_WRN("SIM, network not ready yet (%d / %d ms)", COUNT_MS(network_ctx.state_enter_ms), SIM_READY_TIMEOUT_MS);
  }
}

static void net_run_mqtt_init(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) >= MQTT_INIT_TIMEOUT_MS)
  {
    LOG_WRN("MQTT init timeout (%d ms)", MQTT_INIT_TIMEOUT_MS);
    net_transition(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_deinit() != STATUS_OK)
  {
    LOG_DBG("MQTT deinit non-OK (stale session or first boot — continuing)");
  }

  if (bsp_sim_mqtt_init() != STATUS_OK)
  {
    LOG_WRN("MQTT init failed");
    net_transition(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_sub(MQTT_TOPIC_COMMAND, net_on_mqtt_message) != STATUS_OK)
  {
    LOG_WRN("MQTT subscribe failed");
    net_transition(NETWORK_STATE_ERROR);
    return;
  }

  LOG_DBG("MQTT initialized");
  network_ctx.mqtt_ready        = true;
  network_ctx.retry_count       = 0;  // successful init resets retry counter
  network_ctx.last_publish_ms   = millis();
  network_ctx.last_keepalive_ms = millis();

  net_transition(NETWORK_STATE_ONLINE);
}

static void net_run_online(void)
{
  // Public data if ready
  if (is_data_network_ready)
  {
    is_data_network_ready = false;
    sys_input_get_data(&last_published_data);
    if (!sys_network_build_payload(&last_published_data))
    {
      LOG_WRN("Failed to build telemetry payload");
      return;
    }
    mqtt_message_t msg = {
      .topic   = MQTT_PUB_TOPIC,
      .payload = mqtt_payload_buffer,
    };
    if (bsp_sim_mqtt_pub(&msg) != STATUS_OK)
    {
      LOG_WRN("Publish failed — assuming connection lost");
      net_transition(NETWORK_STATE_ERROR);
      return;
    }
    network_ctx.last_publish_ms = millis();
  }

  // Periodic keepalive / connection health check
  if (COUNT_MS(network_ctx.last_keepalive_ms) >= MQTT_KEEPALIVE_MS)
  {
    if (bsp_sim_is_ready() != STATUS_OK)
    {
      LOG_WRN("Sim or network not ready");
      net_transition(NETWORK_STATE_ERROR);
      return;
    }
    LOG_DBG("MQTT keepalive OK");
    network_ctx.last_keepalive_ms = millis();
  }
}

/* -------------------------------------------------------------------- */
/* State: ERROR_BACKOFF                                                   */
/* -------------------------------------------------------------------- */

static void net_run_error_backoff(void)
{
  uint32_t backoff = net_backoff_ms(network_ctx.retry_count);
  uint32_t elapsed = COUNT_MS(network_ctx.state_enter_ms);

  if (elapsed < backoff)
  {
    // Progress log every 5 s to avoid log spam
    if (COUNT_MS(network_ctx.last_log_ms) >= 5000)
    {
      LOG_DBG("Error SIM");
      network_ctx.last_log_ms = millis();
    }
    return;
  }

  network_ctx.retry_count++;

  if (network_ctx.retry_count >= RETRY_MAX_BEFORE_RESET)
  {
    LOG_WRN("Before %d retries — performing hard SIM reset", network_ctx.retry_count);
    network_ctx.retry_count = 0;
    net_transition(NETWORK_STATE_SIM_RESET);
  }
  else
  {
    LOG_DBG("Backoff complete — soft retry %d", network_ctx.retry_count);
    net_transition(NETWORK_STATE_SIM_WAIT_READY);
  }
}

static void net_run_sim_hard_reset(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) < 10)
  {
    // TODO: Hardware reset by mosfet power control
  }

  if (COUNT_MS(network_ctx.state_enter_ms) < SIM_HARD_RESET_DELAY_MS)
  {
    return;
  }

  net_transition(NETWORK_STATE_SIM_INIT);
}

/* -------------------------------------------------------------------- */
/* MQTT helpers                                                           */
/* -------------------------------------------------------------------- */

static void net_on_mqtt_message(const char *topic, const uint8_t *data, size_t len)
{
  LOG_INF("MQTT rx [%s]: %.*s", topic, (int) len, (const char *) data);

  if (strcmp((const char *) data, "LOCK") == 0)
  {
    sys_ui_simple_change_ui(SYS_UI_SIMPLE_VIEW_LOGIN);
  }
  else if (strcmp((const char *) data, "UNLOCK") == 0)
  {
    sys_ui_simple_change_ui(SYS_UI_SIMPLE_VIEW_MAIN);
  }
  else
  {
    // Do nothing
  }
}

static void net_build_payload(const sys_input_data_t *data, char *buf, size_t buf_len)
{
  snprintf(buf, buf_len,
           "{"
           "\"ts\":%lu,"
           "\"spd\":%.2f,"
           "\"dist\":%.1f,"
           "\"hdg\":%.1f,"
           "\"dir\":\"%s\","
           "\"lat\":%.6f,"
           "\"lon\":%.6f,"
           "\"dust\":%.1f,"
           "\"temp\":%.1f,"
           "\"hum\":%.1f"
           "}",
           data->timestamp_ms, data->velocity_kmh, data->distance_m, data->heading_deg,
           (data->direction_str != NULL) ? data->direction_str : "?", data->gps_position.latitude,
           data->gps_position.longitude, data->dust_value, data->temp_hum.temperature, data->temp_hum.humidity);
}

static bool sys_network_build_payload(sys_input_data_t *data)
{
  if (data == NULL)
  {
    mqtt_payload_buffer[0] = '\0';
    return false;
  }

  timeline_t now;
  memset(&now, 0, sizeof(now));
  bsp_rtc_get(&now);

  int written =
    snprintf(mqtt_payload_buffer, MQTT_MESSAGE_MAX_LEN,
             "{"
             "\"battery\":%.1f,"
             "\"time\":[%d/%d/%d-%d:%d:%d],"
             "\"velocity_ms\":%.2f,"
             "\"velocity_kmh\":%.2f,"
             "\"distance_m\":%.1f,"
             "\"direction\":%.1f %s,"
             "\"position\":(%.6f,%.6f),"
             "\"dust\":%.1f,"
             "\"temp\":%.1f,"
             "\"hum\":%.1f"
             "}",
             data->battery_level, now.year, now.month, now.date, now.hour, now.minute, now.second, data->velocity_ms,
             data->velocity_kmh, data->distance_m, data->heading_deg,
             (data->direction_str != NULL) ? data->direction_str : "?", data->gps_position.latitude,
             data->gps_position.longitude, data->dust_value, data->temp_hum.temperature, data->temp_hum.humidity);

  if (written < 0 || written >= (int) MQTT_MESSAGE_MAX_LEN)
  {
    LOG_WRN("Payload truncated: need %d bytes, buffer only %d", written, MQTT_MESSAGE_MAX_LEN);
    mqtt_payload_buffer[0] = '\0';
    return false;
  }

  return true;
}

/* End of file -------------------------------------------------------- */
