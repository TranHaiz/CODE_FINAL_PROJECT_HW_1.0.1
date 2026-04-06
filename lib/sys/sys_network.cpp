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
#include "sys_cmd.h"
#include "sys_input.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_network, LOG_LEVEL_DBG)
#define MQTT_CLIENT_ID          "haq-trk-001"
#define MQTT_PUB_TOPIC          "haq-trk-001/data"
#define MQTT_TOPIC_COMMAND      "haq-trk-001/cmd"
#define MQTT_KEEPALIVE_S        (60)
#define MQTT_QOS                (1)

#define SIM_READY_TIMEOUT_MS    (10000)
#define SIM_READY_POLL_MS       (500)
#define SIM_HARD_RESET_DELAY_MS (2000)

#if (DEVICE_FUSION_DEBUG_MODE == 1)
#define MQTT_MESSAGE_MAX_LEN (1024)
#else
#define MQTT_MESSAGE_MAX_LEN (512)
#endif
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
OS_SEM_DEFINE_STATIC(sys_network_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
static void     sys_network_change_state(net_state_t new_state);
static uint32_t sys_network_backoff(uint8_t retry);

static void sys_network_run_sim_init(void);
static void sys_network_run_mqtt_init(void);
static void sys_network_run_sim_wait_ready(void);
static void sys_network_run_online(void);
static void sys_network_run_error_backoff(void);
static void sys_network_run_sim_hard_reset(void);

static bool sys_network_build_payload(sys_input_data_t *data);
static void sys_network_mqtt_message_cb(const char *topic, const uint8_t *data, size_t len);

static void sys_network_process_idle(void);
static void sys_network_process_locked(void);
static void sys_network_process_active(void);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  memset(&network_ctx, 0, sizeof(network_ctx));
  network_ctx.state      = NETWORK_STATE_SIM_INIT;
  network_ctx.prev_state = NETWORK_STATE_SIM_INIT;
  OS_SEM_CREATE(sys_network_wakeup_sem);

  LOG_DBG("Network layer initialized");
}

void sys_network_process(void)
{
  switch (g_device_info.state)
  {
  case DEVICE_STATE_LOCKED:
  case DEVICE_STATE_ACTIVE:
  {
    sys_network_process_active();
    break;
  }
  case DEVICE_STATE_IDLE:
  {
    sys_network_process_idle();
    break;
  }
  default: break;
  }
}

void sys_network_wakeup(void)
{
  OS_SEM_GIVE(sys_network_wakeup_sem);
}

/* Private definitions ----------------------------------------------- */
static void sys_network_change_state(net_state_t new_state)
{
  LOG_DBG("net: %d → %d  (retry=%d)", network_ctx.state, new_state, network_ctx.retry_count);

  network_ctx.prev_state     = network_ctx.state;
  network_ctx.state          = new_state;
  network_ctx.state_enter_ms = OS_GET_TICK();

  network_ctx.last_poll_ms = 0;
  network_ctx.last_log_ms  = 0;
}

static uint32_t sys_network_backoff(uint8_t retry)
{
  uint8_t  shift   = (retry < 4) ? retry : 4;
  uint32_t backoff = (uint32_t) BACKOFF_BASE_MS << shift;
  return (backoff > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : backoff;
}

static void sys_network_run_sim_init(void)
{
  LOG_DBG("Triggering SIM init");
  network_ctx.sim_ready  = false;
  network_ctx.mqtt_ready = false;

  if (bsp_sim_init() != STATUS_OK)
  {
    LOG_ERR("Failed to initialize SIM");
    sys_network_change_state(NETWORK_STATE_SIM_WAIT_READY);
    return;
  }

  sys_network_change_state(NETWORK_STATE_MQTT_INIT);
}

static void sys_network_run_sim_wait_ready(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) >= SIM_READY_TIMEOUT_MS)
  {
    LOG_WRN("SIM not ready after %d ms", SIM_READY_TIMEOUT_MS);
    sys_network_change_state(NETWORK_STATE_ERROR);
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
    sys_network_change_state(NETWORK_STATE_MQTT_INIT);
  }
  else
  {
    LOG_WRN("SIM, network not ready yet (%d / %d ms)", COUNT_MS(network_ctx.state_enter_ms), SIM_READY_TIMEOUT_MS);
  }
}

static void sys_network_run_mqtt_init(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) >= MQTT_INIT_TIMEOUT_MS)
  {
    LOG_WRN("MQTT init timeout (%d ms)", MQTT_INIT_TIMEOUT_MS);
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_deinit() != STATUS_OK)
  {
    LOG_DBG("MQTT deinit non-OK (stale session or first boot — continuing)");
  }

  if (bsp_sim_mqtt_init() != STATUS_OK)
  {
    LOG_WRN("MQTT init failed");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_sub(MQTT_TOPIC_COMMAND, sys_network_mqtt_message_cb) != STATUS_OK)
  {
    LOG_WRN("MQTT subscribe failed");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  LOG_DBG("MQTT initialized");
  network_ctx.mqtt_ready        = true;
  network_ctx.retry_count       = 0;  // successful init resets retry counter
  network_ctx.last_publish_ms   = millis();
  network_ctx.last_keepalive_ms = millis();

  sys_network_change_state(NETWORK_STATE_ONLINE);
}

static void sys_network_run_online(void)
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
      sys_network_change_state(NETWORK_STATE_ERROR);
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
      sys_network_change_state(NETWORK_STATE_ERROR);
      return;
    }
    LOG_DBG("MQTT keepalive OK");
    network_ctx.last_keepalive_ms = millis();
  }
}

static void sys_network_run_error_backoff(void)
{
  uint32_t backoff = sys_network_backoff(network_ctx.retry_count);
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
    sys_network_change_state(NETWORK_STATE_SIM_RESET);
  }
  else
  {
    LOG_DBG("Backoff complete — soft retry %d", network_ctx.retry_count);
    sys_network_change_state(NETWORK_STATE_SIM_WAIT_READY);
  }
}

static void sys_network_run_sim_hard_reset(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) < 10)
  {
    // TODO: Hardware reset by mosfet power control
  }

  if (COUNT_MS(network_ctx.state_enter_ms) < SIM_HARD_RESET_DELAY_MS)
  {
    return;
  }

  sys_network_change_state(NETWORK_STATE_SIM_INIT);
}

static void sys_network_mqtt_message_cb(const char *topic, const uint8_t *data, size_t len)
{
  LOG_DBG("MQTT rx [%s]: %d bytes, %s", topic, (int) len, (const char *) data);
  memset(g_cmd_input_buffer, 0, CMD_INPUT_MAX_LEN);
  if ((data == NULL) || (len >= CMD_INPUT_MAX_LEN))
  {
    LOG_WRN("Invalid command payload: %d bytes", (int) len);
    return;
  }

  strncpy(g_cmd_input_buffer, (const char *) data, len);
  OS_SEM_GIVE(sys_cmd_req_sem);
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

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  int dbg_written =
    snprintf(mqtt_payload_buffer + written, MQTT_MESSAGE_MAX_LEN - written,
             ","
             "\"acc_rx\":%.3f,\"acc_ry\":%.3f,\"acc_rz\":%.3f,"
             "\"acc_fx\":%.3f,\"acc_fy\":%.3f,\"acc_fz\":%.3f,"
             "\"gyr_rx\":%.3f,\"gyr_ry\":%.3f,\"gyr_rz\":%.3f,"
             "\"gyr_fx\":%.3f,\"gyr_fy\":%.3f,\"gyr_fz\":%.3f,"
             "\"cmp_rx\":%.1f,\"cmp_ry\":%.1f,\"cmp_rz\":%.1f,"
             "\"cmp_fx\":%.3f,\"cmp_fy\":%.3f,\"cmp_fz\":%.3f,"
             "\"v_ins\":%.3f,\"v_gps\":%.3f,"
             "\"d_ins\":%.2f,\"d_gps\":%.2f"
             "}",
             data->debug.acc_raw_x, data->debug.acc_raw_y, data->debug.acc_raw_z, data->debug.acc_filter_x,
             data->debug.acc_filter_y, data->debug.acc_filter_z, data->debug.gyro_raw_x, data->debug.gyro_raw_y,
             data->debug.gyro_raw_z, data->debug.gyro_filter_x, data->debug.gyro_filter_y, data->debug.gyro_filter_z,
             data->debug.compass_raw_x, data->debug.compass_raw_y, data->debug.compass_raw_z,
             data->debug.compass_filter_x, data->debug.compass_filter_y, data->debug.compass_filter_z,
             data->debug.v_ins, data->debug.v_gps, data->debug.distance_ins, data->debug.distance_gps);

  if (dbg_written < 0 || (written + dbg_written) >= (int) MQTT_MESSAGE_MAX_LEN)
  {
    LOG_WRN("Debug payload truncated");
    mqtt_payload_buffer[0] = '\0';
    return false;
  }
#else
  int close_written = snprintf(mqtt_payload_buffer + written, MQTT_MESSAGE_MAX_LEN - written, "}");
  if (close_written < 0 || (written + close_written) >= (int) MQTT_MESSAGE_MAX_LEN)
  {
    mqtt_payload_buffer[0] = '\0';
    return false;
  }
#endif

  return true;
}

static void sys_network_process_idle(void)
{
  // TODO: Enter low-power mode, deinit SIM, etc.
  bsp_sim_mqtt_deinit();
  network_ctx.sim_ready  = false;
  network_ctx.mqtt_ready = false;
  sys_network_change_state(NETWORK_STATE_SIM_INIT);
  OS_SEM_TAKE(sys_network_wakeup_sem, OS_MAX_DELAY);
}

static void sys_network_process_locked(void)
{
  // Do nothing for now, just receive commands
}

static void sys_network_process_active(void)
{
  switch (network_ctx.state)
  {
  case NETWORK_STATE_SIM_INIT:
  {
    sys_network_run_sim_init();
    break;
  }
  case NETWORK_STATE_SIM_WAIT_READY:
  {
    sys_network_run_sim_wait_ready();
    break;
  }
  case NETWORK_STATE_MQTT_INIT:
  {
    sys_network_run_mqtt_init();
    break;
  }
  case NETWORK_STATE_ONLINE:
  {
    if (g_device_info.state != DEVICE_STATE_ACTIVE)
      return;
    sys_network_run_online();
    break;
  }
  case NETWORK_STATE_ERROR:
  {
    sys_network_run_error_backoff();
    break;
  }
  case NETWORK_STATE_SIM_RESET:
  {
    sys_network_run_sim_hard_reset();
    break;
  }
  default:
  {
    LOG_ERR("Unknown state %d — resetting", network_ctx.state);
    sys_network_change_state(NETWORK_STATE_SIM_INIT);
    break;
  }
  }
}

/* End of file -------------------------------------------------------- */
