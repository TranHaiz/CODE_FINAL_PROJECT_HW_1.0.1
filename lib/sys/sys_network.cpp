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
#include "bsp_sdcard.h"
#include "cbuffer.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_cmd.h"
#include "sys_input.h"
#include "sys_ui_simple.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_network, LOG_LEVEL_DBG)

// MQTT
#define MQTT_KEEPALIVE_S            (30)
#define MQTT_QOS                    (1)
#define MQTT_KEEPALIVE_MS           (MQTT_KEEPALIVE_S * 1000UL)
#define MQTT_PUBLISH_RETRY          (3)
#define MQTT_PUBLISH_RETRY_DELAY_MS (100)
#define MQTT_REQUEST_PUBLISH_MAX    (10)
#if (DEVICE_FUSION_DEBUG_MODE == 1)
#define MQTT_MESSAGE_MAX_LEN (1024)
#else
#define MQTT_MESSAGE_MAX_LEN (512)
#endif

// Timming
#define OFFLINE_POLL_MS            (100)
#define ONLINE_FAST_POLL_MS        (50)
#define ONLINE_POLL_MS             (500)
#define ONLINE_LOCKED_POLL_MS      (2000)
#define ONLINE_IDLE_POLL_MS        (10000)
#define NETWORK_DATA_TASK_POLL_MS  (700)
#define SIM_READY_TIMEOUT_MS       (10000)
#define SIM_HARD_RESET_DELAY_MS    (2000)
#define MQTT_INIT_TIMEOUT_MS       (15000)

// Retry, backoff, and reset
#define BACKOFF_BASE_MS            (2000)
#define BACKOFF_MAX_MS             (32000)
#define RETRY_MAX_BEFORE_RESET     (3)

#define NETWORK_CBUFF_SLOT_SIZE    (MQTT_MESSAGE_MAX_LEN)
#define NETWORK_CBUFF_COUNT        (100)
#define NETWORK_BYTES              (NETWORK_CBUFF_COUNT * NETWORK_CBUFF_SLOT_SIZE)
#define NETWORK_CBUFF_FLUSH_THRESH (80)
#define CBUFFER_FAST_MSG_THRESHOLD (2)

#define SD_OFFLINE_DIR             "/buff"
#define SD_OFFLINE_LOG_PATH        "/buff/offline_log.json"
#define SD_JSON_LINE_MAX_LEN       (MQTT_MESSAGE_MAX_LEN + 2)  // 1 line JSON + <CRLF>
#define SD_CARD_RETRY_COUNT        (3)
#define SD_CARD_RETRY_DELAY_MS     (100)

#define NETWORK_KEEPALIVE_MES      "KEEPALIVE"

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

  uint8_t retry_count;
  size_t  state_enter_ms;
  size_t  last_publish_ms;
  size_t  last_keepalive_ms;
  size_t  last_poll_ms;
  size_t  last_log_ms;
  size_t  network_sd_offset;

  bool is_sys_network_init;
  bool sim_ready;
  bool mqtt_ready;
  bool is_data_sd_pending;

  cbuffer_t cbuffer;
} net_ctx_t;

/* Private macros ----------------------------------------------------- */
#define COUNT_MS(since_ms) ((size_t) (OS_GET_TICK() - (since_ms)))

/* Public variables --------------------------------------------------- */
volatile bool is_data_network_ready = false;

/* Private variables -------------------------------------------------- */
static net_ctx_t network_ctx;
static uint8_t   network_buffer[NETWORK_BYTES];
static char      s_pub_slot[NETWORK_CBUFF_SLOT_SIZE];
static bool      s_pub_slot_valid = false;
static cbuffer_t req_pub_cbuffer;
static char      req_pub_buffer[MQTT_REQUEST_PUBLISH_MAX * MQTT_REQUEST_PUBLISH_SIZE];

/* Shared scratch buffer for SD line read */
static char s_sd_line[SD_JSON_LINE_MAX_LEN];

OS_MUTEX_DEFINE_STATIC(network_data_mutex);
OS_MUTEX_DEFINE_STATIC(network_noti_mutex);
OS_SEM_DEFINE_STATIC(sys_network_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
static void   sys_network_change_state(net_state_t new_state);
static size_t sys_network_backoff(uint8_t retry);

static void sys_network_run_sim_init(void);
static void sys_network_run_sim_wait_ready(void);
static void sys_network_run_mqtt_init(void);
static void sys_network_run_online(void);
static void sys_network_run_error_backoff(void);
static void sys_network_run_sim_hard_reset(void);

static void sys_network_process_idle(void);
static void sys_network_process_active(void);

static bool sys_network_build_payload(sys_input_data_t *data, char *buf, size_t buf_len);

static void              sys_network_publish_online(void);
static void              sys_network_flush_cbuff_to_sd(void);
static status_function_t sys_network_push_sd_to_mqtt(void);
static bool              sys_network_check_pending(void);
static bool              sys_network_need_push_sd(void);
static bool              sys_network_need_fast_poll(void);
static status_function_t sys_network_prepare_sd_card(void);
static status_function_t sys_network_push_cbuffer(const char *payload);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  network_ctx.is_sys_network_init = false;

  memset(&network_ctx, 0, sizeof(network_ctx));
  network_ctx.state      = NETWORK_STATE_SIM_INIT;
  network_ctx.prev_state = NETWORK_STATE_SIM_INIT;

  OS_SEM_CREATE(sys_network_wakeup_sem);
  OS_MUTEX_CREATE(network_noti_mutex);
  OS_MUTEX_CREATE(network_data_mutex);

  cb_init(&network_ctx.cbuffer, network_buffer, NETWORK_BYTES);
  cb_init(&req_pub_cbuffer, req_pub_buffer, MQTT_REQUEST_PUBLISH_MAX * MQTT_REQUEST_PUBLISH_SIZE);

  (void) sys_network_prepare_sd_card();

  network_ctx.is_data_sd_pending = sys_network_check_pending();
  network_ctx.network_sd_offset  = 0;
  if (network_ctx.is_data_sd_pending)
  {
    LOG_INF("Offline log found on SD — will drain after reconnect");
  }

  network_ctx.is_sys_network_init = true;
}

void sys_network_process(void *param)
{
  while (1)
  {
    switch (g_device_info.nvs_info.curr_state)
    {
    case DEVICE_STATE_LOCKED:
    case DEVICE_STATE_ACTIVE:
    case DEVICE_STATE_IDLE:
    {
      sys_network_process_active();
      break;
    }
    default: break;
    }
  }
}

void sys_network_wakeup(void)
{
  OS_SEM_GIVE(sys_network_wakeup_sem);
}

void sys_network_mqtt_publish_noti(const char *payload, size_t payload_len)
{
  if (payload == NULL || payload_len == 0 || payload_len >= MQTT_REQUEST_PUBLISH_SIZE)
  {
    return;
  }

  char slot[MQTT_REQUEST_PUBLISH_SIZE];
  memset(slot, 0, sizeof(slot));
  memcpy(slot, payload, payload_len);

  OS_MUTEX_LOCK(network_noti_mutex);
  uint32_t ret = cb_write(&req_pub_cbuffer, slot, MQTT_REQUEST_PUBLISH_SIZE);
  OS_MUTEX_UNLOCK(network_noti_mutex);

  if (ret != MQTT_REQUEST_PUBLISH_SIZE)
  {
    LOG_WRN("req_pub cbuffer full — notification dropped");
  }
}

void sys_network_data_task(void *param)
{
  while (1)
  {
    if (!network_ctx.is_sys_network_init)
    {
      OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
      continue;
    }

    if ((g_device_info.nvs_info.curr_state == DEVICE_STATE_ACTIVE) && is_data_network_ready)
    {
      is_data_network_ready = false;

      sys_input_data_t input_data;
      if (sys_input_get_data(&input_data) == STATUS_OK)
      {
        char payload[NETWORK_CBUFF_SLOT_SIZE];
        if (sys_network_build_payload(&input_data, payload, sizeof(payload)))
        {
          sys_network_push_cbuffer(payload);
        }
      }
    }

    if (sys_network_need_push_sd())
    {
      sys_network_flush_cbuff_to_sd();
    }

    OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_network_change_state(net_state_t new_state)
{
  LOG_DBG("net: %d → %d  (retry=%d)", network_ctx.state, new_state, network_ctx.retry_count);

  /* Discard in-flight publish slot when leaving ONLINE */
  if (network_ctx.state == NETWORK_STATE_ONLINE && new_state != NETWORK_STATE_ONLINE)
  {
    s_pub_slot_valid = false;
  }

  network_ctx.prev_state     = network_ctx.state;
  network_ctx.state          = new_state;
  network_ctx.state_enter_ms = OS_GET_TICK();
  network_ctx.last_poll_ms   = 0;
  network_ctx.last_log_ms    = 0;
}

static size_t sys_network_backoff(uint8_t retry)
{
  uint8_t shift   = (retry < 4) ? retry : 4;
  size_t  backoff = (size_t) BACKOFF_BASE_MS << shift;
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
    sys_network_change_state(NETWORK_STATE_ERROR);
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

  if (COUNT_MS(network_ctx.last_poll_ms) < OFFLINE_POLL_MS)
  {
    return;
  }
  network_ctx.last_poll_ms = OS_GET_TICK();

  if (bsp_sim_is_ready())
  {
    LOG_DBG("SIM ready");
    network_ctx.sim_ready = true;
    sys_network_change_state(NETWORK_STATE_MQTT_INIT);
  }
  else
  {
    LOG_WRN("SIM not ready yet (%d / %d ms)", COUNT_MS(network_ctx.state_enter_ms), SIM_READY_TIMEOUT_MS);
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

  if (bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_mqtt_message_cb) != STATUS_OK)
  {
    LOG_WRN("MQTT subscribe failed");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  LOG_INF("MQTT connected");
  network_ctx.mqtt_ready        = true;
  network_ctx.retry_count       = 0;
  network_ctx.last_publish_ms   = OS_GET_TICK();
  network_ctx.last_keepalive_ms = OS_GET_TICK();

  sys_network_change_state(NETWORK_STATE_ONLINE);
}

static void sys_network_run_online(void)
{
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
    if (COUNT_MS(network_ctx.last_keepalive_ms) >= MQTT_KEEPALIVE_MS)
    {
      if (!bsp_sim_is_ready())
      {
        LOG_WRN("Keepalive: SIM or network lost");
        sys_network_change_state(NETWORK_STATE_ERROR);
        return;
      }
      mqtt_message_t mes = {
        .topic   = g_device_info.mqtt_noti_topic,
        .payload = NETWORK_KEEPALIVE_MES,
      };

      bool publish_ok = false;
      for (uint8_t i = 0; i < MQTT_PUBLISH_RETRY; i++)
      {
        if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
        {
          LOG_DBG("Keepalive OK");
          publish_ok = true;
          break;
        }
        OS_DELAY_MS(MQTT_PUBLISH_RETRY_DELAY_MS);
      }
      if (!publish_ok)
      {
        LOG_WRN("Keepalive failed after %d attempts", MQTT_PUBLISH_RETRY);
        sys_network_change_state(NETWORK_STATE_ERROR);
        return;
      }
      network_ctx.last_keepalive_ms = OS_GET_TICK();
    }
    return;
  }

  // 1. Publish notifications or commands if pending
  char     req_payload[MQTT_REQUEST_PUBLISH_SIZE] = { 0 };
  uint32_t req_count                              = 0;
  bool     is_pub_noti_ok                         = false;

  OS_MUTEX_LOCK(network_noti_mutex);
  if (cb_data_count(&req_pub_cbuffer) >= MQTT_REQUEST_PUBLISH_SIZE)
  {
    req_count = cb_read(&req_pub_cbuffer, req_payload, MQTT_REQUEST_PUBLISH_SIZE);
  }
  OS_MUTEX_UNLOCK(network_noti_mutex);

  if (req_count != 0)
  {
    mqtt_message_t mes = {
      .topic   = g_device_info.mqtt_noti_topic,
      .payload = req_payload,
    };
    for (uint8_t attempt = 0; attempt < MQTT_PUBLISH_RETRY; attempt++)
    {
      if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
      {
        is_pub_noti_ok = true;
        break;
      }
    }

    if (!is_pub_noti_ok)
    {
      LOG_WRN("Failed to publish notification after %d attempts", MQTT_PUBLISH_RETRY);
      sys_network_change_state(NETWORK_STATE_ERROR);
    }
    return;
  }
  else
  {
    // Do nothing
  }

  // 2. Publish mes in sd
  if (network_ctx.is_data_sd_pending)
  {
    sys_network_push_sd_to_mqtt();
    return;
  }

  // 3. Publish from cbuffer if available
  sys_network_publish_online();
}

static void sys_network_run_error_backoff(void)
{
  size_t backoff = sys_network_backoff(network_ctx.retry_count);
  size_t elapsed = COUNT_MS(network_ctx.state_enter_ms);

  if (elapsed < backoff)
  {
    if (COUNT_MS(network_ctx.last_log_ms) >= 5000)
    {
      LOG_DBG("Error backoff — waiting %u ms (elapsed %u ms)", (unsigned) backoff, (unsigned) elapsed);
      network_ctx.last_log_ms = OS_GET_TICK();
    }
    return;
  }

  network_ctx.retry_count++;

  if (network_ctx.retry_count >= RETRY_MAX_BEFORE_RESET)
  {
    LOG_WRN("Max retries (%d) — performing hard SIM reset", network_ctx.retry_count);
    network_ctx.retry_count = 0;
    sys_network_change_state(NETWORK_STATE_SIM_RESET);
  }
  else
  {
    LOG_DBG("Soft retry %d", network_ctx.retry_count);
    sys_network_change_state(NETWORK_STATE_SIM_WAIT_READY);
  }
}

static void sys_network_run_sim_hard_reset(void)
{
  if (COUNT_MS(network_ctx.state_enter_ms) < 10)
  {
    // TODO: Hardware reset via MOSFET power control
  }

  if (COUNT_MS(network_ctx.state_enter_ms) < SIM_HARD_RESET_DELAY_MS)
  {
    return;
  }

  sys_network_change_state(NETWORK_STATE_SIM_INIT);
}

void sys_network_mqtt_message_cb(const char *topic, const uint8_t *data, size_t len)
{
  LOG_DBG("MQTT rx [%s]: %d bytes", topic, (int) len);
  if ((data == NULL) || (len == 0) || (len >= CMD_INPUT_MAX_LEN))
  {
    LOG_WRN("Invalid command payload: %d bytes", (int) len);
    return;
  }

  memset(g_cmd_input_buffer, 0, CMD_INPUT_MAX_LEN);
  strncpy(g_cmd_input_buffer, (const char *) data, len);
  OS_SEM_GIVE(sys_cmd_req_sem);
}

static bool sys_network_build_payload(sys_input_data_t *data, char *buf, size_t buf_len)
{
  if (data == NULL || buf == NULL || buf_len == 0)
  {
    return false;
  }

  timeline_t now;
  memset(&now, 0, sizeof(now));
  bsp_rtc_get(&now);

  int written =
    snprintf(buf, buf_len,
             "{"
             "\"battery\":%.1f,"
             "\"time\":\"%d/%02d/%02d-%02d:%02d:%02d\","
             "\"velocity_ms\":%.2f,"
             "\"velocity_kmh\":%.2f,"
             "\"distance_m\":%.1f,"
             "\"direction\":\"%.1f %s\","
             "\"position\":[%.6f,%.6f],"
             "\"dust\":%.1f,"
             "\"temp\":%.1f,"
             "\"hum\":%.1f",
             data->battery_level, now.year, now.month, now.date, now.hour, now.minute, now.second, data->velocity_ms,
             data->velocity_kmh, data->distance_m, data->heading_deg,
             (data->direction_str != NULL) ? data->direction_str : "?", data->gps_position.latitude,
             data->gps_position.longitude, data->dust_value, data->temp_hum.temperature, data->temp_hum.humidity);

  if (written < 0 || written >= (int) buf_len)
  {
    LOG_WRN("Payload truncated: need %d, buf %u", written, (unsigned) buf_len);
    buf[0] = '\0';
    return false;
  }

#if (DEVICE_FUSION_DEBUG_MODE == 1)
  int dbg_written =
    snprintf(buf + written, buf_len - written,
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

  if (dbg_written < 0 || (written + dbg_written) >= (int) buf_len)
  {
    LOG_WRN("Debug payload truncated");
    buf[0] = '\0';
    return false;
  }
#else
  int close_written = snprintf(buf + written, buf_len - written, "}");
  if (close_written < 0 || (written + close_written) >= (int) buf_len)
  {
    buf[0] = '\0';
    return false;
  }
#endif

  // LOG_DBG("Payload built: %s", buf);
  return true;
}

static void sys_network_process_idle(void)
{
  OS_SEM_TAKE(sys_network_wakeup_sem, OS_MAX_DELAY);
}

static void sys_network_process_active(void)
{
  switch (network_ctx.state)
  {
  case NETWORK_STATE_SIM_INIT: sys_network_run_sim_init(); break;
  case NETWORK_STATE_SIM_WAIT_READY: sys_network_run_sim_wait_ready(); break;
  case NETWORK_STATE_MQTT_INIT: sys_network_run_mqtt_init(); break;
  case NETWORK_STATE_ONLINE:
  {
    sys_network_run_online();
    break;
  }
  case NETWORK_STATE_ERROR: sys_network_run_error_backoff(); break;
  case NETWORK_STATE_SIM_RESET: sys_network_run_sim_hard_reset(); break;
  default:
  {
    LOG_ERR("Unknown state %d — resetting", network_ctx.state);
    sys_network_change_state(NETWORK_STATE_SIM_INIT);
    break;
  }
  }

  if (network_ctx.state != NETWORK_STATE_ONLINE)
  {
    OS_DELAY_MS(OFFLINE_POLL_MS);
    return;
  }

  if (sys_network_need_fast_poll())
  {
    OS_DELAY_MS(ONLINE_FAST_POLL_MS);
  }
  else if (g_device_info.nvs_info.curr_state == DEVICE_STATE_LOCKED)
  {
    OS_DELAY_MS(ONLINE_LOCKED_POLL_MS);
  }
  else if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    OS_DELAY_MS(ONLINE_IDLE_POLL_MS);
  }
  else
  {
    OS_DELAY_MS(ONLINE_POLL_MS);
  }
}

static void sys_network_publish_online(void)
{
  if (!s_pub_slot_valid)
  {
    OS_MUTEX_LOCK(network_data_mutex);
    size_t available = cb_data_count(&network_ctx.cbuffer);
    if (available < NETWORK_CBUFF_SLOT_SIZE)
    {
      OS_MUTEX_UNLOCK(network_data_mutex);
      return;
    }
    size_t read = cb_read(&network_ctx.cbuffer, s_pub_slot, NETWORK_CBUFF_SLOT_SIZE);
    OS_MUTEX_UNLOCK(network_data_mutex);

    if (read != NETWORK_CBUFF_SLOT_SIZE)
    {
      LOG_WRN("Partial cbuffer read (%u bytes) — discarding", (unsigned) read);
      return;
    }
    s_pub_slot_valid = true;
  }

  mqtt_message_t mes = {
    .topic   = g_device_info.mqtt_data_topic,
    .payload = s_pub_slot,
  };

  bool pub_ok = false;
  for (uint8_t i = 0; i < MQTT_PUBLISH_RETRY; i++)
  {
    if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
    {
      pub_ok = true;
      break;
    }
  }
  if (!pub_ok)
  {
    LOG_WRN("Publish failed — keeping in-flight slot for next retry");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  s_pub_slot_valid            = false;
  network_ctx.last_publish_ms = OS_GET_TICK();
}

static void sys_network_flush_cbuff_to_sd(void)
{
  OS_MUTEX_LOCK(network_data_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(network_data_mutex);

  if (available < NETWORK_CBUFF_SLOT_SIZE)
  {
    return;
  }

  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    LOG_ERR("SD not mounted — cannot flush");
    return;
  }

  if (sys_network_prepare_sd_card() != STATUS_OK)
  {
    LOG_ERR("Cannot prepare offline directory");
    return;
  }

  bsp_sdcard_file_t off_log_handle;
  bsp_sdcard_mode_t mode;

  if (bsp_sdcard_file_exists(SD_OFFLINE_LOG_PATH) == STATUS_OK)
  {
    mode = BSP_SDCARD_MODE_APPEND;
  }
  else
  {
    mode = BSP_SDCARD_MODE_WRITE;
  }

  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, mode, &off_log_handle) != STATUS_OK)
  {
    LOG_ERR("Cannot open offline log for append");
    return;
  }

  size_t flushed = 0;

  while (1)
  {
    char slot[NETWORK_CBUFF_SLOT_SIZE];

    OS_MUTEX_LOCK(network_data_mutex);
    size_t avail = cb_data_count(&network_ctx.cbuffer);
    if (avail < NETWORK_CBUFF_SLOT_SIZE)
    {
      OS_MUTEX_UNLOCK(network_data_mutex);
      break;
    }
    size_t read = cb_read(&network_ctx.cbuffer, slot, NETWORK_CBUFF_SLOT_SIZE);
    OS_MUTEX_UNLOCK(network_data_mutex);

    if (read != NETWORK_CBUFF_SLOT_SIZE)
    {
      break;
    }

    size_t json_len    = strnlen(slot, NETWORK_CBUFF_SLOT_SIZE);
    slot[json_len]     = '\n';
    slot[json_len + 1] = '\0';

    size_t written_len = 0;
    bsp_sdcard_write(&off_log_handle, (const uint8_t *) slot, json_len + 1, &written_len);
    if (written_len != json_len + 1)
    {
      LOG_WRN("SD write incomplete (%u / %u bytes)", (unsigned) written_len, (unsigned) (json_len + 1));
    }

    flushed++;
  }

  bsp_sdcard_close(&off_log_handle);

  if (flushed > 0)
  {
    LOG_INF("Flushed %u records to SD", (unsigned) flushed);
    network_ctx.is_data_sd_pending = true;
  }
}

static status_function_t sys_network_push_sd_to_mqtt(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    LOG_ERR("SD not mounted — cannot drain");
    network_ctx.is_data_sd_pending = false;
    network_ctx.network_sd_offset  = 0;
    return STATUS_BUSY;
  }

  if (sys_network_prepare_sd_card() != STATUS_OK)
  {
    return STATUS_BUSY;
  }

  bsp_sdcard_file_t file_handle;
  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_READ, &file_handle) != STATUS_OK)
  {
    LOG_WRN("Offline log not found — clearing flag");
    network_ctx.is_data_sd_pending = false;
    network_ctx.network_sd_offset  = 0;
    return STATUS_BUSY;
  }

  size_t total_size = file_handle.file.size();
  if (network_ctx.network_sd_offset >= total_size)
  {
    bsp_sdcard_close(&file_handle);
    bsp_sdcard_delete(SD_OFFLINE_LOG_PATH);
    network_ctx.is_data_sd_pending = false;
    network_ctx.network_sd_offset  = 0;
    LOG_INF("Offline log fully drained and deleted");
    return STATUS_OK;
  }

  if (bsp_sdcard_seek(&file_handle, network_ctx.network_sd_offset) != STATUS_OK)
  {
    LOG_ERR("Cannot seek to offset %u", (unsigned) network_ctx.network_sd_offset);
    bsp_sdcard_close(&file_handle);
    return STATUS_BUSY;
  }

  size_t  line_len = 0;
  uint8_t ch;
  size_t  rd = 0;

  while (line_len < sizeof(s_sd_line) - 1)
  {
    if (bsp_sdcard_read(&file_handle, &ch, 1, &rd) != STATUS_OK || rd == 0)
    {
      break;
    }
    if (ch == '\n')
    {
      break;
    }
    s_sd_line[line_len++] = (char) ch;
  }

  bsp_sdcard_close(&file_handle);

  if (line_len == 0)
  {
    network_ctx.network_sd_offset++;
    return STATUS_BUSY;
  }

  s_sd_line[line_len] = '\0';

  mqtt_message_t mes = {
    .topic   = g_device_info.mqtt_data_topic,
    .payload = s_sd_line,
  };

  bool publish_ok = false;
  for (uint8_t attempt = 0; attempt < MQTT_PUBLISH_RETRY; attempt++)
  {
    if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
    {
      publish_ok = true;
      break;
    }
    if (attempt + 1 < MQTT_PUBLISH_RETRY)
    {
      OS_DELAY_MS(MQTT_PUBLISH_RETRY_DELAY_MS);
    }
  }

  if (!publish_ok)
  {
    LOG_WRN("Failed to publish SD record after %d attempts — will retry", MQTT_PUBLISH_RETRY);
    sys_network_change_state(NETWORK_STATE_ERROR);
    return STATUS_ERROR;
  }

  network_ctx.network_sd_offset += line_len + 1;

  if (network_ctx.network_sd_offset >= total_size)
  {
    bsp_sdcard_delete(SD_OFFLINE_LOG_PATH);
    network_ctx.is_data_sd_pending = false;
    network_ctx.network_sd_offset  = 0;
    LOG_INF("Offline log fully drained and deleted");
  }

  return STATUS_OK;
}

static bool sys_network_need_fast_poll(void)
{
  if (network_ctx.is_data_sd_pending)
  {
    return true;
  }

  OS_MUTEX_LOCK(network_data_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(network_data_mutex);

  return (available >= (CBUFFER_FAST_MSG_THRESHOLD * NETWORK_CBUFF_SLOT_SIZE));
}

static bool sys_network_need_push_sd(void)
{
  if (network_ctx.state == NETWORK_STATE_ONLINE)
  {
    return false;
  }

  if (network_ctx.cbuffer.size <= 1)
  {
    return false;
  }

  OS_MUTEX_LOCK(network_data_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(network_data_mutex);

  size_t total_bytes  = network_ctx.cbuffer.size - 1;
  size_t used_percent = (available * 100) / total_bytes;

  return (used_percent >= NETWORK_CBUFF_FLUSH_THRESH);
}

static bool sys_network_check_pending(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return false;
  }

  if (sys_network_prepare_sd_card() != STATUS_OK)
  {
    return false;
  }

  bsp_sdcard_file_t off_log_handle;
  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_READ, &off_log_handle) != STATUS_OK)
  {
    return false;
  }

  bool non_empty = (off_log_handle.file.size() > 0);
  bsp_sdcard_close(&off_log_handle);
  return non_empty;
}

static status_function_t sys_network_prepare_sd_card(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  if (bsp_sdcard_dir_exists(SD_OFFLINE_DIR) != STATUS_OK)
  {
    if (bsp_sdcard_mkdir(SD_OFFLINE_DIR) != STATUS_OK)
    {
      LOG_ERR("Cannot create offline directory: %s", SD_OFFLINE_DIR);
      return STATUS_ERROR;
    }
  }

  if (bsp_sdcard_file_exists(SD_OFFLINE_LOG_PATH) == STATUS_OK)
  {
    LOG_DBG("Offline log exists on SD");
    return STATUS_OK;
  }

  bsp_sdcard_file_t file_handle;
  for (uint8_t i = 0; i < SD_CARD_RETRY_COUNT; i++)
  {
    if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_WRITE, &file_handle) == STATUS_OK)
    {
      bsp_sdcard_close(&file_handle);
      return STATUS_OK;
    }
    OS_DELAY_MS(SD_CARD_RETRY_DELAY_MS);
  }

  LOG_ERR("Cannot create offline log after %d attempts", SD_CARD_RETRY_COUNT);
  return STATUS_ERROR;
}

static status_function_t sys_network_push_cbuffer(const char *payload)
{
  if (payload == NULL)
  {
    return STATUS_ERROR;
  }

  char slot[NETWORK_CBUFF_SLOT_SIZE];
  memset(slot, 0, sizeof(slot));
  strncpy(slot, payload, NETWORK_CBUFF_SLOT_SIZE - 1);

  OS_MUTEX_LOCK(network_data_mutex);
  size_t written = cb_write(&network_ctx.cbuffer, slot, NETWORK_CBUFF_SLOT_SIZE);
  OS_MUTEX_UNLOCK(network_data_mutex);

  if (written != NETWORK_CBUFF_SLOT_SIZE)
  {
    LOG_WRN("Cbuffer full — record dropped (overflow=%u)", (unsigned) network_ctx.cbuffer.overflow);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */
