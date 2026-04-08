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
LOG_MODULE_REGISTER(sys_network, LOG_LEVEL_INFO)

// MQTT
#define MQTT_KEEPALIVE_S            (60)
#define MQTT_QOS                    (1)
#define MQTT_KEEPALIVE_MS           (MQTT_KEEPALIVE_S * 1000UL)
#define MQTT_PUBLISH_RETRY          (3)
#define MQTT_PUBLISH_RETRY_DELAY_MS (100)
#if (DEVICE_FUSION_DEBUG_MODE == 1)
#define MQTT_MESSAGE_MAX_LEN (1024)
#else
#define MQTT_MESSAGE_MAX_LEN (512)
#endif

// Timming
#define OFFLINE_POLL_MS               (100)
#define ONLINE_FAST_POLL_MS           (50)
#define ONLINE_POLL_MS                (500)
#define NETWORK_DATA_TASK_POLLMS      (1000)
#define SIM_READY_TIMEOUT_MS          (10000)
#define SIM_HARD_RESET_DELAY_MS       (2000)
#define MQTT_INIT_TIMEOUT_MS          (15000)

// Retry, backoff, and reset
#define BACKOFF_BASE_MS               (2000)
#define BACKOFF_MAX_MS                (32000)
#define RETRY_MAX_BEFORE_RESET        (3)

// Cbuffer and SD
#define NETWORK_CBUFF_COUNT           (50)
#define NETWORK_BYTES                 (NETWORK_CBUFF_COUNT * sizeof(sys_input_data_t))
#define NETWORK_CBUFF_FLUSH_THRESHOLD (80)
#define NETWORK_PUBLISH_BATCH         (4)
#define CBUFFER_FAST_MSG_THRESHOLD    (2)
#define CBUFFER_CBUFFER_MSG_THRESHOLD (40)
#define SD_OFFLINE_DIR                "/buff"
#define SD_OFFLINE_LOG_PATH           "/buff/offline_log.csv"
#define SD_CSV_LINE_MAX_LEN           (256)

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
  bool cbuffer_sending;
  bool is_data_sd_pending;
  bool mqtt_last_payload_valid;

  char             sd_csv_line[SD_CSV_LINE_MAX_LEN];
  cbuffer_t        cbuffer;
  sys_input_data_t data_input_buffer;

} net_ctx_t;

/* Private macros ----------------------------------------------------- */
#define COUNT_MS(since_ms) ((size_t) (OS_GET_TICK() - (since_ms)))

/* Public variables --------------------------------------------------- */
bool is_data_network_ready = false;

/* Private variables -------------------------------------------------- */
static net_ctx_t network_ctx;

static char    mqtt_payload_buffer[MQTT_MESSAGE_MAX_LEN];
static char    mqtt_last_payload[MQTT_MESSAGE_MAX_LEN];
static uint8_t network_buffer[NETWORK_BYTES];

OS_MUTEX_DEFINE_STATIC(s_telem_mutex);
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

static bool sys_network_build_payload(sys_input_data_t *data);
static void sys_network_mqtt_message_cb(const char *topic, const uint8_t *data, size_t len);

static void              sys_network_publish_online(void);
static void              sys_network_flush_cbuff_to_sd(void);
static status_function_t sys_network_push_sd_to_mqtt(void);
static bool              sys_network_check_pending(void);
static bool              sys_network_need_push_sd(void);
static bool              sys_network_need_fast_poll(void);
static status_function_t sys_network_prepare_sd_card(void);
static status_function_t sys_network_push_cbuffer(const sys_input_data_t *data);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  network_ctx.is_sys_network_init = false;

  memset(&network_ctx, 0, sizeof(network_ctx));
  network_ctx.state      = NETWORK_STATE_SIM_INIT;
  network_ctx.prev_state = NETWORK_STATE_SIM_INIT;

  OS_SEM_CREATE(sys_network_wakeup_sem);
  OS_MUTEX_CREATE(s_telem_mutex);

  cb_init(&network_ctx.cbuffer, network_buffer, NETWORK_BYTES);

  (void) sys_network_prepare_sd_card();

  network_ctx.is_data_sd_pending      = sys_network_check_pending();
  network_ctx.network_sd_offset       = 0;
  network_ctx.mqtt_last_payload_valid = false;
  if (network_ctx.is_data_sd_pending)
  {
    LOG_DBG("Offline log found on SD — will drain after reconnect");
  }

  network_ctx.is_sys_network_init = true;
}

void sys_network_process(void *param)
{
  while (1)
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
}

void sys_network_wakeup(void)
{
  OS_SEM_GIVE(sys_network_wakeup_sem);
}

void sys_network_data_task(void *param)
{
  while (1)
  {
    if (!network_ctx.is_sys_network_init)
    {
      OS_DELAY_MS(NETWORK_DATA_TASK_POLLMS);
      continue;
    }

    if ((g_device_info.state == DEVICE_STATE_ACTIVE) && is_data_network_ready)
    {
      is_data_network_ready = false;

      sys_input_data_t input_data;
      if (sys_input_get_data(&input_data) == STATUS_OK)
      {
        sys_network_push_cbuffer(&input_data);
      }
    }

    if (sys_network_need_push_sd())
    {
      sys_network_flush_cbuff_to_sd();
    }

    OS_DELAY_MS(NETWORK_DATA_TASK_POLLMS);
  }
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

  if (bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_mqtt_message_cb) != STATUS_OK)
  {
    LOG_WRN("MQTT subscribe failed");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  LOG_DBG("MQTT connected");
  network_ctx.mqtt_ready        = true;
  network_ctx.retry_count       = 0;
  network_ctx.last_publish_ms   = OS_GET_TICK();
  network_ctx.last_keepalive_ms = OS_GET_TICK();

  sys_network_change_state(NETWORK_STATE_ONLINE);
}

static void sys_network_run_online(void)
{
  // 1. Check SD card first
  if (network_ctx.is_data_sd_pending)
  {
    status_function_t res = sys_network_push_sd_to_mqtt();
    if ((res == STATUS_OK) || (res == STATUS_ERROR))
    {
      return;
    }
    else
    {
      // Keep going
    }
  }

  // 2. Publish new data in cbuffer if available
  sys_network_publish_online();
}

static void sys_network_run_error_backoff(void)
{
  size_t backoff = sys_network_backoff(network_ctx.retry_count);
  size_t elapsed = COUNT_MS(network_ctx.state_enter_ms);

  if (elapsed < backoff)
  {
    // Progress log every 5 s to avoid log spam
    if (COUNT_MS(network_ctx.last_log_ms) >= 5000)
    {
      LOG_DBG("Error SIM");
      network_ctx.last_log_ms = OS_GET_TICK();
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
             "\"direction\":\"%.1f %s\","
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

  LOG_DBG("MQTT payload: %s", mqtt_payload_buffer);
  return true;
}

static void sys_network_process_idle(void)
{
  // TODO: Enter low-power mode, deinit SIM, etc.
  if (network_ctx.mqtt_ready || network_ctx.sim_ready)
  {
    bsp_sim_mqtt_deinit();
    network_ctx.sim_ready  = false;
    network_ctx.mqtt_ready = false;
  }
  if (network_ctx.state != NETWORK_STATE_SIM_INIT)
    sys_network_change_state(NETWORK_STATE_SIM_INIT);

  OS_SEM_TAKE(sys_network_wakeup_sem, OS_MAX_DELAY);
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

  if (network_ctx.state != NETWORK_STATE_ONLINE)
  {
    OS_DELAY_MS(OFFLINE_POLL_MS);
    return;
  }

  if (sys_network_need_fast_poll())
  {
    OS_DELAY_MS(ONLINE_FAST_POLL_MS);
  }
  else
  {
    OS_DELAY_MS(ONLINE_POLL_MS);
  }
}

static bool sys_network_need_fast_poll(void)
{
  if (network_ctx.is_data_sd_pending)
  {
    return true;
  }

  OS_MUTEX_LOCK(s_telem_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(s_telem_mutex);

  return (available > (CBUFFER_FAST_MSG_THRESHOLD * sizeof(sys_input_data_t)));
}

static bool sys_network_need_push_sd(void)
{
  if (network_ctx.is_data_sd_pending)
  {
    return true;
  }

  OS_MUTEX_LOCK(s_telem_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(s_telem_mutex);

  return (available > (CBUFFER_CBUFFER_MSG_THRESHOLD * sizeof(sys_input_data_t)));
}

static void sys_network_publish_online(void)
{
  if (!network_ctx.cbuffer_sending)
  {
    OS_MUTEX_LOCK(s_telem_mutex);
    size_t available = cb_data_count(&network_ctx.cbuffer);
    OS_MUTEX_UNLOCK(s_telem_mutex);

    if (available < sizeof(sys_input_data_t))
    {
      return;
    }

    OS_MUTEX_LOCK(s_telem_mutex);
    size_t read = cb_read(&network_ctx.cbuffer, &network_ctx.data_input_buffer, sizeof(sys_input_data_t));
    OS_MUTEX_UNLOCK(s_telem_mutex);

    if (read != sizeof(sys_input_data_t))
    {
      LOG_WRN("Partial cbuffer read (%lu bytes) — discarding", read);
      return;
    }

    network_ctx.cbuffer_sending = true;
  }

  if (!sys_network_build_payload(&network_ctx.data_input_buffer))
  {
    LOG_WRN("Payload build failed — record discarded");
    network_ctx.cbuffer_sending = false;
    return;
  }

  mqtt_message_t msg = {
    .topic   = g_device_info.mqtt_data_topic,
    .payload = mqtt_payload_buffer,
  };

  if (network_ctx.mqtt_last_payload_valid && (strcmp(mqtt_payload_buffer, mqtt_last_payload) == 0))
  {
    network_ctx.cbuffer_sending = false;
    network_ctx.last_publish_ms = OS_GET_TICK();
    return;
  }

  if (bsp_sim_mqtt_pub(&msg) != STATUS_OK)
  {
    LOG_WRN("Publish failed — keep in-flight record for next retry");
    sys_network_change_state(NETWORK_STATE_ERROR);
    return;
  }

  strncpy(mqtt_last_payload, mqtt_payload_buffer, sizeof(mqtt_last_payload) - 1);
  mqtt_last_payload[sizeof(mqtt_last_payload) - 1] = '\0';
  network_ctx.mqtt_last_payload_valid              = true;

  network_ctx.cbuffer_sending = false;
  network_ctx.last_publish_ms = OS_GET_TICK();
}

static void sys_network_flush_cbuff_to_sd(void)
{
  OS_MUTEX_LOCK(s_telem_mutex);
  size_t available = cb_data_count(&network_ctx.cbuffer);
  OS_MUTEX_UNLOCK(s_telem_mutex);

  if (available < sizeof(sys_input_data_t))
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

  bsp_sdcard_file_t f;
  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_APPEND, &f) != STATUS_OK)
  {
    LOG_ERR("Cannot open offline log for append");
    return;
  }

  size_t flushed = 0;

  while (1)
  {
    OS_MUTEX_LOCK(s_telem_mutex);
    size_t available = cb_data_count(&network_ctx.cbuffer);
    OS_MUTEX_UNLOCK(s_telem_mutex);

    if (available < sizeof(sys_input_data_t))
    {
      break;
    }

    OS_MUTEX_LOCK(s_telem_mutex);
    size_t read = cb_read(&network_ctx.cbuffer, &network_ctx.data_input_buffer, sizeof(sys_input_data_t));
    OS_MUTEX_UNLOCK(s_telem_mutex);

    if (read != sizeof(sys_input_data_t))
    {
      break;
    }

    int len = snprintf(
      network_ctx.sd_csv_line, sizeof(network_ctx.sd_csv_line),
      "%lu,%.3f,%.2f,%.1f,%.1f,%s,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f\n", network_ctx.data_input_buffer.timestamp_ms,
      network_ctx.data_input_buffer.velocity_ms, network_ctx.data_input_buffer.velocity_kmh,
      network_ctx.data_input_buffer.distance_m, network_ctx.data_input_buffer.heading_deg,
      (network_ctx.data_input_buffer.direction_str != NULL) ? network_ctx.data_input_buffer.direction_str : "?",
      network_ctx.data_input_buffer.gps_position.latitude, network_ctx.data_input_buffer.gps_position.longitude,
      network_ctx.data_input_buffer.dust_value, network_ctx.data_input_buffer.temp_hum.temperature,
      network_ctx.data_input_buffer.temp_hum.humidity, network_ctx.data_input_buffer.battery_level);

    if (len <= 0 || len >= (int) sizeof(network_ctx.sd_csv_line))
    {
      LOG_WRN("CSV line truncated — skipping record");
      continue;
    }

    size_t written_len = 0;
    bsp_sdcard_write(&f, (const uint8_t *) network_ctx.sd_csv_line, (size_t) len, &written_len);
    if (written_len != (size_t) len)
    {
      LOG_WRN("SD write incomplete (%u / %d bytes)", (unsigned) written_len, len);
    }

    flushed++;
  }

  bsp_sdcard_close(&f);

  if (flushed > 0)
  {
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
    return STATUS_OK;
  }

  if (bsp_sdcard_seek(&file_handle, network_ctx.network_sd_offset) != STATUS_OK)
  {
    LOG_ERR("Cannot seek offline log offset=%u", (unsigned) network_ctx.network_sd_offset);
    bsp_sdcard_close(&file_handle);
    return STATUS_BUSY;
  }

  size_t  offset_next_line = 0;
  uint8_t char_buff;
  size_t  line_len = 0;
  size_t  rd       = 0;
  while (line_len < sizeof(network_ctx.sd_csv_line) - 1)
  {
    if (bsp_sdcard_read(&file_handle, &char_buff, 1, &rd) != STATUS_OK || rd == 0)
    {
      break;
    }

    network_ctx.sd_csv_line[line_len++] = (char) char_buff;
    if (char_buff == '\n')
    {
      break;
    }
  }

  if (line_len == 0)
  {
    bsp_sdcard_close(&file_handle);
    return STATUS_BUSY;
  }

  network_ctx.sd_csv_line[line_len] = '\0';
  static char dir_buf[8]            = { 0 };
  int         parsed                = sscanf(
    network_ctx.sd_csv_line, "%lu,%f,%f,%f,%f,%7[^,],%f,%f,%f,%f,%f,%f", &network_ctx.data_input_buffer.timestamp_ms,
    &network_ctx.data_input_buffer.velocity_ms, &network_ctx.data_input_buffer.velocity_kmh,
    &network_ctx.data_input_buffer.distance_m, &network_ctx.data_input_buffer.heading_deg, dir_buf,
    &network_ctx.data_input_buffer.gps_position.latitude, &network_ctx.data_input_buffer.gps_position.longitude,
    &network_ctx.data_input_buffer.dust_value, &network_ctx.data_input_buffer.temp_hum.temperature,
    &network_ctx.data_input_buffer.temp_hum.humidity, &network_ctx.data_input_buffer.battery_level);

  if (parsed < 12)
  {
    LOG_WRN("Malformed CSV line (parsed %d/12) — dropping one line", parsed);
    offset_next_line = line_len;
  }
  else
  {
    network_ctx.data_input_buffer.direction_str = dir_buf;

    if (!sys_network_build_payload(&network_ctx.data_input_buffer))
    {
      LOG_WRN("Payload build failed for SD record — keeping line for retry");
      bsp_sdcard_close(&file_handle);
      return STATUS_BUSY;
    }

    mqtt_message_t msg = {
      .topic   = g_device_info.mqtt_data_topic,
      .payload = mqtt_payload_buffer,
    };

    if (network_ctx.mqtt_last_payload_valid && (strcmp(mqtt_payload_buffer, mqtt_last_payload) == 0))
    {
      LOG_DBG("Skip duplicate SD payload");
      offset_next_line = line_len;
      bsp_sdcard_close(&file_handle);
      goto _NET_LINE;
    }

    bool publish_ok = false;
    for (uint8_t attempt = 0; attempt < MQTT_PUBLISH_RETRY; attempt++)
    {
      if (bsp_sim_mqtt_pub(&msg) == STATUS_OK)
      {
        publish_ok = true;
        break;
      }
    }

    if (!publish_ok)
    {
      LOG_WRN("Failed to publish SD record after %d attempts — keeping line for retry", MQTT_PUBLISH_RETRY);
      bsp_sdcard_close(&file_handle);
      return STATUS_ERROR;
    }

    strncpy(mqtt_last_payload, mqtt_payload_buffer, sizeof(mqtt_last_payload) - 1);
    mqtt_last_payload[sizeof(mqtt_last_payload) - 1] = '\0';
    network_ctx.mqtt_last_payload_valid              = true;

    offset_next_line = line_len;
  }

  bsp_sdcard_close(&file_handle);
_NET_LINE:
  if (offset_next_line == 0)
  {
    return STATUS_BUSY;
  }

  network_ctx.network_sd_offset += offset_next_line;
  network_ctx.is_data_sd_pending = true;
  if (network_ctx.network_sd_offset >= total_size)
  {
    bsp_sdcard_delete(SD_OFFLINE_LOG_PATH);
    network_ctx.is_data_sd_pending = false;
    network_ctx.network_sd_offset  = 0;
    LOG_DBG("Offline log fully drained and deleted");
  }
  else
  {
    network_ctx.is_data_sd_pending = true;
  }

  return STATUS_OK;
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

  bsp_sdcard_file_t f;
  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_READ, &f) != STATUS_OK)
  {
    return false;
  }

  bool non_empty = (f.file.size() > 0);
  bsp_sdcard_close(&f);
  return non_empty;
}

static status_function_t sys_network_prepare_sd_card(void)
{
  bsp_sdcard_file_t file_handle;

  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  if (bsp_sdcard_mkdir(SD_OFFLINE_DIR) == STATUS_OK)
  {
    return STATUS_OK;
  }

  if (bsp_sdcard_open(SD_OFFLINE_LOG_PATH, BSP_SDCARD_MODE_APPEND, &file_handle) == STATUS_OK)
  {
    bsp_sdcard_close(&file_handle);
    return STATUS_OK;
  }

  return STATUS_ERROR;
}

static status_function_t sys_network_push_cbuffer(const sys_input_data_t *data)
{
  if (data == NULL)
  {
    return STATUS_ERROR;
  }

  OS_MUTEX_LOCK(s_telem_mutex);
  size_t written = cb_write(&network_ctx.cbuffer, (void *) data, sizeof(sys_input_data_t));
  OS_MUTEX_UNLOCK(s_telem_mutex);

  if (written != sizeof(sys_input_data_t))
  {
    LOG_WRN("Telemetry cbuffer full — record dropped (overflow=%lu)", network_ctx.cbuffer.overflow);
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

/* End of file -------------------------------------------------------- */