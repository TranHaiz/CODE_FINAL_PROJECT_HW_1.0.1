/**
 * @file       sys_network.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      Network dispatcher — transport-agnostic.
 *             Owns the data path (cbuffer + SD) and switches between
 *             g_net_adapter_lte and g_net_adapter_ble at runtime.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network.h"

#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "cbuffer.h"
#include "device_config.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_cmd.h"
#include "sys_input.h"
#include "sys_led.h"
#include "sys_manager.h"
#include "sys_network_adapter_ble.h"
#include "sys_network_adapter_lte.h"
#include "sys_ui_simple.h"

#include <esp_heap_caps.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_network, LOG_LEVEL_SYS_NETWORK)

#define NETWORK_DATA_TASK_POLL_MS   (700)
#define NETWORK_CBUFF_SLOT_SIZE     (1024)
#define NETWORK_CBUFF_COUNT         (100)
#define NETWORK_BYTES               (NETWORK_CBUFF_COUNT * NETWORK_CBUFF_SLOT_SIZE)
#define NETWORK_CBUFF_FLUSH_THRESH  (80)
#define MQTT_REQUEST_PUBLISH_MAX    (10)
#define SD_OFFLINE_DIR              "/offline"
#define SD_TRIP_PREFIX              "/offline/trip_"
#define SD_JSON_LINE_MAX_LEN        (NETWORK_CBUFF_SLOT_SIZE + 2)
#define SD_CARD_RETRY_COUNT         (3)
#define SD_CARD_RETRY_DELAY_MS      (100)
#define MQTT_PUBLISH_RETRY_DELAY_MS (100)
#define MAX_UPLOAD_TRIPS            (100)
#define MAX_OFFLINE_TRIP_PATH_LEN   (128)
#define MAX_OFFLINE_TRIP_NAME_LEN   (64)

/* Private variables -------------------------------------------------- */
static bool s_is_init = false;

static uint8_t  *network_buffer = NULL;
static cbuffer_t network_cbuffer;
static cbuffer_t req_pub_cbuffer;
static char      req_pub_buffer[MQTT_REQUEST_PUBLISH_MAX * MQTT_REQUEST_PUBLISH_SIZE];

static bool   s_is_data_sd_pending = false;
static size_t s_network_sd_offset  = 0;
static char   s_sd_line[SD_JSON_LINE_MAX_LEN];
static char   s_pub_slot[NETWORK_CBUFF_SLOT_SIZE];
static bool   s_pub_slot_valid = false;

static uint32_t s_active_trip_id                        = 0;
static char     s_current_upload_trip[MAX_UPLOAD_TRIPS] = { 0 };

volatile bool is_data_network_ready = false;

OS_MUTEX_DEFINE_STATIC(net_data_mutex);
OS_MUTEX_DEFINE_STATIC(net_noti_mutex);
OS_SEM_DEFINE_STATIC(net_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
static void              sys_network_data_received_callback(const uint8_t *data, size_t len);
static bool              sys_network_build_payload(sys_input_data_t *data, char *buf, size_t buf_len);
static status_function_t sys_network_push_cbuffer(const char *payload);
static void              sys_network_flush_cbuffer_to_sd(void);
static status_function_t sys_network_push_sd_to_adapter(net_adapter_t *adapter);
static bool              sys_network_publish_from_cbuffer(net_adapter_t *adapter);
static bool              sys_network_need_push_sd(void);
static status_function_t sys_network_prepare_sd_card(void);
static bool              sys_network_check_sd_pending(void);
static void              sys_network_write_trip_info(const char *meta_path, uint32_t trip_id, trip_state_t state);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  s_is_init = false;

  OS_SEM_CREATE(net_wakeup_sem);
  OS_MUTEX_CREATE(net_data_mutex);
  OS_MUTEX_CREATE(net_noti_mutex);

  if (network_buffer == NULL)
  {
    network_buffer = (uint8_t *) heap_caps_malloc(NETWORK_BYTES, MALLOC_CAP_SPIRAM);
    if (network_buffer == NULL)
    {
      LOG_ERR("PSRAM alloc failed (%u bytes) — network disabled", (unsigned) NETWORK_BYTES);
      return;
    }
  }

  cb_init(&network_cbuffer, network_buffer, NETWORK_BYTES);
  cb_init(&req_pub_cbuffer, req_pub_buffer, MQTT_REQUEST_PUBLISH_MAX * MQTT_REQUEST_PUBLISH_SIZE);

  (void) sys_network_prepare_sd_card();
  s_is_data_sd_pending = sys_network_check_sd_pending();
  if (s_is_data_sd_pending)
  {
    LOG_INF("Offline log found on SD — will drain after reconnect");
  }

  /* Init adapters — both register sys_network_data_received_callback */
  g_net_adapter_lte.init(sys_network_data_received_callback);
  g_net_adapter_ble.init(sys_network_data_received_callback);

  s_is_init = true;
}

void sys_network_wakeup(void)
{
  OS_SEM_GIVE(net_wakeup_sem);
}

void sys_network_trigger_new_trip(void)
{
  uint32_t new_trip_id = 1;
  char     info_path[MAX_OFFLINE_TRIP_PATH_LEN];
  snprintf(info_path, sizeof(info_path), "%s/trip_info.txt", SD_OFFLINE_DIR);

  if (sys_network_prepare_sd_card() == STATUS_OK)
  {
    bsp_sdcard_file_t trip_info_file;
    size_t            read_len     = 0;
    char              info_buf[32] = { 0 };
    if (bsp_sdcard_open(info_path, BSP_SDCARD_MODE_READ, &trip_info_file) == STATUS_OK)
    {
      bsp_sdcard_read(&trip_info_file, (uint8_t *) info_buf, sizeof(info_buf) - 1, &read_len);
      bsp_sdcard_close(&trip_info_file);
    }
    if (read_len > 0)
    {
      size_t saved_id = 0;
      sscanf(info_buf, "%lu", &saved_id);
      if (saved_id > 0)
        new_trip_id = (uint32_t) saved_id + 1;
    }

    sys_network_write_trip_info(info_path, new_trip_id, TRIP_ACTIVE);
  }

  s_active_trip_id         = new_trip_id;
  s_current_upload_trip[0] = '\0';
  s_is_data_sd_pending     = false;
  LOG_INF("New trip started: %lu", new_trip_id);
}

void sys_network_trigger_end_trip(void)
{
  if (s_active_trip_id != 0 && sys_network_prepare_sd_card() == STATUS_OK)
  {
    char info_path[MAX_OFFLINE_TRIP_PATH_LEN];
    snprintf(info_path, sizeof(info_path), "%s/trip_info.txt", SD_OFFLINE_DIR);
    sys_network_write_trip_info(info_path, s_active_trip_id, TRIP_COMPLETED);
  }

  LOG_INF("Trip %lu ended", s_active_trip_id);
  s_active_trip_id     = 0;
  s_is_data_sd_pending = sys_network_check_sd_pending();
}

void sys_network_publish_noti(const char *payload, size_t payload_len)
{
  if (payload == NULL || payload_len == 0 || payload_len >= MQTT_REQUEST_PUBLISH_SIZE)
  {
    return;
  }

  char slot[MQTT_REQUEST_PUBLISH_SIZE];
  memset(slot, 0, sizeof(slot));
  memcpy(slot, payload, payload_len);

  OS_MUTEX_LOCK(net_noti_mutex);
  uint32_t written = cb_write(&req_pub_cbuffer, slot, MQTT_REQUEST_PUBLISH_SIZE);
  OS_MUTEX_UNLOCK(net_noti_mutex);

  if (written != MQTT_REQUEST_PUBLISH_SIZE)
  {
    LOG_WRN("req_pub cbuffer full — noti dropped");
  }
}

void sys_network_task(void *param)
{
  net_adapter_t *active         = &g_net_adapter_lte;
  uint8_t        lte_lost_count = 0;

  while (1)
  {
    if (!s_is_init)
    {
      OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
      continue;
    }

    /* --- 1. Collect new telemetry from sys_input --- */
    if (((g_device_info.nvs_info.curr_state == DEVICE_STATE_ACTIVE)
         || (g_device_info.nvs_info.curr_state == DEVICE_STATE_STOLEN))
        && is_data_network_ready)
    {
      is_data_network_ready = false;
      sys_input_data_t input;
      if (sys_input_get_data(&input) == STATUS_OK)
      {
        char payload[NETWORK_CBUFF_SLOT_SIZE];
        if (sys_network_build_payload(&input, payload, sizeof(payload)))
        {
          sys_network_push_cbuffer(payload);
        }
      }
    }

    /* --- 2. Adapter switching --- */
    bool lte_ready = g_net_adapter_lte.is_ready();
    bool ble_ready = g_net_adapter_ble.is_ready();

    if (active == &g_net_adapter_ble && lte_ready)
    {
      active           = &g_net_adapter_lte;
      lte_lost_count   = 0;
      s_pub_slot_valid = false;
#if (DEVICE_BLE_FALLBACK_ENABLED)
      sys_network_adapter_ble_set_advertise(false);
#endif
      LOG_INF("Network: BLE => LTE");
    }
    else if (active == &g_net_adapter_lte)
    {
      if (lte_ready)
      {
#if (DEVICE_BLE_FALLBACK_ENABLED)
        if (lte_lost_count >= NETWORK_SWITCH_LOST_THRESHOLD)
          sys_network_adapter_ble_set_advertise(false);
#endif
        lte_lost_count = 0;
      }
      else
      {
        if (lte_lost_count < NETWORK_SWITCH_LOST_THRESHOLD)
          lte_lost_count++;

        if (lte_lost_count >= NETWORK_SWITCH_LOST_THRESHOLD)
        {
#if (DEVICE_BLE_FALLBACK_ENABLED)
          sys_network_adapter_ble_set_advertise(true);
#endif
          if (ble_ready)
          {
            active           = &g_net_adapter_ble;
            s_pub_slot_valid = false;
            LOG_INF("Network: LTE => BLE (lost=%d)", lte_lost_count);
          }
        }
      }
    }

    /* --- 3. Flush cbuffer to SD when offline and buffer getting full --- */
    if (!active->is_ready() && sys_network_need_push_sd())
    {
      sys_network_flush_cbuffer_to_sd();
    }

    if (!active->is_ready())
    {
      OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
      continue;
    }

    /* --- 4. Drain noti queue (priority over data) --- */
    char     req_payload[MQTT_REQUEST_PUBLISH_SIZE] = { 0 };
    uint32_t req_count                              = 0;

    OS_MUTEX_LOCK(net_noti_mutex);
    if (cb_data_count(&req_pub_cbuffer) >= MQTT_REQUEST_PUBLISH_SIZE)
    {
      req_count = cb_read(&req_pub_cbuffer, req_payload, MQTT_REQUEST_PUBLISH_SIZE);
    }
    OS_MUTEX_UNLOCK(net_noti_mutex);

    if (req_count != 0)
    {
      size_t noti_len = strnlen(req_payload, MQTT_REQUEST_PUBLISH_SIZE);
      if (active->publish(NET_CH_NOTI, (const uint8_t *) req_payload, noti_len) != STATUS_OK)
      {
        LOG_WRN("Noti publish failed via %s", active->name);
      }
      OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
      continue;
    }

    // --- 5. Drain SD first, then cbuffer.
    // check_sd_pending() ensures active trip's SD takes priority > old backlog. */
    if (s_is_data_sd_pending)
    {
      sys_network_push_sd_to_adapter(active);
    }
    else
    {
      sys_network_publish_from_cbuffer(active);
    }

    OS_DELAY_MS(NETWORK_DATA_TASK_POLL_MS);
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_network_data_received_callback(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0 || len >= CMD_INPUT_MAX_LEN)
  {
    LOG_WRN("Invalid incoming command: %d bytes", (int) len);
    return;
  }
  memset(g_cmd_input_buffer, 0, CMD_INPUT_MAX_LEN);
  strncpy(g_cmd_input_buffer, (const char *) data, len);
  OS_SEM_GIVE(sys_cmd_req_sem);
}

static bool sys_network_build_payload(sys_input_data_t *data, char *buf, size_t buf_len)
{
  if (data == NULL || buf == NULL || buf_len == 0)
    return false;

  timeline_t now;
  memset(&now, 0, sizeof(now));
  bsp_rtc_get(&now);

#if (DEVICE_NETWORK_TOTAL_KM_ENABLED)
  int written =
    snprintf(buf, buf_len,
             "{"
             "\"time\":\"%d/%02d/%02d-%02d:%02d:%02d\","
             "\"battery\":%.1f,"
             "\"velocity_ms\":%.2f,"
             "\"velocity_kmh\":%.2f,"
             "\"distance_m\":%.1f,"
             "\"totalKm\":%.1f,"
             "\"direction_deg\":%.1f,"
             "\"direction_str\":\"%s\","
             "\"position\":[%.6f,%.6f],"
             "\"dust\":%.1f,"
             "\"temp\":%.1f,"
             "\"hum\":%.1f",
             now.year, now.month, now.date, now.hour, now.minute, now.second, data->battery_level, data->velocity_ms,
             data->velocity_kmh, data->distance_m, g_device_info.nvs_info.total_km, data->heading_deg,
             (data->direction_str != NULL) ? data->direction_str : "?", data->gps_position.latitude,
             data->gps_position.longitude, data->dust_value, data->temp_hum.temperature, data->temp_hum.humidity);
#else
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
#endif

  if (written < 0 || written >= (int) buf_len)
  {
    LOG_WRN("Payload truncated: need %d, buf %u", written, (unsigned) buf_len);
    buf[0] = '\0';
    return false;
  }

#if (DEVICE_FUSION_DEBUG_MODE && DEVICE_FUSION_DEBUG_VIA_NETWORK)
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
    buf[0] = '\0';
    return false;
  }
#else
  int close = snprintf(buf + written, buf_len - written, "}");
  if (close < 0 || (written + close) >= (int) buf_len)
  {
    buf[0] = '\0';
    return false;
  }
#endif
  return true;
}

static status_function_t sys_network_push_cbuffer(const char *payload)
{
  if (payload == NULL)
    return STATUS_ERROR;

  char slot[NETWORK_CBUFF_SLOT_SIZE];
  memset(slot, 0, sizeof(slot));
  strncpy(slot, payload, NETWORK_CBUFF_SLOT_SIZE - 1);

  OS_MUTEX_LOCK(net_data_mutex);
  size_t written = cb_write(&network_cbuffer, slot, NETWORK_CBUFF_SLOT_SIZE);
  OS_MUTEX_UNLOCK(net_data_mutex);

  if (written != NETWORK_CBUFF_SLOT_SIZE)
  {
    LOG_WRN("cbuffer full — record dropped (overflow=%u)", (unsigned) network_cbuffer.overflow);
    return STATUS_ERROR;
  }
  return STATUS_OK;
}

static bool sys_network_publish_from_cbuffer(net_adapter_t *adapter)
{
  if (!s_pub_slot_valid)
  {
    OS_MUTEX_LOCK(net_data_mutex);
    size_t avail = cb_data_count(&network_cbuffer);
    if (avail < NETWORK_CBUFF_SLOT_SIZE)
    {
      OS_MUTEX_UNLOCK(net_data_mutex);
      return false;
    }
    size_t read = cb_read(&network_cbuffer, s_pub_slot, NETWORK_CBUFF_SLOT_SIZE);
    OS_MUTEX_UNLOCK(net_data_mutex);
    if (read != NETWORK_CBUFF_SLOT_SIZE)
    {
      LOG_WRN("Partial cbuffer read (%u bytes)", (unsigned) read);
      return false;
    }
    s_pub_slot_valid = true;
  }

  size_t payload_len = strnlen(s_pub_slot, NETWORK_CBUFF_SLOT_SIZE);
  if (adapter->publish(NET_CH_DATA, (const uint8_t *) s_pub_slot, payload_len) != STATUS_OK)
  {
    LOG_WRN("Data publish failed via %s — keeping slot", adapter->name);
    return false;
  }

  s_pub_slot_valid = false;
  return true;
}

static void sys_network_flush_cbuffer_to_sd(void)
{
  OS_MUTEX_LOCK(net_data_mutex);
  size_t available = cb_data_count(&network_cbuffer);
  OS_MUTEX_UNLOCK(net_data_mutex);

  if (available < NETWORK_CBUFF_SLOT_SIZE)
    return;

  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    LOG_ERR("SD not mounted - cannot flush");
    return;
  }

  if (sys_network_prepare_sd_card() != STATUS_OK)
    return;

  char log_path[MAX_OFFLINE_TRIP_PATH_LEN];
  if (s_active_trip_id != 0)
  {
    snprintf(log_path, sizeof(log_path), "%s%lu.log", SD_TRIP_PREFIX, s_active_trip_id);
  }
  else
  {
    snprintf(log_path, sizeof(log_path), "%s/trip_0.log", SD_OFFLINE_DIR);
  }

  bsp_sdcard_file_t fh;
  bsp_sdcard_mode_t mode =
    (bsp_sdcard_file_exists(log_path) == STATUS_OK) ? BSP_SDCARD_MODE_APPEND : BSP_SDCARD_MODE_WRITE;

  if (bsp_sdcard_open(log_path, mode, &fh) != STATUS_OK)
  {
    LOG_ERR("Cannot open offline log %s", log_path);
    return;
  }

  size_t flushed = 0;
  while (1)
  {
    char slot[NETWORK_CBUFF_SLOT_SIZE];
    OS_MUTEX_LOCK(net_data_mutex);
    size_t avail = cb_data_count(&network_cbuffer);
    if (avail < NETWORK_CBUFF_SLOT_SIZE)
    {
      OS_MUTEX_UNLOCK(net_data_mutex);
      break;
    }
    size_t read = cb_read(&network_cbuffer, slot, NETWORK_CBUFF_SLOT_SIZE);
    OS_MUTEX_UNLOCK(net_data_mutex);
    if (read != NETWORK_CBUFF_SLOT_SIZE)
      break;

    size_t json_len    = strnlen(slot, NETWORK_CBUFF_SLOT_SIZE);
    slot[json_len]     = '\n';
    slot[json_len + 1] = '\0';
    size_t wlen        = 0;
    bsp_sdcard_write(&fh, (const uint8_t *) slot, json_len + 1, &wlen);
    flushed++;
  }

  bsp_sdcard_close(&fh);
  if (flushed > 0)
  {
    LOG_INF("Flushed %u records to SD (%s)", (unsigned) flushed, log_path);
    s_is_data_sd_pending = sys_network_check_sd_pending();
  }
}

static status_function_t sys_network_push_sd_to_adapter(net_adapter_t *adapter)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK || s_current_upload_trip[0] == '\0')
  {
    s_is_data_sd_pending = false;
    return STATUS_BUSY;
  }

  char  log_path[128];
  char  ack_path[128];
  char *dot;
  snprintf(log_path, sizeof(log_path), "%s/%s", SD_OFFLINE_DIR, s_current_upload_trip);
  snprintf(ack_path, sizeof(ack_path), "%s", log_path);
  dot = strrchr(ack_path, '.');
  if (dot != NULL)
    strcpy(dot, "_ack.txt");

  bsp_sdcard_file_t fh;
  if (bsp_sdcard_open(log_path, BSP_SDCARD_MODE_READ, &fh) != STATUS_OK)
  {
    s_is_data_sd_pending     = false;
    s_current_upload_trip[0] = '\0';
    return STATUS_BUSY;
  }

  size_t total = fh.file.size();
  if (s_network_sd_offset >= total)
  {
    bsp_sdcard_close(&fh);
    bsp_sdcard_delete(log_path);
    bsp_sdcard_delete(ack_path);
    s_current_upload_trip[0] = '\0';
    s_is_data_sd_pending     = sys_network_check_sd_pending();
    LOG_INF("Offline log %s fully drained", log_path);
    return STATUS_OK;
  }

  if (bsp_sdcard_seek(&fh, s_network_sd_offset) != STATUS_OK)
  {
    bsp_sdcard_close(&fh);
    return STATUS_BUSY;
  }

  size_t  line_len = 0;
  uint8_t ch_byte;
  size_t  rd = 0;
  while (line_len < sizeof(s_sd_line) - 1)
  {
    if (bsp_sdcard_read(&fh, &ch_byte, 1, &rd) != STATUS_OK || rd == 0)
      break;
    if (ch_byte == '\n')
      break;
    s_sd_line[line_len++] = (char) ch_byte;
  }
  bsp_sdcard_close(&fh);

  if (line_len == 0)
  {
    s_network_sd_offset++;
    return STATUS_BUSY;
  }
  s_sd_line[line_len] = '\0';

  if (adapter->publish(NET_CH_DATA, (const uint8_t *) s_sd_line, line_len) != STATUS_OK)
  {
    LOG_WRN("SD drain: publish failed via %s", adapter->name);
    return STATUS_ERROR;
  }

  s_network_sd_offset += line_len + 1;

  bsp_sdcard_file_t ack_fh;
  if (bsp_sdcard_open(ack_path, BSP_SDCARD_MODE_WRITE, &ack_fh) == STATUS_OK)
  {
    char ack_buf[24];
    snprintf(ack_buf, sizeof(ack_buf), "%u", (unsigned) s_network_sd_offset);
    bsp_sdcard_write(&ack_fh, (const uint8_t *) ack_buf, strlen(ack_buf), NULL);
    bsp_sdcard_close(&ack_fh);
  }

  if (s_network_sd_offset >= total)
  {
    bsp_sdcard_delete(log_path);
    bsp_sdcard_delete(ack_path);
    s_current_upload_trip[0] = '\0';
    s_is_data_sd_pending     = sys_network_check_sd_pending();
    LOG_INF("Offline log %s fully drained", log_path);
  }
  return STATUS_OK;
}

static bool sys_network_need_push_sd(void)
{
  if (network_cbuffer.size <= 1)
    return false;

  OS_MUTEX_LOCK(net_data_mutex);
  size_t available = cb_data_count(&network_cbuffer);
  OS_MUTEX_UNLOCK(net_data_mutex);

  size_t total_bytes  = network_cbuffer.size - 1;
  size_t used_percent = (available * 100) / total_bytes;
  return (used_percent >= NETWORK_CBUFF_FLUSH_THRESH);
}

static status_function_t sys_network_prepare_sd_card(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
    return STATUS_ERROR;

  if (bsp_sdcard_dir_exists(SD_OFFLINE_DIR) != STATUS_OK)
  {
    if (bsp_sdcard_mkdir(SD_OFFLINE_DIR) != STATUS_OK)
    {
      LOG_ERR("Cannot create offline dir: %s", SD_OFFLINE_DIR);
      return STATUS_ERROR;
    }
  }

  return STATUS_OK;
}

static bool sys_network_check_sd_pending(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
    return false;
  if (sys_network_prepare_sd_card() != STATUS_OK)
    return false;

  char  ack_path[MAX_OFFLINE_TRIP_PATH_LEN];
  char *dot;

  // Active trip mode: only look for this trip's own log, never touch old backlog.
  if (s_active_trip_id != 0)
  {
    char active_path[MAX_OFFLINE_TRIP_PATH_LEN];
    snprintf(s_current_upload_trip, sizeof(s_current_upload_trip), "trip_%lu.log", s_active_trip_id);
    snprintf(active_path, sizeof(active_path), "%s/%s", SD_OFFLINE_DIR, s_current_upload_trip);
    if (bsp_sdcard_file_exists(active_path) != STATUS_OK)
    {
      s_current_upload_trip[0] = '\0';
      return false;
    }
    s_network_sd_offset = 0;
    snprintf(ack_path, sizeof(ack_path), "%s/trip_%lu_ack.txt", SD_OFFLINE_DIR, s_active_trip_id);
    bsp_sdcard_file_t ack_file_handle;
    if (bsp_sdcard_open(ack_path, BSP_SDCARD_MODE_READ, &ack_file_handle) == STATUS_OK)
    {
      char   ack_buf[24] = { 0 };
      size_t read_len    = 0;
      bsp_sdcard_read(&ack_file_handle, (uint8_t *) ack_buf, sizeof(ack_buf) - 1, &read_len);
      bsp_sdcard_close(&ack_file_handle);
      if (read_len > 0)
      {
        unsigned long val = 0;
        sscanf(ack_buf, "%lu", &val);
        s_network_sd_offset = (size_t) val;
      }
    }
    return true;
  }

  // No active trip: scan for any remaining backlog.
  bsp_sdcard_dir_t dir;
  if (bsp_sdcard_dir_open(SD_OFFLINE_DIR, &dir) != STATUS_OK)
    return false;

  char file_name[64];
  while (bsp_sdcard_dir_read_next(&dir, file_name, sizeof(file_name)) == STATUS_OK)
  {
    if (strstr(file_name, ".log") == NULL)
      continue;

    snprintf(s_current_upload_trip, sizeof(s_current_upload_trip), "%s", file_name);
    bsp_sdcard_dir_close(&dir);

    s_network_sd_offset = 0;
    snprintf(ack_path, sizeof(ack_path), "%s/%s", SD_OFFLINE_DIR, file_name);
    dot = strrchr(ack_path, '.');
    if (dot != NULL)
    {
      strcpy(dot, "_ack.txt");
      bsp_sdcard_file_t ack_file_handle;
      if (bsp_sdcard_open(ack_path, BSP_SDCARD_MODE_READ, &ack_file_handle) == STATUS_OK)
      {
        char   ack_buf[24] = { 0 };
        size_t read_len    = 0;
        bsp_sdcard_read(&ack_file_handle, (uint8_t *) ack_buf, sizeof(ack_buf) - 1, &read_len);
        bsp_sdcard_close(&ack_file_handle);
        if (read_len > 0)
        {
          unsigned long val = 0;
          sscanf(ack_buf, "%lu", &val);
          s_network_sd_offset = (size_t) val;
        }
      }
    }
    return true;
  }

  bsp_sdcard_dir_close(&dir);
  s_current_upload_trip[0] = '\0';
  return false;
}

void sys_network_reset_offline_data(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
    return;

  bsp_sdcard_dir_t dir;
  if (bsp_sdcard_dir_open(SD_OFFLINE_DIR, &dir) == STATUS_OK)
  {
    char file_name[MAX_OFFLINE_TRIP_NAME_LEN];
    char file_path[MAX_OFFLINE_TRIP_PATH_LEN];
    while (bsp_sdcard_dir_read_next(&dir, file_name, sizeof(file_name)) == STATUS_OK)
    {
      snprintf(file_path, sizeof(file_path), "%s/%s", SD_OFFLINE_DIR, file_name);
      bsp_sdcard_delete(file_path);
    }
    bsp_sdcard_dir_close(&dir);
    bsp_sdcard_delete(SD_OFFLINE_DIR);
  }

  s_is_data_sd_pending     = false;
  s_active_trip_id         = 0;
  s_current_upload_trip[0] = '\0';
  s_network_sd_offset      = 0;
}

static void sys_network_write_trip_info(const char *meta_path, uint32_t trip_id, trip_state_t state)
{
  bsp_sdcard_file_t trip_info_file;
  if (bsp_sdcard_open(meta_path, BSP_SDCARD_MODE_WRITE, &trip_info_file) != STATUS_OK)
    return;
  char meta_buf[32];
  snprintf(meta_buf, sizeof(meta_buf), "%lu,%d", (unsigned long) trip_id, (int) state);
  bsp_sdcard_write(&trip_info_file, (const uint8_t *) meta_buf, strlen(meta_buf), NULL);
  bsp_sdcard_close(&trip_info_file);
}

/* End of file -------------------------------------------------------- */
