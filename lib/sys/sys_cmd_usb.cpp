/**
 * @file       sys_cmd_usb.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System command through USB processing implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_cmd_usb.h"

#include "bsp_device.h"
#include "bsp_rtc.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_fusion_log.h"
#include "sys_manager.h"
#include "sys_network_adapter_lte.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_cmd_usb, LOG_LEVEL_SYS_CMD_USB);

#define CMD_USB_MAX_LEN               (128)
#define CMD_USB_TIME_VALID_FORMAT_LEN (17)  // "HH:MM:SS-DD/MM/YYYY"

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  char command[32];
  void (*handler)(void);
} sys_cmd_usb_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void              sys_cmd_usb_callback_handler(bsp_usb_event_t event, void *arg);
static status_function_t sys_cmd_usb_parse_and_execute(const char *input);
static void              sys_cmd_usb_reset_handler(void);
static void              sys_cmd_usb_unlock_handler(void);
static void              sys_cmd_usb_lock_handler(void);
static void              sys_cmd_usb_set_time_handler(void);
static void              sys_cmd_usb_set_device_id_handler(void);
static void              sys_cmd_usb_reset_offline_data_handler(void);
static void              sys_cmd_usb_set_danger_noti_handler(void);
#if (DEVICE_NETWORK_TOTAL_KM_ENABLED)
static void sys_cmd_usb_reset_distance_handler(void);
#endif
static void sys_cmd_usb_sleep_handler(void);
#if (DEVICE_FUSION_DEBUG_LOG_ENABLED)
static void sys_cmd_usb_flush_fusion_log_handler(void);
#endif
static void sys_cmd_usb_device_info_handler(void);
static void sys_cmd_usb_check_lte_band_handler(void);
static void sys_cmd_usb_modem_reset_handler(void);

/* Private variables -------------------------------------------------- */
// clang-format off
#define INFO(name, handler) { name, handler }
static sys_cmd_usb_t CMD_USB_INFO[SYS_CMD_USB_CMD_MAX] = {
  INFO("RESET",              sys_cmd_usb_reset_handler),
  INFO("UNLOCK",             sys_cmd_usb_unlock_handler),
  INFO("LOCK",               sys_cmd_usb_lock_handler),
  INFO("SET_TIME",           sys_cmd_usb_set_time_handler),
  INFO("SET_DEVICE",         sys_cmd_usb_set_device_id_handler),
  INFO("CLEAR_OFFLINE_DATA", sys_cmd_usb_reset_offline_data_handler),
  #if (DEVICE_NETWORK_TOTAL_KM_ENABLED)
  INFO("CLEAR_TOTAL_DISTANCE", sys_cmd_usb_reset_distance_handler),
  #endif
  INFO("SET_DANGER_NOTI",   sys_cmd_usb_set_danger_noti_handler),
  INFO("SLEEP",             sys_cmd_usb_sleep_handler),
  #if (DEVICE_FUSION_DEBUG_LOG_ENABLED)
  INFO("FLUSH_FUSION_LOG",  sys_cmd_usb_flush_fusion_log_handler),
  #endif
  INFO("DEVICE_INFO",       sys_cmd_usb_device_info_handler),
  INFO("CHECK_LTE_BAND",    sys_cmd_usb_check_lte_band_handler),
  INFO("MODEM_RESET",       sys_cmd_usb_modem_reset_handler)
};
#undef INFO
// clang-format on

OS_SEM_DEFINE_STATIC(sys_cmd_usb_req_sem);
static char cmd_usb_buffer[CMD_USB_MAX_LEN];

/* Function definitions ----------------------------------------------- */
void sys_cmd_usb_init(void)
{
  bsp_usb_init(sys_cmd_usb_callback_handler);
  OS_SEM_CREATE(sys_cmd_usb_req_sem);
}

void sys_cmd_usb_process(void)
{
  OS_SEM_TAKE(sys_cmd_usb_req_sem, OS_MAX_DELAY);
  bsp_usb_read((uint8_t *) cmd_usb_buffer, CMD_USB_MAX_LEN);
  if (sys_cmd_usb_parse_and_execute(cmd_usb_buffer) == STATUS_OK)
  {
    LOG_DBG("Command executed successfully");
  }
  else
  {
    LOG_WRN("Failed to execute command: %s", cmd_usb_buffer);
  }
}

/* Private definitions ----------------------------------------------- */
static void sys_cmd_usb_callback_handler(bsp_usb_event_t event, void *arg)
{
  switch (event)
  {
  case BSP_USB_EVENT_DATA_RX:
  {
    OS_SEM_GIVE(sys_cmd_usb_req_sem);
    break;
  }
  case BSP_USB_EVENT_CONNECTED:
  case BSP_USB_EVENT_DISCONNECTED:
  {
    // Do nothing for now
    break;
  }

  default: break;
  }
}

static status_function_t sys_cmd_usb_parse_and_execute(const char *input)
{
  LOG_DBG("Processing command: %s", input);
  for (int i = 0; i < SYS_CMD_USB_CMD_MAX; ++i)
  {
    if (strncmp(input, CMD_USB_INFO[i].command, strlen(CMD_USB_INFO[i].command)) == 0)
    {
      if (CMD_USB_INFO[i].handler)
      {
        CMD_USB_INFO[i].handler();
        return STATUS_OK;
      }
      else
      {
        // Do nothing
        return STATUS_OK;
      }
    }
  }
  LOG_WRN("Unknown command: %s", input);
  return STATUS_ERROR;
}

static void sys_cmd_usb_reset_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_REBOOT);
}

static void sys_cmd_usb_unlock_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_UNLOCKED);
}

static void sys_cmd_usb_lock_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_LOCKED);
}

static void sys_cmd_usb_set_time_handler(void)
{
  timeline_t  req_time = { 0 };
  const char *cmd      = cmd_usb_buffer;
  const char *time_str = strchr(cmd, '=');

  if (!time_str || strlen(time_str + 1) < CMD_USB_TIME_VALID_FORMAT_LEN)
  {
    LOG_WRN("SET_TIME: Invalid format");
    return;
  }

  uint8_t hour, min, sec, day, mon, year;
  char    daystr[8] = { 0 };
  uint8_t num       = sscanf(time_str + 1, "%d:%d:%d-%d/%d/%d-%7s", &hour, &min, &sec, &day, &mon, &year, daystr);

  if (num >= 6)
  {
    req_time.hour   = hour;
    req_time.minute = min;
    req_time.second = sec;
    req_time.date   = day;
    req_time.month  = mon;
    req_time.year   = year;

    if (num == 7)
    {
      if (strcmp(daystr, "SUN") == 0)
      {
        req_time.day = SUNDAY;
      }
      else if (strcmp(daystr, "MON") == 0)
      {
        req_time.day = MONDAY;
      }
      else if (strcmp(daystr, "TUES") == 0)
      {
        req_time.day = TUESDAY;
      }
      else if (strcmp(daystr, "WED") == 0)
      {
        req_time.day = WEDNESDAY;
      }
      else if (strcmp(daystr, "THU") == 0)
      {
        req_time.day = THURSDAY;
      }
      else if (strcmp(daystr, "FRI") == 0)
      {
        req_time.day = FRIDAY;
      }
      else if (strcmp(daystr, "SAT") == 0)
      {
        req_time.day = SATURDAY;
      }
      else
      {
        req_time.day = SUNDAY;
      }
    }
    else
    {
      LOG_WRN("No day of week provided, defaulting to SUNDAY");
    }
    LOG_DBG("SET_TIME parsed: %02d:%02d:%02d %02d/%02d/%04d day=%d", req_time.hour, req_time.minute, req_time.second,
            req_time.date, req_time.month, req_time.year, req_time.day);
    bsp_rtc_set(&req_time);
  }
  else
  {
    LOG_WRN("Invalid format SET_TIME");
  }
}

static void sys_cmd_usb_set_device_id_handler(void)
{
  const char *cmd = cmd_usb_buffer;
  const char *eq  = strchr(cmd, '=');
  char        name_buffer[DEVICE_NAME_MAX_LEN];

  // Validate '=' exists and has content after it
  if (!eq || strlen(eq + 1) == 0)
  {
    LOG_WRN("SET_DEVICE: Invalid format, expected SET_DEVICE=<id>,<serial>");
    return;
  }

  // Parse device_id — token before ','
  const char *id_start = eq + 1;
  const char *comma    = strchr(id_start, ',');

  if (!comma || comma == id_start)
  {
    LOG_WRN("SET_DEVICE: Missing comma or empty device_id");
    return;
  }

  char   id_str[8] = { 0 };
  size_t id_len    = comma - id_start;

  if (id_len >= sizeof(id_str))
  {
    LOG_WRN("SET_DEVICE: device_id too long");
    return;
  }

  strncpy(id_str, id_start, id_len);
  id_str[id_len] = '\0';

  const char *serial_start = comma + 1;

  if (strlen(serial_start) == 0)
  {
    LOG_WRN("SET_DEVICE: Empty serial number");
    return;
  }

  if (strlen(serial_start) >= sizeof(g_device_info.nvs_info.serial_number))
  {
    LOG_WRN("SET_DEVICE: Serial number too long");
    return;
  }

  g_device_info.nvs_info.device_id = (uint8_t) atoi(id_str);
  strncpy(g_device_info.nvs_info.serial_number, serial_start, sizeof(g_device_info.nvs_info.serial_number) - 1);
  g_device_info.nvs_info.serial_number[sizeof(g_device_info.nvs_info.serial_number) - 1] = '\0';

  snprintf(name_buffer, sizeof(name_buffer), "haq-trk-%s", id_str);
  strncpy(g_device_info.device_name, name_buffer, sizeof(g_device_info.device_name) - 1);
  g_device_info.device_name[sizeof(g_device_info.device_name) - 1] = '\0';

  snprintf(g_device_info.mqtt_cmd_topic, sizeof(g_device_info.mqtt_cmd_topic), "%s/cmd", g_device_info.device_name);
  snprintf(g_device_info.mqtt_data_topic, sizeof(g_device_info.mqtt_data_topic), "%s/data", g_device_info.device_name);

  bsp_device_flash_write(&g_device_info.nvs_info);
  sys_manager_write_event(SYS_MANAGER_EVT_CHANGE_CMD_TOPIC);

  LOG_DBG("SET_DEVICE: id=%s serial=%s name=%s cmd=%s data=%s", id_str, g_device_info.nvs_info.serial_number,
          g_device_info.device_name, g_device_info.mqtt_cmd_topic, g_device_info.mqtt_data_topic);

  sys_manager_write_event(SYS_MANAGER_EVT_REBOOT);
}

#if (DEVICE_NETWORK_TOTAL_KM_ENABLED)
static void sys_cmd_usb_reset_distance_handler(void)
{
  g_device_info.nvs_info.total_km = 0.0f;
  bsp_device_flash_write(&g_device_info.nvs_info);
  LOG_DBG("Distance reset to 0 km");
}
#endif

static void sys_cmd_usb_reset_offline_data_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_RESET_OFFLINE_DATA);
}

static void sys_cmd_usb_set_danger_noti_handler(void)
{
  const char *cmd = cmd_usb_buffer;
  const char *eq  = strchr(cmd, '=');
  const char *val = (eq && strlen(eq + 1) > 0) ? eq + 1 : NULL;

  if (val)
  {
    if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0)
    {
      g_device_info.danger_noti_enabled = true;
      LOG_DBG("Danger notification enabled");
      sys_manager_write_event(SYS_MANAGER_EVT_APPLY_DANGER_NOTI);
    }
    else if (strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)
    {
      g_device_info.danger_noti_enabled = false;
      g_device_info.danger_level        = DEVICE_DANGER_LEVEL_LOW;
      LOG_DBG("Danger notification disabled");
      sys_manager_write_event(SYS_MANAGER_EVT_APPLY_DANGER_NOTI);
    }
    else
    {
      LOG_WRN("Invalid value for SET_DANGER_NOTI: %s", val);
    }
  }
  else
  {
    LOG_WRN("No value provided for SET_DANGER_NOTI, expected SET_DANGER_NOTI=<0|1|true|false>");
  }
}

static void sys_cmd_usb_sleep_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_SHUTDOWN);
}

#if (DEVICE_FUSION_DEBUG_LOG_ENABLED)
static void sys_cmd_usb_flush_fusion_log_handler(void)
{
  sys_fusion_log_flush();
}
#endif

static const char *sys_cmd_usb_state_str(device_state_t state)
{
  switch (state)
  {
  case DEVICE_STATE_IDLE: return "IDLE";
  case DEVICE_STATE_LOCKED: return "LOCKED";
  case DEVICE_STATE_ACTIVE: return "ACTIVE";
  case DEVICE_STATE_ERROR: return "ERROR";
  case DEVICE_STATE_PAUSED: return "PAUSED";
  case DEVICE_STATE_NOTI: return "NOTI";
  case DEVICE_STATE_STOLEN: return "STOLEN";
  default: return "UNKNOWN";
  }
}

static void sys_cmd_usb_device_info_handler(void)
{
  LOG_INF("===== DEVICE INFO =====");
  LOG_INF("Name        : %s", g_device_info.device_name);
  LOG_INF("Version     : %s", g_device_info.device_version);
  LOG_INF("Device ID   : %u", g_device_info.nvs_info.device_id);
  LOG_INF("Serial      : %s", g_device_info.nvs_info.serial_number);
  LOG_INF("State       : %s (prev %s)", sys_cmd_usb_state_str(g_device_info.nvs_info.curr_state),
          sys_cmd_usb_state_str(g_device_info.nvs_info.prev_state));
  LOG_INF("Total km    : %.2f", g_device_info.nvs_info.total_km);
  LOG_INF("Err count   : %u", g_device_info.nvs_info.err_count);
  LOG_INF("Reset reason: %d", g_device_info.last_reset_reason);
  LOG_INF("Danger noti : %s (level %d)", g_device_info.danger_noti_enabled ? "ON" : "OFF", g_device_info.danger_level);
  LOG_INF("CMD topic   : %s", g_device_info.mqtt_cmd_topic);
  LOG_INF("Data topic  : %s", g_device_info.mqtt_data_topic);
  LOG_INF("Noti topic  : %s", g_device_info.mqtt_noti_topic);
  LOG_INF("Log SD path : %s", g_device_info.log_sd_path);
  LOG_INF("=======================");
}

static void sys_cmd_usb_check_lte_band_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_CHECK_LTE_BAND);
}

static void sys_cmd_usb_modem_reset_handler(void)
{
  LOG_INF("MODEM_RESET requested");
  sys_network_adapter_lte_request_reset();
}

/* End of file -------------------------------------------------------- */
