
/**
 * @file       sys_cmd.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System command processing implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_cmd.h"

#include "bsp_device.h"
#include "bsp_rtc.h"
#include "bsp_sim.h"
#include "log_service.h"
#include "sys_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_cmd, LOG_LEVEL_DBG)

#define CMD_TIME_VALID_FORMAT_LEN (17)  // "HH:MM:SS-DD/MM/YYYY"

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  char command[32];
  void (*handler)(void);
} sys_command_t;

/* Private function prototypes ---------------------------------------- */
static status_function_t sys_cmd_parse_and_execute(const char *input);

static void sys_cmd_lock_device_handler(void);
static void sys_cmd_unlock_device_handler(void);
static void sys_cmd_set_time_handler(void);
static void sys_cmd_set_device_id_handler(void);
static void sys_cmd_reboot_handler(void);

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
OS_SEM_DEFINE_GLOBAL(sys_cmd_req_sem)
char g_cmd_input_buffer[CMD_INPUT_MAX_LEN];

/* Private variables -------------------------------------------------- */
// clang-format off
#define INFO(name, handler) { name, handler }
static sys_command_t CMD_INFO[CMD_MAX] = {
  INFO("LOCK",      sys_cmd_lock_device_handler),
  INFO("UNLOCK",    sys_cmd_unlock_device_handler),
  INFO("SET_TIME",  sys_cmd_set_time_handler),
  INFO("SET_DEVICE",  sys_cmd_set_device_id_handler),
  INFO("RESET",    sys_cmd_reboot_handler)
};
#undef INFO
// clang-format on

/* Function definitions ----------------------------------------------- */
void sys_cmd_process()
{
  OS_SEM_TAKE(sys_cmd_req_sem, OS_MAX_DELAY);
  char cmd_buffer[CMD_INPUT_MAX_LEN];
  strncpy(cmd_buffer, g_cmd_input_buffer, CMD_INPUT_MAX_LEN);
  if (sys_cmd_parse_and_execute(cmd_buffer) == STATUS_OK)
  {
    LOG_DBG("Command executed successfully");
  }
  else
  {
    LOG_WRN("Failed to execute command: %s", cmd_buffer);
  }
}

/* Private definitions ----------------------------------------------- */
static status_function_t sys_cmd_parse_and_execute(const char *input)
{
  LOG_DBG("Processing command: %s", input);
  for (int i = 0; i < CMD_MAX; ++i)
  {
    if (strncmp(input, CMD_INFO[i].command, strlen(CMD_INFO[i].command)) == 0)
    {
      if (CMD_INFO[i].handler)
      {
        CMD_INFO[i].handler();
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

static void sys_cmd_lock_device_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_LOCKED);
}

static void sys_cmd_unlock_device_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_UNLOCKED);
}

static void sys_cmd_set_time_handler(void)
{
  timeline_t  req_time = { 0 };
  const char *cmd      = g_cmd_input_buffer;
  const char *time_str = strchr(cmd, '=');

  if (!time_str || strlen(time_str + 1) < CMD_TIME_VALID_FORMAT_LEN)
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

static void sys_cmd_set_device_id_handler(void)
{
  const char *cmd = g_cmd_input_buffer;
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
}

static void sys_cmd_reboot_handler(void)
{
  sys_manager_write_event(SYS_MANAGER_EVT_REBOOT);
}

/* End of file -------------------------------------------------------- */
