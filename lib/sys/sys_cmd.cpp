
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

#include "bsp_rtc.h"
#include "log_service.h"

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

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
OS_SEM_DEFINE_GLOBAL(sys_cmd_req_sem)
char sys_cmd_input_buffer[CMD_INPUT_MAX_LEN];

/* Private variables -------------------------------------------------- */
// clang-format off
#define INFO(name, handler) { name, handler }
static sys_command_t CMD_INFO[CMD_MAX] = {
  INFO("LOCK",      sys_cmd_lock_device_handler),
  INFO("UNLOCK",    sys_cmd_unlock_device_handler),
  INFO("SET_TIME",  sys_cmd_set_time_handler)
};
#undef INFO
// clang-format on

/* Command Handlers --------------------------------------------------- */
static void sys_cmd_lock_device_handler(void)
{
  // TODO: Add lock logic
}

static void sys_cmd_unlock_device_handler(void)
{
  // TODO: Add unlock logic
}

static void sys_cmd_set_time_handler(void)
{
  timeline_t  req_time = { 0 };
  const char *cmd      = sys_cmd_input_buffer;
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

/* Function definitions ----------------------------------------------- */
/* Private definitions ----------------------------------------------- */
static status_function_t sys_cmd_parse_and_execute(const char *input)
{
  for (int i = 0; i < CMD_MAX; ++i)
  {
    if (strcmp(input, CMD_INFO[i].command) == 0)
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

/* End of file -------------------------------------------------------- */
