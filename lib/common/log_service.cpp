/**
 * @file       log_service.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Basic logging service implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "log_service.h"

#include "bsp_rtc.h"
#include "bsp_sdcard.h"

#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
#define LOG_LINE_MAX_LEN (256)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static log_handler_t external_handler = nullptr;  // Function pointer for ex log handler (e.g. SD card, flash)

static const char *level_str[] = { "-", "ERR", "WRN", "INF", "DBG" };

/* Private function prototypes ---------------------------------------- */
static void log_format_timestamp(char *buf, size_t len);

/* Function definitions ----------------------------------------------- */
void log_service_init(void)
{
#if LOG_USB_ENABLE
  if (!Serial)
  {
    Serial.begin(LOG_SERIAL_BAUD);
    delay(1000);  // Wait for Serial to initialize
  }
  Serial.println("[LOG] Log service initialized");
#endif
}

void log_service_print(log_level_t level, const char *tag, const char *fmt, ...)
{
  if ((level == LOG_LEVEL_NONE) || (level >= LOG_LEVEL_MAX))
  {
    return;
  }

  char line[LOG_LINE_MAX_LEN];
  char msg[LOG_LINE_MAX_LEN - 40];  // 40 chars reserved for timestamp, level, tag
  char timestamp[16];

  // Format timestamp
  log_format_timestamp(timestamp, sizeof(timestamp));

  // Format
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  // Build log line
  size_t len = snprintf(line, sizeof(line), "[%s][%s][%s] %s\n", timestamp, level_str[level], tag ? tag : "?", msg);

  // Output to USB
#if LOG_USB_ENABLE
  Serial.println(line);
#endif

  // Call external handler (for SD card)
#if LOG_SDCARD_ENABLE
  if ((external_handler != nullptr) && (bsp_sdcard_is_mounted() == STATUS_OK))
  {
    external_handler(line, len);
  }
#endif
}

void log_service_register_handler(log_handler_t handler)
{
  external_handler = handler;
}

/* Private definitions ----------------------------------------------- */

static void log_format_timestamp(char *buf, size_t len)
{
  // Implement RTC timestamp
  timeline_t current_time;
  bsp_rtc_get(&current_time);
  snprintf(buf, len, "%d/%d/%d][%02u:%02u:%02u]", current_time.date, current_time.month, current_time.year,
           current_time.hour, current_time.minute, current_time.second);
}

/* End of file -------------------------------------------------------- */
