/**
 * @file       sys_log.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      System log to SD card implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_log.h"

#include "bsp_sdcard.h"
#include "cbuffer.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
#if LOG_SDCARD_ENABLE

static bool      is_initialized = false;
static uint8_t   ram_log_buff[SYS_LOG_BUFFER_SIZE];
static cbuffer_t cbuff_ram_log;
static size_t    last_flush_tick = 0;

#endif

/* Private function prototypes ---------------------------------------- */
#if LOG_SDCARD_ENABLE
static void sys_log_handler(log_level_t level, const char *tag, const char *message, size_t len);
static void sys_log_buffer_write(const char *data, size_t len);
#endif

/* Function definitions ----------------------------------------------- */

void sys_log_init(void)
{
#if LOG_SDCARD_ENABLE
  cb_init(&cbuff_ram_log, ram_log_buff, SYS_LOG_BUFFER_SIZE);
  cb_clear(&cbuff_ram_log);
  last_flush_tick = millis();
  is_initialized  = true;

  // Register handler with log_service
  log_service_register_handler(sys_log_buffer_write);
#else
  // Do nothing
#endif
}

status_function_t sys_log_flush(void)
{
#if LOG_SDCARD_ENABLE
  size_t data_len = cb_data_count(&cbuff_ram_log);
  if (!is_initialized || data_len == 0 || bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return STATUS_ERROR;
  }

  // Allocate flush buffer
  char *flush_buf = (char *) malloc(data_len + 1);
  if (flush_buf == nullptr)
  {
    return STATUS_ERROR;
  }

  // Read from cbuffer
  size_t read_len     = cb_read(&cbuff_ram_log, flush_buf, data_len);
  flush_buf[read_len] = '\0';

  // Write to SD card using new API
  bsp_sdcard_file_t log_file;
  status_function_t ret = bsp_sdcard_open(SYS_LOG_FILE_PATH, BSP_SDCARD_MODE_APPEND, &log_file);
  if (ret == STATUS_OK)
  {
    bsp_sdcard_write(&log_file, (const uint8_t *) flush_buf, read_len, nullptr);
    bsp_sdcard_close(&log_file);
  }

  free(flush_buf);
  last_flush_tick = millis();

  return ret;
#else
  return STATUS_OK;
#endif
}

void sys_log_process(void)
{
#if LOG_SDCARD_ENABLE
  if (!is_initialized || bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return;
  }

  bool should_flush = false;

  // Check periodic interval
  if ((millis() - last_flush_tick) >= SYS_LOG_FLUSH_INTERVAL_MS && cb_data_count(&cbuff_ram_log) > 0)
  {
    should_flush = true;
  }

  // Check buffer threshold
  if (sys_log_get_buffer_usage() >= SYS_LOG_BUFFER_THRESHOLD)
  {
    should_flush = true;
  }

  if (should_flush)
  {
    sys_log_flush();
  }
#endif
}

uint8_t sys_log_get_buffer_usage(void)
{
#if LOG_SDCARD_ENABLE
  return (uint8_t) ((cb_data_count(&cbuff_ram_log) * 100) / SYS_LOG_BUFFER_SIZE);
#else
  return 0;
#endif
}

/* Private definitions ----------------------------------------------- */

#if LOG_SDCARD_ENABLE

static void sys_log_handler(log_level_t level, const char *tag, const char *message, size_t len)
{
  if (!is_initialized)
  {
    return;
  }
  sys_log_buffer_write(message, len);
}

static void sys_log_buffer_write(const char *data, size_t len)
{
  if (data == nullptr || len == 0)
  {
    return;
  }

  // Check space, flush if needed
  if (len > cb_space_count(&cbuff_ram_log) && bsp_sdcard_is_mounted() == STATUS_OK)
  {
    sys_log_flush();
  }

  // Write to cbuffer (cbuffer handles overflow internally)
  cb_write(&cbuff_ram_log, (void *) data, len);
}

#endif

/* End of file -------------------------------------------------------- */
