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
#include "os_lib.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
#define SYS_LOG_MKDIR_RETRIES (3)
#define SYS_LOG_DEBUG_MODE    (0)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#if SYS_LOG_DEBUG_MODE
#define DEBUG_LOG(msg)        Serial.println(msg)
#else
#define DEBUG_LOG(msg)
#endif

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
#if LOG_SDCARD_ENABLE

static uint8_t   ram_log_buff[SYS_LOG_BUFFER_SIZE];
static cbuffer_t cbuff_ram_log;
static size_t    last_flush_tick = 0;

/* Private function prototypes ---------------------------------------- */
static void              sys_log_buffer_write(const char *data, size_t len);
static uint8_t           sys_log_get_buffer_usage(void);
static status_function_t sys_log_flush(void);

/* Function definitions ----------------------------------------------- */

void sys_log_init(void)
{
  cb_init(&cbuff_ram_log, ram_log_buff, SYS_LOG_BUFFER_SIZE);
  cb_clear(&cbuff_ram_log);
  last_flush_tick = OS_GET_TICK();

  if (bsp_sdcard_is_mounted() == STATUS_OK)
  {
    for (int i = 0; i < SYS_LOG_MKDIR_RETRIES; i++)
    {
      if (bsp_sdcard_mkdir("/logs") == STATUS_OK)
      {
        DEBUG_LOG("SD card log directory initialized");
        break;
      }
      else
      {
        DEBUG_LOG("Failed to create log directory on SD card, retrying...");
        OS_DELAY_MS(100);
      }
    }
  }

  // Register handler with log_service
  log_service_register_handler(sys_log_buffer_write);
}

void sys_log_process(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    DEBUG_LOG("SD card not mounted, skipping log flush");
    return;
  }

  bool should_flush = false;

  // Check periodic interval
  if ((OS_GET_TICK() - last_flush_tick) >= SYS_LOG_FLUSH_INTERVAL_MS && cb_data_count(&cbuff_ram_log) > 0)
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
    DEBUG_LOG("Flushing logs to SD card...");
    sys_log_flush();
  }
}

uint8_t sys_log_get_buffer_usage(void)
{
  return (uint8_t) ((cb_data_count(&cbuff_ram_log) * 100) / SYS_LOG_BUFFER_SIZE);
}

void sys_log_deinit(void)
{
  // Flush remaining logs
  sys_log_flush();

  // Unregister handler
  log_service_register_handler(nullptr);
}

/* Private definitions ----------------------------------------------- */

static status_function_t sys_log_flush(void)
{
  size_t data_len = cb_data_count(&cbuff_ram_log);
  if (data_len == 0 || bsp_sdcard_is_mounted() != STATUS_OK)
  {
    DEBUG_LOG("No data to flush or SD card not mounted");
    return STATUS_ERROR;
  }

  // Allocate flush buffer
  char *flush_buf = (char *) malloc(data_len + 1);
  if (flush_buf == nullptr)
  {
    DEBUG_LOG("Failed to allocate flush buffer");
    return STATUS_ERROR;
  }

  // Read from cbuffer
  size_t read_len     = cb_read(&cbuff_ram_log, flush_buf, data_len);
  flush_buf[read_len] = '\0';

  // Write to SD card
  bsp_sdcard_file_t log_file;
  status_function_t ret = bsp_sdcard_open(SYS_LOG_FILE_PATH, BSP_SDCARD_MODE_APPEND, &log_file);
  if (ret == STATUS_OK)
  {
    DEBUG_LOG("Flushing logs to SD card...");
    bsp_sdcard_write(&log_file, (const uint8_t *) flush_buf, read_len, nullptr);
    bsp_sdcard_close(&log_file);
  }

  free(flush_buf);
  last_flush_tick = OS_GET_TICK();

  return ret;
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
