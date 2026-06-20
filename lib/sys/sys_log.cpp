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

#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "cbuffer.h"
#include "os_lib.h"

#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
#define SYS_LOG_MKDIR_RETRIES  (3)
#define SYS_LOG_DEBUG_MODE     (0)
#define SYS_LOG_NAME_MAX       (32)  // longest log file name we handle (e.g. 31-12-2026.log)
#define SYS_LOG_PRUNE_BATCH    (16)  // files deleted per directory scan
#define SYS_LOG_PRUNE_MAX_PASS (64)  // bound scans per enforce call

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#if SYS_LOG_DEBUG_MODE
#define DEBUG_LOG(msg) Serial.println(msg)
#else
#define DEBUG_LOG(msg)
#endif

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
#if LOG_SDCARD_ENABLE

static uint8_t   ram_log_buff[SYS_LOG_BUFFER_SIZE];
static cbuffer_t cbuff_ram_log;
static size_t    last_flush_tick     = 0;
static size_t    last_cap_check_tick = 0;

/* Private function prototypes ---------------------------------------- */
static void              sys_log_buffer_write(const char *data, size_t len);
static uint8_t           sys_log_get_buffer_usage(void);
static status_function_t sys_log_flush(void);
static void              sys_log_enforce_capacity(void);
static void              sys_log_prune_old_days(void);
static long              sys_log_days_from_civil(int y, unsigned m, unsigned d);

/* Function definitions ----------------------------------------------- */

void sys_log_init(void)
{
  cb_init(&cbuff_ram_log, ram_log_buff, SYS_LOG_BUFFER_SIZE);
  cb_clear(&cbuff_ram_log);
  last_flush_tick     = OS_GET_TICK();
  last_cap_check_tick = OS_GET_TICK();

  if (bsp_sdcard_is_mounted() == STATUS_OK)
  {
    for (int i = 0; i < SYS_LOG_MKDIR_RETRIES; i++)
    {
      if (bsp_sdcard_mkdir(SD_LOG_DIR) == STATUS_OK)
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
    sys_log_enforce_capacity();  // trim oversized log dir at boot
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

  // Periodically enforce the log storage cap (scan is heavy, keep it infrequent)
  if ((OS_GET_TICK() - last_cap_check_tick) >= SYS_LOG_CAP_CHECK_INTERVAL_MS)
  {
    last_cap_check_tick = OS_GET_TICK();
    sys_log_enforce_capacity();
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
  status_function_t ret = bsp_sdcard_open(g_device_info.log_sd_path, BSP_SDCARD_MODE_APPEND, &log_file);
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

static long sys_log_days_from_civil(int y, unsigned m, unsigned d)
{
  y -= (m <= 2);
  const long     era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned) (y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long) doe - 719468;
}

static void sys_log_prune_old_days(void)
{
  timeline_t today;
  if (bsp_rtc_get(&today) != STATUS_OK)
  {
    return;
  }
  const long today_dn = sys_log_days_from_civil((int) today.year, today.month, today.date);

  const char *active = strrchr(g_device_info.log_sd_path, '/');
  active             = (active != nullptr) ? active + 1 : g_device_info.log_sd_path;

  for (int pass = 0; pass < SYS_LOG_PRUNE_MAX_PASS; pass++)
  {
    bsp_sdcard_dir_t dir;
    if (bsp_sdcard_dir_open(SD_LOG_DIR, &dir) != STATUS_OK)
    {
      return;
    }

    // Collect victims first, delete after closing the dir (don't mutate it while iterating)
    char victims[SYS_LOG_PRUNE_BATCH][SYS_LOG_NAME_MAX];
    int  victim_count = 0;
    bool more         = false;
    char name[SYS_LOG_NAME_MAX];
    while (bsp_sdcard_dir_read_next(&dir, name, sizeof(name)) == STATUS_OK)
    {
      if (strcmp(name, active) == 0)
      {
        continue;
      }
      int d, m, y;
      if (sscanf(name, "%d-%d-%d.log", &d, &m, &y) != 3)
      {
        continue;  // not a daily log, leave it alone
      }
      const long file_dn = sys_log_days_from_civil(y, (unsigned) m, (unsigned) d);
      if ((today_dn - file_dn) > SYS_LOG_KEEP_DAYS)
      {
        if (victim_count >= SYS_LOG_PRUNE_BATCH)
        {
          more = true;
          break;
        }
        strncpy(victims[victim_count], name, SYS_LOG_NAME_MAX - 1);
        victims[victim_count][SYS_LOG_NAME_MAX - 1] = '\0';
        victim_count++;
      }
    }
    bsp_sdcard_dir_close(&dir);

    for (int i = 0; i < victim_count; i++)
    {
      char del_path[DEVICE_LOG_SD_PATH_MAX_LEN];
      snprintf(del_path, sizeof(del_path), SD_LOG_DIR "/%s", victims[i]);
      bsp_sdcard_delete(del_path);
      DEBUG_LOG("Log cap: deleted old daily log");
    }

    if (!more)
    {
      break;
    }
  }
}

// When SD_LOG_DIR exceeds the usage threshold, prune logs down to the most recent days.
static void sys_log_enforce_capacity(void)
{
  if (bsp_sdcard_is_mounted() != STATUS_OK)
  {
    return;
  }

  const uint64_t hi_bytes = (SYS_LOG_DIR_MAX_BYTES / 100ULL) * SYS_LOG_DIR_USAGE_PERCENT;
  uint64_t       used     = 0;
  if (bsp_sdcard_dir_total_size(SD_LOG_DIR, &used) != STATUS_OK)
  {
    return;
  }
  if (used < hi_bytes)
  {
    return;  // within budget
  }

  sys_log_prune_old_days();
}

#else
void sys_log_init(void)
{
  // No-op
}

void sys_log_process(void)
{
  // No-op
}

#endif

/* End of file -------------------------------------------------------- */
