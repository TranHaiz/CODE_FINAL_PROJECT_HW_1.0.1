/**
 * @file       sys_fusion_log.cpp
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-24
 * @author     Hai Tran
 *
 * @brief      Dedicated CSV log of fusion data to SD card
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_fusion_log.h"

#if (DEVICE_FUSION_DEBUG_LOG_ENABLED == 1)

#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "cbuffer.h"
#include "log_service.h"
#include "os_lib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_fusion_log, LOG_LEVEL_INFO)

#define FUSION_LOG_BUFFER_SIZE       (8192)  // 8KB ring buffer (DRAM)
#define FUSION_LOG_FLUSH_THRESHOLD   (50)    // % full → trigger flush
#define FUSION_LOG_FLUSH_INTERVAL_MS (2000)  // Fallback flush interval
#define FUSION_LOG_LINE_MAX          (512)
#define FUSION_LOG_PATH_MAX          (64)

#define FUSION_LOG_HEADER                                              \
  "t_ms,"                                                              \
  "acc_raw_x,acc_raw_y,acc_raw_z,"                                     \
  "acc_filter_x,acc_filter_y,acc_filter_z,"                            \
  "gyro_raw_x,gyro_raw_y,gyro_raw_z,"                                  \
  "gyro_filter_x,gyro_filter_y,gyro_filter_z,"                         \
  "compass_raw_x,compass_raw_y,compass_raw_z,"                         \
  "compass_filter_x,compass_filter_y,compass_filter_z,"                \
  "acc_forward,roll_deg,pitch_deg,heading_deg,"                        \
  "vins_ms,vgps_ms,vout_ms,"                                           \
  "distance_ins_m,distance_gps_m,distance_out_m,"                      \
  "gps_state,is_stationary,gps_reliable,hdop,satellites,"              \
  "lat,lon\n"

/* Private variables -------------------------------------------------- */
static uint8_t   buf_storage[FUSION_LOG_BUFFER_SIZE];
static cbuffer_t cbuf;
static char      log_path[FUSION_LOG_PATH_MAX];
static size_t    last_flush_ms = 0;
static bool      initialized   = false;

/* Private function prototypes ---------------------------------------- */
static status_function_t flush_to_sd(void);

/* Function definitions ----------------------------------------------- */
void sys_fusion_log_init(void)
{
  if (initialized)
    return;

  cb_init(&cbuf, buf_storage, FUSION_LOG_BUFFER_SIZE);

  timeline_t t = { 0 };
  bsp_rtc_get(&t);
  snprintf(log_path, sizeof(log_path), "/logs/fusion_%lu-%u-%u.csv",
           (unsigned long) t.year, (unsigned) t.month, (unsigned) t.date);

  if (bsp_sdcard_is_mounted() == STATUS_OK)
  {
    bsp_sdcard_mkdir("/logs");
    if (bsp_sdcard_file_exists(log_path) != STATUS_OK)
    {
      bsp_sdcard_file_t f;
      if (bsp_sdcard_open(log_path, BSP_SDCARD_MODE_WRITE, &f) == STATUS_OK)
      {
        bsp_sdcard_write(&f, (const uint8_t *) FUSION_LOG_HEADER, strlen(FUSION_LOG_HEADER), nullptr);
        bsp_sdcard_close(&f);
        LOG_INF("Fusion log created: %s", log_path);
      }
    }
    else
    {
      LOG_INF("Fusion log appending: %s", log_path);
    }
  }

  last_flush_ms = OS_GET_TICK();
  initialized   = true;
}

void sys_fusion_log_push(const sys_fusion_data_t *data, size_t t_ms)
{
  if (!initialized || data == NULL)
    return;

  char line[FUSION_LOG_LINE_MAX];
  int  len = snprintf(line, sizeof(line),
                      "%lu,"
                      "%.4f,%.4f,%.4f,"
                      "%.4f,%.4f,%.4f,"
                      "%.3f,%.3f,%.3f,"
                      "%.3f,%.3f,%.3f,"
                      "%.1f,%.1f,%.1f,"
                      "%.1f,%.1f,%.1f,"
                      "%.3f,%.2f,%.2f,%.1f,"
                      "%.3f,%.3f,%.3f,"
                      "%.2f,%.2f,%.2f,"
                      "%u,%u,%u,%.2f,%u,"
                      "%.7f,%.7f\n",
                      (unsigned long) t_ms,
                      data->debug.acc_raw_x, data->debug.acc_raw_y, data->debug.acc_raw_z,
                      data->debug.acc_filter_x, data->debug.acc_filter_y, data->debug.acc_filter_z,
                      data->debug.gyro_raw_x, data->debug.gyro_raw_y, data->debug.gyro_raw_z,
                      data->debug.gyro_filter_x, data->debug.gyro_filter_y, data->debug.gyro_filter_z,
                      data->debug.compass_raw_x, data->debug.compass_raw_y, data->debug.compass_raw_z,
                      data->debug.compass_filter_x, data->debug.compass_filter_y, data->debug.compass_filter_z,
                      data->debug.acc_forward, data->debug.roll_deg, data->debug.pitch_deg, data->heading_deg,
                      data->debug.v_ins, data->debug.v_gps, data->debug.v_out,
                      data->debug.distance_ins_total, data->debug.distance_gps_total, data->distance_m,
                      (unsigned) data->debug.gps_state, (unsigned) data->debug.is_stationary,
                      (unsigned) data->debug.gps_reliable, data->debug.hdop,
                      (unsigned) data->debug.satellites,
                      data->debug.lat, data->debug.lon);

  if (len <= 0 || len >= (int) sizeof(line))
    return;

  // Async-only: never block caller on SD I/O. Drop sample if buffer full.
  cb_write(&cbuf, line, (uint32_t) len);
}

void sys_fusion_log_process(void)
{
  if (!initialized)
    return;
  if (bsp_sdcard_is_mounted() != STATUS_OK)
    return;

  uint32_t used = cb_data_count(&cbuf);
  uint32_t pct  = (used * 100U) / FUSION_LOG_BUFFER_SIZE;

  bool should_flush = false;
  if (pct >= FUSION_LOG_FLUSH_THRESHOLD)
    should_flush = true;
  if ((OS_GET_TICK() - last_flush_ms) >= FUSION_LOG_FLUSH_INTERVAL_MS && used > 0)
    should_flush = true;

  if (should_flush)
    flush_to_sd();
}

void sys_fusion_log_flush(void)
{
  if (!initialized)
    return;
  flush_to_sd();
  LOG_INF("Fusion log force-flushed: %s", log_path);
}

/* Private definitions ----------------------------------------------- */
static status_function_t flush_to_sd(void)
{
  uint32_t len = cb_data_count(&cbuf);
  if (len == 0)
    return STATUS_OK;

  char *tmp = (char *) malloc(len + 1);
  if (tmp == NULL)
    return STATUS_ERROR;

  uint32_t read_len = cb_read(&cbuf, tmp, len);

  bsp_sdcard_file_t f;
  status_function_t ret = bsp_sdcard_open(log_path, BSP_SDCARD_MODE_APPEND, &f);
  if (ret == STATUS_OK)
  {
    bsp_sdcard_write(&f, (const uint8_t *) tmp, read_len, nullptr);
    bsp_sdcard_close(&f);
  }

  free(tmp);
  last_flush_ms = OS_GET_TICK();
  return ret;
}

#endif /* DEVICE_FUSION_DEBUG_LOG_ENABLED */

/* End of file -------------------------------------------------------- */
