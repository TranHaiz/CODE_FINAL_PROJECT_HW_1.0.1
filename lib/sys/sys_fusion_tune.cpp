/**
 * @file       sys_fusion_tune.cpp
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-29
 * @author     Hai Tran
 *
 * @brief      Runtime tuning command handler for sys_fusion parameters.
 */

#include "sys_fusion_tune.h"

#if (DEVICE_FUSION_TUNING_MODE_ENABLED)

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

/* Public variables --------------------------------------------------- */
/* Single source of truth for defaults — used to seed g_fusion_params and to honour RESET. */
#define FUSION_PARAMS_DEFAULTS_INIT         \
  {                                         \
    .acc_ema_alpha               = 0.077f,  \
    .acc_threshold_ms2           = 0.02f,   \
    .active_motion_th_ms2        = 0.3f,    \
    .gps_anchor_transient_scale  = 0.2f,    \
    .attitude_gyro_weight        = 0.9975f, \
    .gyro_bias_alpha             = 0.01f,   \
    .acc_forward_max_ms2         = 3.0f,    \
    .yaw_gyro_weight             = 0.98f,   \
    .yaw_gyro_weight_stationary  = 0.85f,   \
    .yaw_rotating_th_degps       = 10.0f,   \
    .yaw_offset_deg              = 25.0f,   \
    .cf_wc                       = 1.0f,    \
    .cf_wc_transient             = 4.0f,    \
    .cf_wc_transient_th_ms2      = 1.5f,    \
    .cf_transient_agree_ms       = 2.0f,    \
    .cf_wc_stopping              = 6.0f,    \
    .vel_near_zero_ms            = 0.18f,   \
    .vel_settle_band_ms          = 0.12f,   \
    .vout_anchor_soft_rate       = 0.002f,  \
    .vout_snap_th_ms             = 3.0f,    \
    .zupt_acc_threshold          = 0.15f,   \
    .zupt_time_threshold_ms      = 400,     \
    .ins_decay_normal            = 0.990f,  \
    .ins_decay_stopping          = 0.75f,   \
    .ins_decay_gps_lost          = 0.95f,   \
    .gps_speed_min_ms            = 0.8f,    \
    .gps_anchor_rate             = 0.8f,    \
    .gps_reliability_threshold_m = 20.0f,   \
    .gps_hdop_max                = 3.0f,    \
    .gps_satellites_min          = 4,       \
    .gps_max_step_m              = 50.0f,   \
    .gps_valid_timeout_ms        = 2000,    \
    .gps_fade_timeout_ms         = 1000,    \
    .compass_ema_alpha           = 0.15f,   \
    .compass_update_ms           = 100,     \
  }

sys_fusion_params_t              g_fusion_params = FUSION_PARAMS_DEFAULTS_INIT;
static const sys_fusion_params_t k_defaults      = FUSION_PARAMS_DEFAULTS_INIT;

/* Private types ----------------------------------------------------- */
typedef enum
{
  TUNE_T_F32,
  TUNE_T_U32,
  TUNE_T_U8,
} tune_type_t;

typedef struct
{
  const char *name;
  tune_type_t type;
  void       *ptr;
  float       min;
  float       max;
} tune_entry_t;

/* Param table ------------------------------------------------------- */
static const tune_entry_t k_table[] = {
  { "ACC_EMA_ALPHA", TUNE_T_F32, &g_fusion_params.acc_ema_alpha, 0.001f, 1.0f },
  { "ACC_THRESHOLD_MS2", TUNE_T_F32, &g_fusion_params.acc_threshold_ms2, 0.0f, 5.0f },
  { "ACTIVE_MOTION_TH_MS2", TUNE_T_F32, &g_fusion_params.active_motion_th_ms2, 0.0f, 10.0f },
  { "GPS_ANCHOR_TRANSIENT_SCALE", TUNE_T_F32, &g_fusion_params.gps_anchor_transient_scale, 0.0f, 1.0f },
  { "ATTITUDE_GYRO_WEIGHT", TUNE_T_F32, &g_fusion_params.attitude_gyro_weight, 0.5f, 1.0f },
  { "GYRO_BIAS_ALPHA", TUNE_T_F32, &g_fusion_params.gyro_bias_alpha, 0.0f, 1.0f },
  { "ACC_FORWARD_MAX_MS2", TUNE_T_F32, &g_fusion_params.acc_forward_max_ms2, 0.5f, 20.0f },
  { "YAW_GYRO_WEIGHT", TUNE_T_F32, &g_fusion_params.yaw_gyro_weight, 0.5f, 1.0f },
  { "YAW_GYRO_WEIGHT_STATIONARY", TUNE_T_F32, &g_fusion_params.yaw_gyro_weight_stationary, 0.0f, 1.0f },
  { "YAW_ROTATING_TH_DEGPS", TUNE_T_F32, &g_fusion_params.yaw_rotating_th_degps, 0.0f, 100.0f },
  { "YAW_OFFSET_DEG", TUNE_T_F32, &g_fusion_params.yaw_offset_deg, -180.0f, 180.0f },
  { "CF_WC", TUNE_T_F32, &g_fusion_params.cf_wc, 0.1f, 50.0f },
  { "CF_WC_TRANSIENT", TUNE_T_F32, &g_fusion_params.cf_wc_transient, 0.1f, 50.0f },
  { "CF_WC_TRANSIENT_TH_MS2", TUNE_T_F32, &g_fusion_params.cf_wc_transient_th_ms2, 0.0f, 20.0f },
  { "CF_TRANSIENT_AGREE_MS", TUNE_T_F32, &g_fusion_params.cf_transient_agree_ms, 0.0f, 10.0f },
  { "CF_WC_STOPPING", TUNE_T_F32, &g_fusion_params.cf_wc_stopping, 0.1f, 50.0f },
  { "VEL_NEAR_ZERO_MS", TUNE_T_F32, &g_fusion_params.vel_near_zero_ms, 0.0f, 2.0f },
  { "VEL_SETTLE_BAND_MS", TUNE_T_F32, &g_fusion_params.vel_settle_band_ms, 0.0f, 2.0f },
  { "VOUT_ANCHOR_SOFT_RATE", TUNE_T_F32, &g_fusion_params.vout_anchor_soft_rate, 0.0f, 1.0f },
  { "VOUT_SNAP_TH_MS", TUNE_T_F32, &g_fusion_params.vout_snap_th_ms, 0.0f, 30.0f },
  { "ZUPT_ACC_THRESHOLD", TUNE_T_F32, &g_fusion_params.zupt_acc_threshold, 0.0f, 2.0f },
  { "ZUPT_TIME_THRESHOLD_MS", TUNE_T_U32, &g_fusion_params.zupt_time_threshold_ms, 100.0f, 10000.0f },
  { "INS_DECAY_NORMAL", TUNE_T_F32, &g_fusion_params.ins_decay_normal, 0.0f, 1.0f },
  { "INS_DECAY_STOPPING", TUNE_T_F32, &g_fusion_params.ins_decay_stopping, 0.0f, 1.0f },
  { "INS_DECAY_GPS_LOST", TUNE_T_F32, &g_fusion_params.ins_decay_gps_lost, 0.0f, 1.0f },
  { "GPS_SPEED_MIN_MS", TUNE_T_F32, &g_fusion_params.gps_speed_min_ms, 0.0f, 5.0f },
  { "GPS_ANCHOR_RATE", TUNE_T_F32, &g_fusion_params.gps_anchor_rate, 0.0f, 1.0f },
  { "GPS_RELIABILITY_THRESHOLD_M", TUNE_T_F32, &g_fusion_params.gps_reliability_threshold_m, 1.0f, 200.0f },
  { "GPS_HDOP_MAX", TUNE_T_F32, &g_fusion_params.gps_hdop_max, 1.0f, 20.0f },
  { "GPS_SATELLITES_MIN", TUNE_T_U8, &g_fusion_params.gps_satellites_min, 3.0f, 32.0f },
  { "GPS_MAX_STEP_M", TUNE_T_F32, &g_fusion_params.gps_max_step_m, 5.0f, 500.0f },
  { "GPS_VALID_TIMEOUT_MS", TUNE_T_U32, &g_fusion_params.gps_valid_timeout_ms, 500.0f, 30000.0f },
  { "GPS_FADE_TIMEOUT_MS", TUNE_T_U32, &g_fusion_params.gps_fade_timeout_ms, 200.0f, 10000.0f },
  { "COMPASS_EMA_ALPHA", TUNE_T_F32, &g_fusion_params.compass_ema_alpha, 0.0f, 1.0f },
  { "COMPASS_UPDATE_MS", TUNE_T_U32, &g_fusion_params.compass_update_ms, 10.0f, 1000.0f },
};
static const size_t k_table_count = sizeof(k_table) / sizeof(k_table[0]);

/* Private helpers --------------------------------------------------- */
static const tune_entry_t *find_entry(const char *name)
{
  for (size_t i = 0; i < k_table_count; i++)
  {
    if (strcasecmp(k_table[i].name, name) == 0)
      return &k_table[i];
  }
  return NULL;
}

static float read_value(const tune_entry_t *e)
{
  switch (e->type)
  {
  case TUNE_T_F32: return *(const float *) e->ptr;
  case TUNE_T_U32: return (float) (*(const uint32_t *) e->ptr);
  case TUNE_T_U8: return (float) (*(const uint8_t *) e->ptr);
  }
  return 0.0f;
}

static bool write_value(const tune_entry_t *e, float v)
{
  if (v < e->min || v > e->max)
    return false;
  switch (e->type)
  {
  case TUNE_T_F32: *(float *) e->ptr = v; return true;
  case TUNE_T_U32: *(uint32_t *) e->ptr = (uint32_t) v; return true;
  case TUNE_T_U8: *(uint8_t *) e->ptr = (uint8_t) v; return true;
  }
  return false;
}

static const char *type_str(tune_type_t t)
{
  switch (t)
  {
  case TUNE_T_F32: return "f32";
  case TUNE_T_U32: return "u32";
  case TUNE_T_U8: return "u8";
  }
  return "?";
}

static size_t append(char *buf, size_t cap, size_t off, const char *fmt, ...)
{
  if (off >= cap)
    return off;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf + off, cap - off, fmt, ap);
  va_end(ap);
  if (n < 0)
    return off;
  if ((size_t) n >= cap - off)
    return cap - 1;
  return off + (size_t) n;
}

static size_t format_value(char *out, size_t cap, const tune_entry_t *e)
{
  switch (e->type)
  {
  case TUNE_T_F32: return (size_t) snprintf(out, cap, "%.6f", *(const float *) e->ptr);
  case TUNE_T_U32: return (size_t) snprintf(out, cap, "%u", (unsigned) *(const uint32_t *) e->ptr);
  case TUNE_T_U8: return (size_t) snprintf(out, cap, "%u", (unsigned) *(const uint8_t *) e->ptr);
  }
  return 0;
}

/* Command parser ---------------------------------------------------- */
size_t sys_fusion_tune_handle_command(const char *cmd, char *out_resp, size_t out_resp_max)
{
  if (cmd == NULL || out_resp == NULL || out_resp_max == 0)
    return 0;

  // Skip leading whitespace
  while (*cmd && isspace((unsigned char) *cmd)) cmd++;

  // Parse verb
  char verb[8] = { 0 };
  int  i       = 0;
  while (*cmd && !isspace((unsigned char) *cmd) && i < (int) sizeof(verb) - 1)
    verb[i++] = (char) toupper((unsigned char) *cmd++);
  verb[i] = '\0';
  while (*cmd && isspace((unsigned char) *cmd)) cmd++;

  size_t off = 0;

  if (strcmp(verb, "LIST") == 0)
  {
    for (size_t k = 0; k < k_table_count; k++)
    {
      const tune_entry_t *e = &k_table[k];
      char                val[32];
      format_value(val, sizeof(val), e);
      off =
        append(out_resp, out_resp_max, off, "%s %s %.6f %.6f %s\n", e->name, type_str(e->type), e->min, e->max, val);
    }
    off = append(out_resp, out_resp_max, off, "END\n");
    return off;
  }

  if (strcmp(verb, "RESET") == 0)
  {
    g_fusion_params = k_defaults;
    return append(out_resp, out_resp_max, 0, "OK\n");
  }

  if (strcmp(verb, "GET") == 0)
  {
    char name[48] = { 0 };
    i             = 0;
    while (*cmd && !isspace((unsigned char) *cmd) && i < (int) sizeof(name) - 1) name[i++] = *cmd++;
    name[i]               = '\0';
    const tune_entry_t *e = find_entry(name);
    if (e == NULL)
      return append(out_resp, out_resp_max, 0, "ERR unknown %s\n", name);
    char val[32];
    format_value(val, sizeof(val), e);
    return append(out_resp, out_resp_max, 0, "%s=%s\n", e->name, val);
  }

  if (strcmp(verb, "SET") == 0)
  {
    char name[48] = { 0 };
    i             = 0;
    while (*cmd && !isspace((unsigned char) *cmd) && i < (int) sizeof(name) - 1) name[i++] = *cmd++;
    name[i] = '\0';
    while (*cmd && isspace((unsigned char) *cmd)) cmd++;
    if (*cmd == '\0')
      return append(out_resp, out_resp_max, 0, "ERR missing value\n");
    const tune_entry_t *e = find_entry(name);
    if (e == NULL)
      return append(out_resp, out_resp_max, 0, "ERR unknown %s\n", name);
    char *endp;
    float v = strtof(cmd, &endp);
    if (endp == cmd)
      return append(out_resp, out_resp_max, 0, "ERR bad value\n");
    if (!write_value(e, v))
      return append(out_resp, out_resp_max, 0, "ERR out of range [%.3f,%.3f]\n", e->min, e->max);
    return append(out_resp, out_resp_max, 0, "OK\n");
  }

  return append(out_resp, out_resp_max, 0, "ERR unknown verb\n");
}

#endif /* DEVICE_FUSION_TUNING_MODE_ENABLED */

/* End of file -------------------------------------------------------- */
