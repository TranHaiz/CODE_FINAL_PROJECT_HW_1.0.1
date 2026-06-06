/**
 * @file       sys_fusion_tune.h
 * @copyright  Copyright (C) 2026 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-29
 * @author     Hai Tran
 *
 * @brief      Runtime tuning for sys_fusion parameters.
 *
 *   When DEVICE_FUSION_TUNING_MODE_ENABLED:
 *     - g_fusion_params holds live values, seeded from the compile-time defaults
 *     - This header #undef's the compile-time #defines and rebinds them to struct fields
 *     - sys_fusion_tune_handle_command() parses text commands (LIST / GET / SET)
 *
 *   When disabled, this header is a no-op and the compile-time #defines stand.
 */

#ifndef SYS_FUSION_TUNE_H
#define SYS_FUSION_TUNE_H

#include "device_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if (DEVICE_FUSION_TUNING_MODE_ENABLED)

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    // Acc filter + dead-band
    float acc_ema_alpha;
    float acc_threshold_ms2;

    // Active motion / GPS anchor scaling
    float active_motion_th_ms2;
    float gps_anchor_transient_scale;

    // Attitude CF
    float attitude_gyro_weight;
    float gyro_bias_alpha;
    float acc_forward_max_ms2;

    // Yaw CF
    float yaw_gyro_weight;
    float yaw_gyro_weight_stationary;
    float yaw_rotating_th_degps;
    float yaw_offset_deg;

    // Velocity CF
    float cf_wc;
    float cf_wc_transient;
    float cf_wc_transient_th_ms2;
    float cf_transient_agree_ms;
    float cf_wc_stopping;
    float vel_near_zero_ms;
    float vel_settle_band_ms;

    // Vout anchor
    float vout_anchor_soft_rate;
    float vout_snap_th_ms;

    // ZUPT
    float    zupt_acc_threshold;
    uint32_t zupt_time_threshold_ms;
    float    ins_decay_normal;
    float    ins_decay_stopping;
    float    ins_decay_gps_lost;

    // GPS
    float    gps_speed_min_ms;
    float    gps_anchor_rate;
    float    gps_reliability_threshold_m;
    float    gps_hdop_max;
    uint8_t  gps_satellites_min;
    float    gps_max_step_m;
    uint32_t gps_valid_timeout_ms;
    uint32_t gps_fade_timeout_ms;

    // Compass
    float    compass_ema_alpha;
    uint32_t compass_update_ms;
  } sys_fusion_params_t;

  extern sys_fusion_params_t g_fusion_params;

  /**
   * @brief Parse one command line and produce a response.
   *
   *   Accepted commands (case-insensitive verb):
   *     LIST              => multi-line output, each line "NAME TYPE MIN MAX CURRENT", ended by "END"
   *     GET <name>        => "NAME=value" or "ERR ..."
   *     SET <name> <val>  => "OK" or "ERR ..."
   *
   * @param  cmd          NUL-terminated command (newline stripped)
   * @param  out_resp     destination buffer for response (NUL-terminated)
   * @param  out_resp_max capacity of out_resp
   * @return number of bytes written to out_resp (excluding NUL)
   */
  size_t sys_fusion_tune_handle_command(const char *cmd, char *out_resp, size_t out_resp_max);

#ifdef __cplusplus
}
#endif

/* -------- Rebind compile-time #defines to runtime struct fields ------- */
/*  sys_fusion.cpp defines these constants near the top of the file.       */
/*  We #undef them and redefine to g_fusion_params reads so all existing   */
/*  references in the body of sys_fusion.cpp become live look-ups.         */

#ifdef ACC_EMA_ALPHA
#undef ACC_EMA_ALPHA
#endif
#define ACC_EMA_ALPHA (g_fusion_params.acc_ema_alpha)

#ifdef ACC_THRESHOLD_MS2
#undef ACC_THRESHOLD_MS2
#endif
#define ACC_THRESHOLD_MS2 (g_fusion_params.acc_threshold_ms2)

#ifdef ACTIVE_MOTION_TH_MS2
#undef ACTIVE_MOTION_TH_MS2
#endif
#define ACTIVE_MOTION_TH_MS2 (g_fusion_params.active_motion_th_ms2)

#ifdef GPS_ANCHOR_TRANSIENT_SCALE
#undef GPS_ANCHOR_TRANSIENT_SCALE
#endif
#define GPS_ANCHOR_TRANSIENT_SCALE (g_fusion_params.gps_anchor_transient_scale)

#ifdef ATTITUDE_GYRO_WEIGHT
#undef ATTITUDE_GYRO_WEIGHT
#endif
#define ATTITUDE_GYRO_WEIGHT (g_fusion_params.attitude_gyro_weight)

#ifdef GYRO_BIAS_ALPHA
#undef GYRO_BIAS_ALPHA
#endif
#define GYRO_BIAS_ALPHA (g_fusion_params.gyro_bias_alpha)

#ifdef ACC_FORWARD_MAX_MS2
#undef ACC_FORWARD_MAX_MS2
#endif
#define ACC_FORWARD_MAX_MS2 (g_fusion_params.acc_forward_max_ms2)

#ifdef YAW_GYRO_WEIGHT
#undef YAW_GYRO_WEIGHT
#endif
#define YAW_GYRO_WEIGHT (g_fusion_params.yaw_gyro_weight)

#ifdef YAW_GYRO_WEIGHT_STATIONARY
#undef YAW_GYRO_WEIGHT_STATIONARY
#endif
#define YAW_GYRO_WEIGHT_STATIONARY (g_fusion_params.yaw_gyro_weight_stationary)

#ifdef YAW_ROTATING_TH_DEGPS
#undef YAW_ROTATING_TH_DEGPS
#endif
#define YAW_ROTATING_TH_DEGPS (g_fusion_params.yaw_rotating_th_degps)

#ifdef YAW_OFFSET_DEG
#undef YAW_OFFSET_DEG
#endif
#define YAW_OFFSET_DEG (g_fusion_params.yaw_offset_deg)

#ifdef CF_WC
#undef CF_WC
#endif
#define CF_WC (g_fusion_params.cf_wc)

#ifdef CF_WC_TRANSIENT
#undef CF_WC_TRANSIENT
#endif
#define CF_WC_TRANSIENT (g_fusion_params.cf_wc_transient)

#ifdef CF_WC_TRANSIENT_TH_MS2
#undef CF_WC_TRANSIENT_TH_MS2
#endif
#define CF_WC_TRANSIENT_TH_MS2 (g_fusion_params.cf_wc_transient_th_ms2)

#ifdef CF_TRANSIENT_AGREE_MS
#undef CF_TRANSIENT_AGREE_MS
#endif
#define CF_TRANSIENT_AGREE_MS (g_fusion_params.cf_transient_agree_ms)

#ifdef CF_WC_STOPPING
#undef CF_WC_STOPPING
#endif
#define CF_WC_STOPPING (g_fusion_params.cf_wc_stopping)

#ifdef VEL_NEAR_ZERO_MS
#undef VEL_NEAR_ZERO_MS
#endif
#define VEL_NEAR_ZERO_MS (g_fusion_params.vel_near_zero_ms)

#ifdef VEL_SETTLE_BAND_MS
#undef VEL_SETTLE_BAND_MS
#endif
#define VEL_SETTLE_BAND_MS (g_fusion_params.vel_settle_band_ms)

#ifdef VOUT_ANCHOR_SOFT_RATE
#undef VOUT_ANCHOR_SOFT_RATE
#endif
#define VOUT_ANCHOR_SOFT_RATE (g_fusion_params.vout_anchor_soft_rate)

#ifdef VOUT_SNAP_TH_MS
#undef VOUT_SNAP_TH_MS
#endif
#define VOUT_SNAP_TH_MS (g_fusion_params.vout_snap_th_ms)

#ifdef ZUPT_ACC_THRESHOLD
#undef ZUPT_ACC_THRESHOLD
#endif
#define ZUPT_ACC_THRESHOLD (g_fusion_params.zupt_acc_threshold)

#ifdef ZUPT_TIME_THRESHOLD_MS
#undef ZUPT_TIME_THRESHOLD_MS
#endif
#define ZUPT_TIME_THRESHOLD_MS (g_fusion_params.zupt_time_threshold_ms)

#ifdef INS_DECAY_NORMAL
#undef INS_DECAY_NORMAL
#endif
#define INS_DECAY_NORMAL (g_fusion_params.ins_decay_normal)

#ifdef INS_DECAY_STOPPING
#undef INS_DECAY_STOPPING
#endif
#define INS_DECAY_STOPPING (g_fusion_params.ins_decay_stopping)

#ifdef INS_DECAY_GPS_LOST
#undef INS_DECAY_GPS_LOST
#endif
#define INS_DECAY_GPS_LOST (g_fusion_params.ins_decay_gps_lost)

#ifdef GPS_SPEED_MIN_MS
#undef GPS_SPEED_MIN_MS
#endif
#define GPS_SPEED_MIN_MS (g_fusion_params.gps_speed_min_ms)

#ifdef GPS_ANCHOR_RATE
#undef GPS_ANCHOR_RATE
#endif
#define GPS_ANCHOR_RATE (g_fusion_params.gps_anchor_rate)

#ifdef GPS_RELIABILITY_THRESHOLD_M
#undef GPS_RELIABILITY_THRESHOLD_M
#endif
#define GPS_RELIABILITY_THRESHOLD_M (g_fusion_params.gps_reliability_threshold_m)

#ifdef GPS_HDOP_MAX
#undef GPS_HDOP_MAX
#endif
#define GPS_HDOP_MAX (g_fusion_params.gps_hdop_max)

#ifdef GPS_SATELLITES_MIN
#undef GPS_SATELLITES_MIN
#endif
#define GPS_SATELLITES_MIN (g_fusion_params.gps_satellites_min)

#ifdef GPS_MAX_STEP_M
#undef GPS_MAX_STEP_M
#endif
#define GPS_MAX_STEP_M (g_fusion_params.gps_max_step_m)

#ifdef GPS_VALID_TIMEOUT_MS
#undef GPS_VALID_TIMEOUT_MS
#endif
#define GPS_VALID_TIMEOUT_MS (g_fusion_params.gps_valid_timeout_ms)

#ifdef GPS_FADE_TIMEOUT_MS
#undef GPS_FADE_TIMEOUT_MS
#endif
#define GPS_FADE_TIMEOUT_MS (g_fusion_params.gps_fade_timeout_ms)

#ifdef COMPASS_EMA_ALPHA
#undef COMPASS_EMA_ALPHA
#endif
#define COMPASS_EMA_ALPHA (g_fusion_params.compass_ema_alpha)

#ifdef COMPASS_UPDATE_MS
#undef COMPASS_UPDATE_MS
#endif
#define COMPASS_UPDATE_MS (g_fusion_params.compass_update_ms)

#endif /* DEVICE_FUSION_TUNING_MODE_ENABLED */

#endif /* SYS_FUSION_TUNE_H */

/* End of file -------------------------------------------------------- */
