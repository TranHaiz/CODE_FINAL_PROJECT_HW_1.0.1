/**
 * @file       sys_ui.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Full UI Layer)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      UI system implementation
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_UI_H_
#define _SYS_UI_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"
#include "bsp_touch.h"
#include "lvgl_driver.h"
#include "sys_ui_widget.h"

#include <Arduino.h>
#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
#define SYS_UI_SPEED_REFRESH_MS (50)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_UI_VIEW_MAIN = 0,
  SYS_UI_VIEW_SETTINGS,
  SYS_UI_VIEW_OUT,
  SYS_UI_VIEW_TIME,
  SYS_UI_VIEW_DISTANCE,
  SYS_UI_VIEW_TEMPERATURE,
  SYS_UI_VIEW_LOCK,
  SYS_UI_VIEW_UNKNOWN
} sys_ui_view_t;

// Order = priority: lower enum value = higher priority (shown first when multiple active).
// NONE = 0 is a sentinel — never used as a real label.
typedef enum
{
  SYS_UI_NOTI_LABEL_NONE = 0,
  SYS_UI_NOTI_LABEL_LOW_BATT,
  SYS_UI_NOTI_LABEL_RENTAL_LIMIT,
  SYS_UI_NOTI_LABEL_OUT_OF_ZONE,
  SYS_UI_NOTI_LABEL_WARN_ADD_FUND,
  SYS_UI_NOTI_LABEL_SHOULD_ADD_FUND,
  SYS_UI_NOTI_LABEL_MAX
} sys_ui_noti_label_type_t;

typedef struct
{
  bool is_fusion_data_ready_for_ui;
  bool is_dust_data_ready_for_ui;
  bool is_temp_hum_data_ready_for_ui;
  bool is_battery_data_ready_for_ui;
} sys_ui_data_status_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern sys_ui_data_status_t g_sys_ui_data_status;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize UI system (display, LVGL, widgets)
 */
void sys_ui_init(void);

/**
 * @brief Process UI loop iteration (update telemetry, refresh display)
 */
void sys_ui_process(void);

/**
 * @brief Lock the device (for rental end)
 */
void sys_ui_lock(void);

/**
 * @brief Unlock the device from lock screen (for rental start)
 */
void sys_ui_unlock(void);

/**
 * @brief Wake up the device from idle/sleep mode
 */
void sys_ui_wakeup(void);

/**
 * @brief Change to main screen and update time (for rental start or resume)
 * @return none
 */
void sys_ui_change_time_active(void);

/**
 * @brief Activate a notification label (priority-based: highest active is shown)
 * @param[in] label: notification label to mark active
 */
void sys_ui_noti_set(sys_ui_noti_label_type_t label);

/**
 * @brief Deactivate a specific notification label
 * @param[in] label: notification label to clear
 */
void sys_ui_noti_clear(sys_ui_noti_label_type_t label);

/**
 * @brief Clear all notification labels
 */
void sys_ui_noti_clear_all(void);

#endif /*End file _SYS_UI_H_*/

/* End of file -------------------------------------------------------- */