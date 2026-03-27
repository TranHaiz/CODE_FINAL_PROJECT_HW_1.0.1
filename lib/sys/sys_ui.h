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
  SYS_UI_VIEW_TEMPERATURE
} sys_ui_view_t;

typedef struct
{
  bool is_fusion_data_ready_for_ui;
  bool is_env_data_ready_for_ui;
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

#endif /*End file _SYS_UI_H_*/

/* End of file -------------------------------------------------------- */