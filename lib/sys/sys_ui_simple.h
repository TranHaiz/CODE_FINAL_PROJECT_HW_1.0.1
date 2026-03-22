
/**
 * @file       sys_ui_simple.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief    Simple UI screen creation functions for quick prototyping.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_UI_SIMPLE_H_
#define _SYS_UI_SIMPLE_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"
#include "bsp_touch.h"
#include "common_type.h"
#include "lvgl_driver.h"
#include "sys_ui_widget.h"

#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
#define SYS_UI_TASK_SLEEP_MS (10)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_UI_SIMPLE_VIEW_MAIN = 0,
  SYS_UI_SIMPLE_VIEW_LOGIN,
} sys_ui_simple_view_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern bool is_ui_data_ready;

/* Public function prototypes ----------------------------------------- */
void                 sys_ui_simple_init(void);
void                 sys_ui_simple_process(void);
void                 sys_ui_simple_change_ui(sys_ui_simple_view_t view);
sys_ui_simple_view_t sys_ui_simple_get_current_view(void);

#endif /*End file _SYS_UI_SIMPLE_H_*/

/* End of file -------------------------------------------------------- */
