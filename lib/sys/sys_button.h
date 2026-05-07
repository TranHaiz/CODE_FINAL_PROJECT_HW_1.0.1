/**
 * @file       sys_button.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-04-28
 * @author     Hai Tran
 *
 * @brief      System Button Layer
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_BUTTON_H_
#define _SYS_BUTTON_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_button.h"
#include "common_type.h"
#include "os_lib.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
OS_SEM_DECLARE(sys_button_wakeup_sem);

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  System button initialization
 * @return none
 */
void sys_button_init(void);

/**
 * @brief  System button process task
 * @return none
 */
void sys_button_process(void);

#endif /*End file _SYS_BUTTON_H_*/

/* End of file -------------------------------------------------------- */