/**
 * @file       sys_error.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System error manager
 *
 */

#ifndef SYS_ERROR_H
#define SYS_ERROR_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the system error manager
 */
void sys_error_init(void);

/**
 * @brief Process system errors in RTOS task
 */
void sys_error_process(void);

#endif /* SYS_ERROR_H */

/* End of file -------------------------------------------------------- */