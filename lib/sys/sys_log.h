/**
 * @file       sys_log.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      System log to SD card - buffering and writing logs
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_LOG_H_
#define _SYS_LOG_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "log_service.h"

/* Public defines ----------------------------------------------------- */
#define SYS_LOG_BUFFER_SIZE       (2048)
#define SYS_LOG_BUFFER_THRESHOLD  (80)  // Percent
#define SYS_LOG_FLUSH_INTERVAL_MS (30000)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize SD card logging
 *
 * @return none
 */
void sys_log_init(void);

/**
 * @brief Process function, call in thread loop
 * @return none
 */
void sys_log_process(void);

/**
 * @brief Deinitialize logging, flush remaining logs
 * @return none
 */
void sys_log_deinit(void);

#endif /*End file _SYS_LOG_H_*/

/* End of file -------------------------------------------------------- */
