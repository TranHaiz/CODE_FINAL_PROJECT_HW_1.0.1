
/**
 * @file       sys_cmd.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief    System command processing header file
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_CMD_H_
#define _SYS_CMD_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"
#include "os_lib.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  CMD_NONE = 0,
  CMD_LOCK_DEVICE,
  CMD_UNLOCK_DEVICE,
  CMD_SET_TIME,
  CMD_MAX
} sys_command_id_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
OS_SEM_DECLARE(sys_cmd_req_sem)

/* Public function prototypes ----------------------------------------- */

#endif /*End file _SYS_CMD_H_*/

/* End of file -------------------------------------------------------- */
