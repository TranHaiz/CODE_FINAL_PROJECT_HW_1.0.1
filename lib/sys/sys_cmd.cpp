
/**
 * @file       sys_cmd.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System command processing implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_cmd.h"

#include "log_service.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_cmd, LOG_LEVEL_DBG)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  char command[32];
  void (*handler)(void *param);
} sys_command_t;

/* Private function prototypes ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

// clang-format off
#define INFO(cmd, name) [cmd] = name
char *SYS_COMMAND_STR[CMD_MAX] = {
    INFO(CMD_NONE, "None"),
    INFO(CMD_LOCK_DEVICE, "Lock device"),
    INFO(CMD_UNLOCK_DEVICE, "Unlock device"),
    INFO(CMD_SET_TIME, "Set time")
};
#undef INFO
// clang-format on

/* Public variables --------------------------------------------------- */
OS_SEM_DEFINE_GLOBAL(sys_cmd_req_sem)

/* Private variables -------------------------------------------------- */
/* Function definitions ----------------------------------------------- */
/* Private definitions ----------------------------------------------- */
/* End of file -------------------------------------------------------- */
