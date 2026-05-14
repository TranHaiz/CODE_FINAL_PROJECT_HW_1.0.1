/**
 * @file       sys_network.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      Network dispatcher — owns cbuffer/SD, chooses active adapter,
 *             routes telemetry and commands through it.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_NETWORK_H_
#define _SYS_NETWORK_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_sim.h"
#include "common_type.h"
#include "sys_input.h"

/* Public defines ----------------------------------------------------- */
#define MQTT_REQUEST_PUBLISH_SIZE (128)

/* Public variables --------------------------------------------------- */
extern volatile bool is_data_network_ready;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the dispatcher: cbuffer (PSRAM), SD, mutexes,
 *        and both adapters.  Call from setup() before spawning tasks.
 */
void sys_network_init(void);

/**
 * @brief Start a new trip session: generates a sequential trip ID from SD,
 * @return none
 */
void sys_network_trigger_new_trip(void);

/**
 * @brief End the active trip: marks trip_info.dat as COMPLETED and backlog SD uploads.
 * @return none
 */
void sys_network_trigger_end_trip(void);

/**
 * @brief Delete all files in the offline SD directory and resetstate Safe to call before reboot.
 * @return none
 */
void sys_network_reset_offline_data(void);

/**
 * @brief Dispatcher task entry point.
 *        Pulls telemetry, manages adapter switching, drains noti/data queues.
 */
void sys_network_task(void *param);

/**
 * @brief Trigger a network wakeup (e.g. after device unlock).
 */
void sys_network_wakeup(void);

/**
 * @brief Enqueue a notification/command-response for the active adapter.
 *        Thread-safe; may be called from any task.
 *
 * @param[in] payload     Null-terminated string payload.
 * @param[in] payload_len Length of payload (excluding null terminator).
 */
void sys_network_publish_noti(const char *payload, size_t payload_len);

#endif /* End file _SYS_NETWORK_H_ */

/* End of file -------------------------------------------------------- */
