/**
 * @file       sys_network.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0
 * @date       2026-04-10
 * @author     Hai Tran
 *
 * @brief      System Network Layer - Network management interface
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
#define SD_OFFLINE_LOG_PATH       "/buff/offline_log.json"

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern volatile bool is_data_network_ready;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the network layer (SIM, MQTT, cbuffer, semaphores).
 *
 * @return none
 */
void sys_network_init(void);

/**
 * @brief Network state machine process task entry point.
 *        Manages SIM/MQTT connection and publishes from cbuffer or SD.
 *
 * @param[in] param  Unused.
 *
 * @return none
 */
void sys_network_process(void *param);

/**
 * @brief Data task entry point.
 *        Reads telemetry from sys_input, builds JSON payload, pushes into cbuffer.
 *        When offline and cbuffer usage exceeds 80%, flushes to SD card.
 *
 * @param[in] param  Unused.
 *
 * @return none
 */
void sys_network_data_task(void *param);

/**
 * @brief Trigger a network wakeup (e.g. after device unlock).
 *
 * @return none
 */
void sys_network_wakeup(void);

/**
 * @brief Publish a notification or command response via MQTT.
 *
 * @param[in] payload Payload data to publish.
 * @param[in] payload_len Length of the payload data in bytes.
 *
 * @return none
 */
void sys_network_mqtt_publish_noti(const char *payload, size_t payload_len);

/**
 * @brief MQTT message callback for incoming commands.
 * @param[in] topic MQTT topic of the received message.
 * @param[in] data Pointer to the received payload data.
 * @param[in] len Length of the received payload data.
 * @return none
 */
void sys_network_mqtt_message_cb(const char *topic, const uint8_t *data, size_t len);

#endif /* End file _SYS_NETWORK_H_ */

/* End of file -------------------------------------------------------- */
