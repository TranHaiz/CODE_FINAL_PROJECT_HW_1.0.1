/**
 * @file       sys_network_adapter_lte.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      LTE/SIM network adapter (SIM + MQTT FSM, v2 state machine preserved).
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef SYS_NETWORK_ADAPTER_LTE_H
#define SYS_NETWORK_ADAPTER_LTE_H

/* Includes ----------------------------------------------------------- */
#include "sys_network_adapter.h"

/* Public variables --------------------------------------------------- */
extern net_adapter_t g_net_adapter_lte;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief LTE adapter task entry point — loops on poll().
 * @return none
 */
void sys_network_adapter_lte_task(void *param);

/**
 * @brief MQTT message callback for PubSubClient subscription.
 * @param[in] topic MQTT topic string
 * @param[in] data Pointer to received data
 * @param[in] len Length of received data
 * @return none
 */
void sys_network_adapter_lte_mqtt_cb(const char *topic, const uint8_t *data, size_t len);

#endif /* SYS_NETWORK_ADAPTER_LTE_H */

/* End of file -------------------------------------------------------- */
