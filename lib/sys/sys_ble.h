/**
 * @file       sys_ble.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System service BLE
 *
 */

#ifndef SYS_BLE_H
#define SYS_BLE_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the system BLE manager
 * @return none
 */
void sys_ble_init(void);

/**
 * @brief Process BLE events
 * @return none
 */
void sys_ble_process(void);

/**
 * @brief Send data over BLE (stored in TX static buffer, max 10 messages)
 *
 * @param data Pointer to the data to send
 * @param len  Length of data
 */
void sys_ble_send(const uint8_t *data, size_t len);

/**
 * @brief Check if BLE is currently connected
 * @return true if connected, false otherwise
 */
bool sys_ble_is_connected(void);

#endif /* SYS_BLE_H */

/* End of file -------------------------------------------------------- */
