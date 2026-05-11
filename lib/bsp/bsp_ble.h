/**
 * @file       bsp_ble.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      Board Support Package for Bluetooth Low Energy (BLE)
 *
 */

#ifndef BSP_BLE_H
#define BSP_BLE_H

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <stddef.h>
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_BLE_EVT_NONE = 0,
  BSP_BLE_EVT_CONNECT,
  BSP_BLE_EVT_DISCONNECT,
  BSP_BLE_EVT_RECEIVE_DATA
} bsp_ble_event_t;

typedef void (*bsp_ble_cb_t)(bsp_ble_event_t event);

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize BLE
 * @param[in] cb Callback function to notify BLE events
 * @return status_function_t STATUS_OK if successful
 */
status_function_t bsp_ble_init(bsp_ble_cb_t cb);

/**
 * @brief Start BLE advertising
 * @return status_function_t STATUS_OK if successful
 */
status_function_t bsp_ble_advertise_start(void);

/**
 * @brief Stop BLE advertising
 * @return status_function_t STATUS_OK if successful
 */
status_function_t bsp_ble_advertise_stop(void);

/**
 * @brief Send data over BLE
 * @param[in] data Pointer to data to send
 * @param[in] size Size of data
 * @return status_function_t STATUS_OK if successful, STATUS_ERROR if no connection
 */
status_function_t bsp_ble_send(const uint8_t *data, size_t size);

/**
 * @brief Disconnect current BLE connection
 * @return status_function_t STATUS_OK if successful
 */
status_function_t bsp_ble_disconnect(void);

/**
 * @brief Get received data from static buffer
 * @param[out] out_buffer Pointer to application buffer
 * @param[in] max_size Maximum length to copy
 * @return size_t Length of data copied
 */
size_t bsp_ble_get(uint8_t *out_buffer, size_t max_size);

#endif /* BSP_BLE_H */

/* End of file -------------------------------------------------------- */
