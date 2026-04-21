/**
 * @file       bsp_usb.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-15
 * @author     Hai Tran
 *
 * @brief      BSP USB CDC driver definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_USB_H_
#define _BSP_USB_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_USB_EVENT_CONNECTED = 0,
  BSP_USB_EVENT_DISCONNECTED,
  BSP_USB_EVENT_DATA_RX,
} bsp_usb_event_t;

typedef void (*bsp_usb_callback_t)(bsp_usb_event_t event, void *arg);

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize USB CDC driver
 * @param[in] callback Callback function for USB events (connected, disconnected, data received)
 * @return none
 */
void bsp_usb_init(bsp_usb_callback_t callback);

/**
 * @brief Poll USB state — call periodically in a task loop
 * @return none
 */
void bsp_usb_process(void);

/**
 * @brief Deinitialize USB CDC driver
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_usb_deinit(void);

/**
 * @brief Send data over USB CDC
 * @param[in] data Pointer to the data buffer to send
 * @param[in] len Length of the data in bytes
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_usb_send(const uint8_t *data, size_t len);

/**
 * @brief Check the number of bytes available for reading
 * @return Number of bytes available for reading
 */
size_t bsp_usb_available(void);

/**
 * @brief Read data from USB CDC
 * @param[out] buf Pointer to the buffer to store the received data
 * @param[in] max_len Maximum number of bytes to read
 * @return Number of bytes actually read
 */
size_t bsp_usb_read(uint8_t *buf, size_t max_len);

#endif /*End file _BSP_USB_H_*/

/* End of file -------------------------------------------------------- */