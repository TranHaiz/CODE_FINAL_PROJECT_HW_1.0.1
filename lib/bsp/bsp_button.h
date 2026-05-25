/**
 * @file       bsp_button.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-04-28
 * @author     Hai Tran
 *
 * @brief      Button control interface
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_BUTTON_H_
#define _BSP_BUTTON_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
#define BUTTON_DEBOUNCE_MS     (50)
#define BUTTON_SHORT_PRESS_MS  (1000)
#define BUTTON_LONG_PRESS_MS   (15000)
#define BUTTON_DOUBLE_PRESS_MS (500)
#define BUTTON_SERVICE_HOLD_MS (5000)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BUTTON_EVT = 0,
  BUTTON_MAX
} bsp_button_type_t;

typedef enum
{
  BUTTON_PRESS_SHORT = 0,
  BUTTON_PRESS_LONG,
  BUTTON_PRESS_COUNT,
  BUTTON_PRESS_SERVICE,
} bsp_button_press_type_t;

typedef void (*bsp_button_callback_t)(bsp_button_press_type_t press_type);
typedef void (*bsp_button_isr_callback_t)(void);

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Button initialization
 * @param[in]  button: button type
 * @param[in]  callback: callback function to call on button event
 * @return none
 */
void bsp_button_init(bsp_button_type_t button, bsp_button_callback_t callback);

/**
 * @brief  Set an ISR callback to be triggered immediately in the button ISR
 * @param[in]  button: button type
 * @param[in]  isr_cb: callback function
 * @return none
 */
void bsp_button_set_isr_callback(bsp_button_type_t button, bsp_button_isr_callback_t isr_cb);

/**
 * @brief  Button process task to be called in main loop or dedicated thread
 * @return none
 */
void bsp_button_process(void);

/**
 * @brief  Get the last reported click count
 * @param[in]  button: button type
 * @return Number of clicks
 */
uint8_t bsp_button_get_count(bsp_button_type_t button);

/**
 * @brief  Check if service mode is pending (long press detected but not yet serviced)
 * @return true if service mode is pending, false otherwise
 */
bool bsp_button_is_service_pending(void);

#endif /*End file _BSP_BUTTON_H_*/

/* End of file -------------------------------------------------------- */
