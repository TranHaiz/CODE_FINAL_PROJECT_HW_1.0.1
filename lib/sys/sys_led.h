/**
 * @file       sys_led.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     System service LED control interface
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_LED_H_
#define _SYS_LED_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_led.h"
#include "common_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SYS_LED_EVT_OFF = 0,
  SYS_LED_EVT_HELP,
  SYS_LED_EVT_SERVICE_ENTER,
  SYS_LED_EVT_SERVICE_DONE,
  SYS_LED_EVT_SERVICE_ABORT,
  SYS_LED_EVT_NOTI_DANGER,
  SYS_LED_EVT_ERROR_FUEL_GAUGE,
  SYS_LED_EVT_ERROR_GPS,
  SYS_LED_EVT_ERROR_TEMP_HUM,
  SYS_LED_EVT_ERROR_IMU,
  SYS_LED_EVT_ERROR_COMPASS,
  SYS_LED_EVT_ERROR_SIM,
  SYS_LED_EVT_ERROR_NETWORK_LOST,
  SYS_LED_EVT_ERROR_RTC,
  SYS_LED_EVT_ERROR_SD,
  SYS_LED_EVT_ERROR_DISPLAY,
  SYS_LED_EVT_NOTI_RENTAL_LIMIT,
  SYS_LED_EVT_NOTI_WARNING_DEBT,
  SYS_LED_EVT_NOTI_LOW_BALANCE,
  SYS_LED_EVT_MAX
} sys_led_evt_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize the system LED (bsp_led)
 * @return none
 */
void sys_led_init(void);

/**
 * @brief Write an event to the system LED event queue
 * @param[in] event The event to write
 * @return none
 */
void sys_led_write_event(sys_led_evt_t event);

/**
 * @brief Clear a specific event from the service, use it for high priority events that need to be cleared immediately
 * @param[in] event The event to clear
 * @return none
 */
void sys_led_clear_event(sys_led_evt_t event);

/**
 * @brief Process the system LED events and update the LED state accordingly
 * @return none
 */
void sys_led_process(void);

/**
 * @brief Check if a specific LED event is currently active
 * @param[in] event The event to check
 * @return true if the event is active, false otherwise
 */
bool sys_led_is_event_active(sys_led_evt_t event);

/**
 * @brief Map a shared device error code to its LED error event
 * @param[in] code The device error code
 * @return The corresponding LED event
 */
sys_led_evt_t sys_led_evt_from_error(device_error_t code);

#endif /*End file _SYS_LED_H_*/

/* End of file -------------------------------------------------------- */
