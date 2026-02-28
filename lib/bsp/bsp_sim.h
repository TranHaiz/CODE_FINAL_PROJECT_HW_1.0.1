/**
 * @file       bsp_sim.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      BSP driver for SIM module (UART DMA Idle)
 *              - Init SIM module
 *              - API contact SMS
 *              - API call via phone number
 *              - API contact with firebase
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_SIM_H_
#define _BSP_SIM_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"
#include "common_type.h"
#include "device_info.h"

/* Public defines ----------------------------------------------------- */
#define SIM_UART_HANDLE    UART_NUM_2
#define SIM_RX_BUFFER_SIZE (512)
#define SIM_RAW_RSP_SIZE   (128)

#if (CONFIG_FIREBASE_SERVER == true)
#define FIREBASE_DEVICE_PATH        "https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json\"
#define FIREBASE_READ_POSION_FIELD  "Position.json"
#define FIREBASE_READ_BATTERY_FIELD "Battery%20level.json"
#endif

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SIM_DATA_FIELD_POSITION = 0,
  SIM_DATA_FIELD_BATTERY_LEVEL,
  SIM_DATA_FIELD_MAX
} sim_data_field_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern uint8_t sim_raw_data[SIM_RAW_RSP_SIZE];

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Initialize SIM module
 */
status_function_t bsp_sim_init(void);

/**
 * @brief  Reset SIM HTTP service
 */
status_function_t bsp_sim_reset_http(void);

// TODO: Add SIM power management functions here

#if (CONFIG_FIREBASE_SERVER == true)
/**
 * @brief  Send data to firebase
 * @param  data: Structure containing data to send
 *
 * @return Function status
 */
status_function_t bsp_sim_send_data_firebase(firebase_data_t *data);

/**
 * @brief Get raw string data from firebase
 */
status_function_t bsp_sim_get_raw_data_firebase(uint8_t *raw_data_buffer, uint16_t *size);

#elif (CONFIG_MQTT_SERVER == true)
// Not implemented yet
#endif

#endif /* End file _BSP_SIM_H_ */

/* End of file -------------------------------------------------------- */
