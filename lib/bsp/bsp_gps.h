/**
 * @file       bsp_gps.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-05
 * @author     Hai Tran
 *
 * @brief      BSP for NEO-M8N GPS module
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _BSP_GPS_H_
#define _BSP_GPS_H_

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"
#include "common_type.h"

#include <TinyGPSPlus.h>

/* Public defines ----------------------------------------------------- */
// Enable raw NMEA debug output (only printable ASCII)
// #define BSP_GPS_DEBUG_RAW_NMEA

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief GPS data structure parsed: position and time information
 */
typedef struct
{
  double   latitude;        // Latitude in degrees
  double   longitude;       // Longitude in degrees
  double   altitude;        // Altitude in meters
  double   speed_kmph;      // Speed in km/h
  double   course;          // Course in degrees
  uint32_t satellites;      // Number of satellites in use
  double   hdop;            // Horizontal Dilution of Precision
  uint16_t year;            // Year
  uint8_t  month;           // Month
  uint8_t  day;             // Day
  uint8_t  hour;            // Hour (UTC)
  uint8_t  minute;          // Minute
  uint8_t  second;          // Second
  bool     location_valid;  // True if location data is valid
  bool     time_valid;      // True if time data is valid
  bool     date_valid;      // True if date data is valid
} bsp_gps_data_t;

/**
 * @brief GPS callback function type
 */
typedef void (*bsp_gps_callback_t)(bsp_gps_data_t *data);

/**
 * @brief GPS sample rates configuration
 */
typedef enum
{
  BSP_GPS_UPDATE_1HZ  = 1000, /**< 1 Hz update rate */
  BSP_GPS_UPDATE_2HZ  = 500,  /**< 2 Hz update rate */
  BSP_GPS_UPDATE_5HZ  = 200,  /**< 5 Hz update rate */
  BSP_GPS_UPDATE_10HZ = 100   /**< 10 Hz update rate */
} bsp_gps_sample_rate_t;

/**
 * @brief GPS power mode configuration
 */
typedef enum
{
  BSP_GPS_POWER_FULL = 0,  // Full power mode - continuous tracking
  BSP_GPS_POWER_SAVE,      // Power save mode - reduced power consumption
  BSP_GPS_POWER_ECO        // Eco mode - balanced power/performance
} bsp_gps_power_mode_t;

/**
 * @brief GPS dynamic model configuration (platform)
 */
typedef enum
{
  BSP_GPS_DYN_PORTABLE = 0,  // Portable - general purpose
  BSP_GPS_DYN_STATIONARY,    // Stationary - no movement expected
  BSP_GPS_DYN_PEDESTRIAN,    // Pedestrian - walking dynamics
  BSP_GPS_DYN_AUTOMOTIVE,    // Automotive - car dynamics
  BSP_GPS_DYN_SEA,           // Sea - sea level, low dynamics
  BSP_GPS_DYN_AIRBORNE_1G,   // Airborne <1g acceleration
  BSP_GPS_DYN_AIRBORNE_2G,   // Airborne <2g acceleration
  BSP_GPS_DYN_AIRBORNE_4G    // Airborne <4g acceleration
} bsp_gps_dynamic_model_t;

/**
 * @brief GPS NMEA message types
 */
typedef enum
{
  BSP_GPS_NMEA_GGA = 0x01,  // GGA - Fix data
  BSP_GPS_NMEA_GLL = 0x02,  // GLL - Lat/Lon
  BSP_GPS_NMEA_GSA = 0x04,  // GSA - DOP and active satellites
  BSP_GPS_NMEA_GSV = 0x08,  // GSV - Satellites in view
  BSP_GPS_NMEA_RMC = 0x10,  // RMC - Recommended minimum data
  BSP_GPS_NMEA_VTG = 0x20,  // VTG - Track and ground speed
  BSP_GPS_NMEA_ALL = 0x3F   /**< All NMEA messages */
} bsp_gps_nmea_msg_t;

/**
 * @brief GPS configuration structure
 */
typedef struct
{
  uart_port_t        uart_port; /**< UART port for GPS communication */
  int                tx_pin;    /**< TX pin number */
  int                rx_pin;    /**< RX pin number */
  int                baudrate;  /**< UART baudrate (default: 9600) */
  bsp_gps_callback_t callback;  /**< Callback when valid position received */
} bsp_gps_config_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize GPS module
 * @param callback Callback function when valid position received (can be NULL)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_init(bsp_gps_callback_t callback);

/**
 * @brief Deinitialize GPS module
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_deinit(void);

/**
 * @brief Get latest GPS data
 * @param data Pointer to GPS data structure to fill
 * @return STATUS_OK if valid data available, STATUS_ERROR otherwise
 */
status_function_t bsp_gps_get_data(bsp_gps_data_t *data);

/**
 * @brief Check if GPS has valid fix
 * @return true if GPS has valid location fix
 */
bool bsp_gps_is_fixed(void);

/**
 * @brief Get number of satellites in view
 * @return Number of satellites
 */
uint32_t bsp_gps_get_satellites(void);

/**
 * @brief Set GPS update rate
 */
status_function_t bsp_gps_set_new_sample_rate(bsp_gps_sample_rate_t rate);

/**
 * @brief Set GPS power mode
 */
status_function_t bsp_gps_set_power_mode(bsp_gps_power_mode_t mode);

/**
 * @brief Set GPS dynamic model (platform type)
 * @param model Dynamic model enum value
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_set_dynamic_model(bsp_gps_dynamic_model_t model);

/**
 * @brief Configure NMEA message output
 * @param msg_mask Bitmask of NMEA messages to enable (see bsp_gps_nmea_msg_t)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_configure_nmea(uint8_t msg_mask);

/**
 * @brief Set GPS baudrate
 * @param baudrate New baudrate (9600, 19200, 38400, 57600, 115200)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_set_baudrate(uint32_t baudrate);

/**
 * @brief Reset GPS module to factory defaults
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_reset_default(void);

/**
 * @brief Perform cold start (clear all data)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_cold_start(void);

/**
 * @brief Perform warm start (keep ephemeris)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_warm_start(void);

/**
 * @brief Perform hot start (keep all data)
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_hot_start(void);

/**
 * @brief Save current configuration to GPS flash memory
 * @return STATUS_OK on success, STATUS_ERROR on failure
 */
status_function_t bsp_gps_save_config(void);

/**
 * @brief Get TinyGPSPlus instance for advanced usage
 * @return Pointer to TinyGPSPlus instance
 */
TinyGPSPlus *bsp_gps_get_instance(void);

#endif /*End file _BSP_GPS_H_*/

/* End of file -------------------------------------------------------- */
