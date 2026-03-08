/**
 * @file       log_service.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Basic logging service - provides core logging API and macros
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _LOG_SERVICE_H_
#define _LOG_SERVICE_H_

/* Includes ----------------------------------------------------------- */
#include "common_type.h"

#include <stdarg.h>

/* Public defines ----------------------------------------------------- */
#define LOG_ENABLE        (1)
#define LOG_USB_ENABLE    (1)
#define LOG_SDCARD_ENABLE (0)

/**
 * @brief Serial baudrate
 */
#define LOG_SERIAL_BAUD   (115200)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Log level enumeration
 */
typedef enum
{
  LOG_LEVEL_NONE = 0,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_WARN,
  LOG_LEVEL_INFO,
  LOG_LEVEL_DBG,
  LOG_LEVEL_MAX
} log_level_t;

/**
 * @brief External log handler callback type
 */
typedef void (*log_handler_t)(const char *message, size_t len);

/* Public macros ------------------------------------------------------ */
#define LOG_MODULE_REGISTER(name, level)             \
  static const char       *LOG_TAG          = #name; \
  static const log_level_t LOG_MODULE_LEVEL = level;

#if LOG_ENABLE

#define LOG_ERR(fmt, ...)                                              \
  do                                                                   \
  {                                                                    \
    if (LOG_MODULE_LEVEL >= LOG_LEVEL_ERROR)                           \
      log_service_print(LOG_LEVEL_ERROR, LOG_TAG, fmt, ##__VA_ARGS__); \
  } while (0)
#define LOG_WRN(fmt, ...)                                             \
  do                                                                  \
  {                                                                   \
    if (LOG_MODULE_LEVEL >= LOG_LEVEL_WARN)                           \
      log_service_print(LOG_LEVEL_WARN, LOG_TAG, fmt, ##__VA_ARGS__); \
  } while (0)
#define LOG_INF(fmt, ...)                                             \
  do                                                                  \
  {                                                                   \
    if (LOG_MODULE_LEVEL >= LOG_LEVEL_INFO)                           \
      log_service_print(LOG_LEVEL_INFO, LOG_TAG, fmt, ##__VA_ARGS__); \
  } while (0)
#define LOG_DBG(fmt, ...)                                            \
  do                                                                 \
  {                                                                  \
    if (LOG_MODULE_LEVEL >= LOG_LEVEL_DBG)                           \
      log_service_print(LOG_LEVEL_DBG, LOG_TAG, fmt, ##__VA_ARGS__); \
  } while (0)
#else

#define LOG_ERR(fmt, ...)  // NULL
#define LOG_WRN(fmt, ...)  // NULL
#define LOG_INF(fmt, ...)  // NULL
#define LOG_DBG(fmt, ...)  // NULL

#endif  // LOG_ENABLE

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize log service (no parameters)
 */
void log_service_init(void);

/**
 * @brief Print log message
 * @note Use macros LOG instead of calling this function directly.
 */
void log_service_print(log_level_t level, const char *tag, const char *fmt, ...);

/**
 * @brief Register external log handler (for SD card logging)
 *
 * @param[in] handler  Callback function
 */
void log_service_register_handler(log_handler_t handler);

#endif /*End file _LOG_SERVICE_H_*/

/* End of file -------------------------------------------------------- */
