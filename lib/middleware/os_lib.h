/**
 * @file       os_lib.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief     Operating System abstraction layer
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __OS_LIB_H_
#define __OS_LIB_H_

/* Includes ----------------------------------------------------------- */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <Arduino.h>

/* Public defines ----------------------------------------------------- */
#define CONFIG_FREE_RTOS

#if defined(CONFIG_FREE_RTOS)

/* Mapping CMSIS-like names to Native FreeRTOS */
#define OS_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define OS_GET_TICK()   xTaskGetTickCount()
#define OS_YIELD()      taskYIELD()
#define OS_MAX_DELAY    portMAX_DELAY

/**
 * @brief  Declare thread
 * ESP32 dùng TaskHandle_t thay vì osThreadId_t
 */
#define OS_THREAD_DECLARE(thread_name, priority, stack_size) \
  TaskHandle_t      thread_name##_handle   = NULL;           \
  const uint32_t    thread_name##_stack    = (stack_size);   \
  const UBaseType_t thread_name##_priority = (priority)

/**
 * @brief Create and start thread
 * Lưu ý: Ưu tiên dùng xTaskCreatePinnedToCore trên ESP32 để cố định Core (mặc định Core 1)
 */
#define OS_THREAD_CREATE(thread_name, func)                                                        \
  xTaskCreatePinnedToCore((func), #thread_name, thread_name##_stack, NULL, thread_name##_priority, \
                          &thread_name##_handle, 1)

/* --- Semaphore Helpers (Giữ nguyên logic Static của bạn) --- */

#define OS_SEM_DEFINE_STATIC(name)                   \
  static SemaphoreHandle_t name##_sem_handle = NULL; \
  static StaticSemaphore_t name##_sem_buffer

#define OS_SEM_DEFINE_GLOBAL(name)            \
  SemaphoreHandle_t name##_sem_handle = NULL; \
  StaticSemaphore_t name##_sem_buffer

#define OS_SEM_DECLARE(name)                  \
  extern SemaphoreHandle_t name##_sem_handle; \
  extern StaticSemaphore_t name##_sem_buffer

#define OS_SEM_CREATE(name)                                                 \
  do                                                                        \
  {                                                                         \
    if (name##_sem_handle == NULL)                                          \
    {                                                                       \
      name##_sem_handle = xSemaphoreCreateBinaryStatic(&name##_sem_buffer); \
    }                                                                       \
  } while (0)

#define OS_SEM_GIVE_FROM_ISR(name)                                       \
  do                                                                     \
  {                                                                      \
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;                       \
    xSemaphoreGiveFromISR(name##_sem_handle, &xHigherPriorityTaskWoken); \
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                        \
  } while (0)

#define OS_SEM_GIVE(name) xSemaphoreGive(name##_sem_handle)

#define OS_SEM_TAKE(name, timeout_ms) \
  xSemaphoreTake(name##_sem_handle, (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms))

#define OS_SEM_DELETE(name)                \
  do                                       \
  {                                        \
    if (name##_sem_handle != NULL)         \
    {                                      \
      vSemaphoreDelete(name##_sem_handle); \
      name##_sem_handle = NULL;            \
    }                                      \
  } while (0)

/* --- Mutex Helpers (Chuyển từ CMSIS sang Native) --- */

#define OS_MUTEX_DEFINE_STATIC(name) static SemaphoreHandle_t name##_mutex = NULL
#define OS_MUTEX_DEFINE_GLOBAL(name) SemaphoreHandle_t name##_mutex = NULL

#define OS_MUTEX_CREATE(name)                 \
  do                                          \
  {                                           \
    if (name##_mutex == NULL)                 \
    {                                         \
      name##_mutex = xSemaphoreCreateMutex(); \
    }                                         \
  } while (0)

#define OS_MUTEX_ACQUIRE(handle, timeout_ms) \
  xSemaphoreTake((handle), (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms))

#define OS_MUTEX_RELEASE(handle) xSemaphoreGive((handle))
#define OS_MUTEX_DELETE(handle)  vSemaphoreDelete((handle))

#endif /* CONFIG_FREE_RTOS */

#endif /* __OS_LIB_H_ */