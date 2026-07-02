/**
 * @file       sys_error.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-05-11
 * @author     Hai Tran
 *
 * @brief      System error manager implementation
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_error.h"

#include "bsp_device.h"
#include "bsp_error.h"
#include "bsp_sim.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_led.h"
#include "sys_network.h"
#include "sys_network_adapter_ble.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_error, LOG_LEVEL_SYS_ERROR);

#define SYS_ERROR_BROADCAST_INTERVAL_MS (15000)  // cadence for log/BLE/MQTT error broadcast
#define SYS_ERROR_SIM_READY_TIMEOUT_MS  (15000)  // give up SIM bring-up after this, keep BLE/USB alive
#define SYS_ERROR_NET_START_DELAY_MS    (3000)   // let the LED light/animate before the blocking SIM bring-up

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  ERR_NET_SIM_INIT = 0,
  ERR_NET_SIM_WAIT,
  ERR_NET_MQTT_INIT,
  ERR_NET_ONLINE,
  ERR_NET_DISABLED,
} err_net_state_t;

typedef struct
{
  size_t          start_ms;
  size_t          last_broadcast_ms;
  err_net_state_t net_state;
  size_t          sim_wait_start_ms;
  device_error_t  code;
  char            noti_topic[MQTT_MAX_TOPIC_LEN];
  char            payload[32];  // "NOTI_DEVICE_ERROR=<code>"
} sys_error_handler_t;

/* Private variables -------------------------------------------------- */
static sys_error_handler_t g_sys_error_handler;

/* Private function prototypes ---------------------------------------- */
static const char *sys_error_code_str(device_error_t code);
static void        sys_error_net_process(size_t now_ms);
static void        sys_error_broadcast(void);

/* Function definitions ----------------------------------------------- */
void sys_error_init(void)
{
  memset(&g_sys_error_handler, 0, sizeof(g_sys_error_handler));
  g_sys_error_handler.code = (device_error_t) g_device_info.last_error_code;

  snprintf(g_device_info.device_name, sizeof(g_device_info.device_name), "haq-trk-%03u",
           g_device_info.nvs_info.device_id);
  snprintf(g_sys_error_handler.noti_topic, sizeof(g_sys_error_handler.noti_topic), "%s/noti",
           g_device_info.device_name);
  snprintf(g_sys_error_handler.payload, sizeof(g_sys_error_handler.payload), NETWORK_NOTI_DEVICE_ERROR_PREFIX "%u",
           (unsigned) g_sys_error_handler.code);

  sys_network_adapter_ble_init();
  sys_led_init();
  sys_led_write_event(sys_led_evt_from_error(g_sys_error_handler.code));

  LOG_ERR("ENTER ERROR MODE: code=%u (%s)", (unsigned) g_sys_error_handler.code,
          sys_error_code_str(g_sys_error_handler.code));
  sys_error_broadcast();
  size_t now                            = OS_GET_TICK();
  g_sys_error_handler.start_ms          = now;
  g_sys_error_handler.last_broadcast_ms = now;
  g_sys_error_handler.net_state         = ERR_NET_SIM_INIT;
}

void sys_error_process(void)
{
  size_t now_ms = OS_GET_TICK();

  // Service the LED + BLE first so the error indicator is alive regardless of the SIM bring-up,
  // which blocks for seconds inside bsp_sim_init while detecting/talking to the modem.
  sys_led_process();
  sys_network_adapter_ble_process();

  if (now_ms - g_sys_error_handler.last_broadcast_ms >= SYS_ERROR_BROADCAST_INTERVAL_MS)
  {
    g_sys_error_handler.last_broadcast_ms = now_ms;
    sys_error_broadcast();
  }

  // Delay the (blocking) SIM bring-up so the LED has lit and latched before the loop stalls.
  if (now_ms - g_sys_error_handler.start_ms >= SYS_ERROR_NET_START_DELAY_MS)
  {
    sys_error_net_process(now_ms);
  }
}

void sys_error_notify(device_error_t code)
{
  char payload[32];
  int  len = snprintf(payload, sizeof(payload), NETWORK_NOTI_DEVICE_ERROR_PREFIX "%u", (unsigned) code);

  LOG_ERR("Device error: code=%u (%s)", (unsigned) code, sys_error_code_str(code));
  sys_led_write_event(sys_led_evt_from_error(code));
  sys_network_publish_noti(payload, (size_t) len);
}

/* Private definitions ----------------------------------------------- */
static const char *sys_error_code_str(device_error_t code)
{
  switch (code)
  {
  case DEVICE_ERROR_SD_INIT: return "SD_INIT";
  case DEVICE_ERROR_SD_MOUNT: return "SD_MOUNT";
  case DEVICE_ERROR_SD_MKDIR: return "SD_MKDIR";
  case DEVICE_ERROR_SD_OPEN_FILE: return "SD_OPEN_FILE";
  case DEVICE_ERROR_SIM_INIT: return "SIM_INIT";
  case DEVICE_ERROR_FUEL_GAUGE_INIT: return "FUEL_GAUGE_INIT";
  case DEVICE_ERROR_TEMP_HUM_INIT: return "TEMP_HUM_INIT";
  case DEVICE_ERROR_IMU_INIT: return "IMU_INIT";
  case DEVICE_ERROR_COMPASS_INIT: return "COMPASS_INIT";
  case DEVICE_ERROR_DISPLAY_INIT: return "DISPLAY_INIT";
  case DEVICE_ERROR_GPS_INIT: return "GPS_INIT";
  default: return "UNKNOWN";
  }
}

// Print the error over USB (Serial) and, when connected, BLE NUS (log_service streams both).
// Also publish NOTI_ERROR=<code> over MQTT when the modem made it online.
static void sys_error_broadcast(void)
{
  LOG_ERR("Device in ERROR: code=%u (%s)", (unsigned) g_sys_error_handler.code,
          sys_error_code_str(g_sys_error_handler.code));

#if (CONFIG_MQTT_SERVER == true)
  if (g_sys_error_handler.net_state == ERR_NET_ONLINE)
  {
    mqtt_message_t msg = { g_sys_error_handler.noti_topic, g_sys_error_handler.payload };
    bsp_sim_mqtt_pub(&msg);
  }
#endif
}

// Best-effort SIM/MQTT bring-up. Each step runs once; if anything fails we settle on
// ERR_NET_DISABLED so the modem never starves the BLE/USB error reporting.
static void sys_error_net_process(size_t now_ms)
{
#if (CONFIG_MQTT_SERVER == true)
  switch (g_sys_error_handler.net_state)
  {
  case ERR_NET_SIM_INIT:
    if (bsp_sim_init() == STATUS_OK)
    {
      g_sys_error_handler.sim_wait_start_ms = now_ms;
      g_sys_error_handler.net_state         = ERR_NET_SIM_WAIT;
    }
    else
    {
      g_sys_error_handler.net_state = ERR_NET_DISABLED;
    }
    break;

  case ERR_NET_SIM_WAIT:
    if (bsp_sim_is_ready())
    {
      g_sys_error_handler.net_state = ERR_NET_MQTT_INIT;
    }
    else if (now_ms - g_sys_error_handler.sim_wait_start_ms >= SYS_ERROR_SIM_READY_TIMEOUT_MS)
    {
      g_sys_error_handler.net_state = ERR_NET_DISABLED;
    }
    break;

  case ERR_NET_MQTT_INIT:
    g_sys_error_handler.net_state = (bsp_sim_mqtt_init() == STATUS_OK) ? ERR_NET_ONLINE : ERR_NET_DISABLED;
    if (g_sys_error_handler.net_state == ERR_NET_ONLINE)
      LOG_INF("Error mode: modem online, publishing NOTI_ERROR");
    break;

  case ERR_NET_ONLINE:
  case ERR_NET_DISABLED:
  default: break;
  }
#else
  (void) now_ms;
  g_sys_error_handler.net_state = ERR_NET_DISABLED;
#endif
}

/* End of file -------------------------------------------------------- */
