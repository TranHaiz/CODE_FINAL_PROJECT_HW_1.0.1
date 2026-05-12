/**
 * @file       sys_network_adapter_lte.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    3.0.0
 * @date       2026-05-12
 * @author     Hai Tran
 *
 * @brief      LTE/SIM network adapter — SIM + MQTT state machine.
 *             Buffer/SD/data-task logic lives in sys_network (dispatcher).
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_network_adapter_lte.h"

#include "bsp_sim.h"
#include "device_config.h"
#include "device_info.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_cmd.h"
#include "sys_led.h"
#include "sys_manager.h"

#include <string.h>

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(net_lte, LOG_LEVEL_SYS_NETWORK)

#define MQTT_KEEPALIVE_S            (30)
#define MQTT_KEEPALIVE_MS           (MQTT_KEEPALIVE_S * 1000UL)
#define MQTT_PUBLISH_RETRY          (3)
#define MQTT_PUBLISH_RETRY_DELAY_MS (100)
#define MQTT_MESSAGE_MAX_LEN        (1024)
#define MQTT_INIT_TIMEOUT_MS        (15000)

#define OFFLINE_POLL_MS             (100)
#define ONLINE_POLL_MS              (500)
#define ONLINE_LOCKED_POLL_MS       (2000)
#define ONLINE_IDLE_POLL_MS         (10000)
#define SIM_READY_TIMEOUT_MS        (10000)
#define SIM_HARD_RESET_DELAY_MS     (2000)
#define BACKOFF_BASE_MS             (2000)
#define BACKOFF_MAX_MS              (32000)
#define RETRY_MAX_BEFORE_RESET      (3)
#define NETWORK_LOST_MAX_COUNT      (10)

#define NETWORK_KEEPALIVE_MES       "KEEPALIVE"

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  NETWORK_STATE_SIM_INIT = 0,
  NETWORK_STATE_SIM_WAIT_READY,
  NETWORK_STATE_MQTT_INIT,
  NETWORK_STATE_ONLINE,
  NETWORK_STATE_ERROR,
  NETWORK_STATE_SIM_RESET,
} net_state_t;

typedef struct
{
  net_state_t state;
  net_state_t prev_state;
  uint8_t     retry_count;
  size_t      state_enter_ms;
  size_t      last_keepalive_ms;
  size_t      last_poll_ms;
  size_t      last_log_ms;
  uint8_t     network_lost_count;
  bool        sim_ready;
  bool        mqtt_ready;
  bool        is_init;
} lte_ctx_t;

/* Private macros ----------------------------------------------------- */
#define COUNT_MS(since_ms) ((size_t) (OS_GET_TICK() - (since_ms)))

/* Private variables -------------------------------------------------- */
static lte_ctx_t             lte_ctx;
static net_incoming_cb_t     lte_incoming_cb = NULL;

OS_MUTEX_DEFINE_STATIC(lte_publish_mutex);

/* Private function prototypes ---------------------------------------- */
static void              lte_change_state(net_state_t new_state);
static size_t            lte_backoff(uint8_t retry);
static void              lte_run_sim_init(void);
static void              lte_run_sim_wait_ready(void);
static void              lte_run_mqtt_init(void);
static void              lte_run_online(void);
static void              lte_run_error_backoff(void);
static void              lte_run_sim_hard_reset(void);
static void              lte_process_active(void);
static status_function_t lte_adapter_init(net_incoming_cb_t on_incoming);
static bool              lte_adapter_is_ready(void);
static status_function_t lte_adapter_publish(net_channel_t ch, const uint8_t *data, size_t len);
static void              lte_adapter_poll(void);

/* Public adapter instance -------------------------------------------- */
net_adapter_t g_net_adapter_lte = {
  .name     = "LTE",
  .init     = lte_adapter_init,
  .is_ready = lte_adapter_is_ready,
  .publish  = lte_adapter_publish,
  .poll     = lte_adapter_poll,
};

/* Function definitions ----------------------------------------------- */
void sys_network_adapter_lte_task(void *param)
{
  while (1)
  {
    lte_adapter_poll();
  }
}

void sys_network_adapter_lte_mqtt_cb(const char *topic, const uint8_t *data, size_t len)
{
  LOG_DBG("MQTT rx [%s]: %d bytes", topic, (int) len);
  if (data == NULL || len == 0 || len >= CMD_INPUT_MAX_LEN)
  {
    LOG_WRN("Invalid command payload: %d bytes", (int) len);
    return;
  }
  if (lte_incoming_cb != NULL)
  {
    lte_incoming_cb(data, len);
  }
}

/* Private definitions ----------------------------------------------- */
static status_function_t lte_adapter_init(net_incoming_cb_t on_incoming)
{
  lte_incoming_cb = on_incoming;
  memset(&lte_ctx, 0, sizeof(lte_ctx));
  lte_ctx.state      = NETWORK_STATE_SIM_INIT;
  lte_ctx.prev_state = NETWORK_STATE_SIM_INIT;
  OS_MUTEX_CREATE(lte_publish_mutex);
  lte_ctx.is_init = true;
  return STATUS_OK;
}

static bool lte_adapter_is_ready(void)
{
  return (lte_ctx.state == NETWORK_STATE_ONLINE);
}

static status_function_t lte_adapter_publish(net_channel_t ch, const uint8_t *data, size_t len)
{
  if (!lte_adapter_is_ready() || data == NULL || len == 0 || len >= MQTT_MESSAGE_MAX_LEN)
  {
    return STATUS_ERROR;
  }

  const char *topic;
  switch (ch)
  {
  case NET_CH_DATA:     topic = g_device_info.mqtt_data_topic; break;
  case NET_CH_NOTI:
  case NET_CH_CMD_RESP: topic = g_device_info.mqtt_noti_topic; break;
  default:              return STATUS_ERROR;
  }

  char payload_str[MQTT_MESSAGE_MAX_LEN];
  memcpy(payload_str, data, len);
  payload_str[len] = '\0';

  mqtt_message_t mes = {
    .topic   = topic,
    .payload = payload_str,
  };

  bool ok = false;
  OS_MUTEX_LOCK(lte_publish_mutex);
  for (uint8_t i = 0; i < MQTT_PUBLISH_RETRY; i++)
  {
    if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
    {
      ok = true;
      break;
    }
    OS_DELAY_MS(MQTT_PUBLISH_RETRY_DELAY_MS);
  }
  OS_MUTEX_UNLOCK(lte_publish_mutex);

  if (!ok)
  {
    LOG_WRN("Publish failed via LTE — triggering reconnect");
    lte_change_state(NETWORK_STATE_ERROR);
    return STATUS_ERROR;
  }
  return STATUS_OK;
}

static void lte_adapter_poll(void)
{
  if (!lte_ctx.is_init)
  {
    OS_DELAY_MS(OFFLINE_POLL_MS);
    return;
  }

  switch (g_device_info.nvs_info.curr_state)
  {
  case DEVICE_STATE_LOCKED:
  case DEVICE_STATE_ACTIVE:
  case DEVICE_STATE_IDLE: lte_process_active(); break;
  default:                OS_DELAY_MS(OFFLINE_POLL_MS); break;
  }
}

static void lte_process_active(void)
{
  switch (lte_ctx.state)
  {
  case NETWORK_STATE_SIM_INIT:       lte_run_sim_init(); break;
  case NETWORK_STATE_SIM_WAIT_READY: lte_run_sim_wait_ready(); break;
  case NETWORK_STATE_MQTT_INIT:      lte_run_mqtt_init(); break;
  case NETWORK_STATE_ONLINE:
    if (lte_ctx.network_lost_count)
    {
      lte_ctx.network_lost_count = 0;
      sys_led_clear_event(SYS_LED_EVT_ERROR_NETWORK_LOST);
      LOG_DBG("LTE: network_lost_count reset");
    }
    lte_run_online();
    break;
  case NETWORK_STATE_ERROR:
    lte_ctx.network_lost_count++;
    if (lte_ctx.network_lost_count >= NETWORK_LOST_MAX_COUNT)
    {
      sys_led_write_event(SYS_LED_EVT_ERROR_NETWORK_LOST);
    }
    lte_run_error_backoff();
    break;
  case NETWORK_STATE_SIM_RESET: lte_run_sim_hard_reset(); break;
  default:
    LOG_ERR("Unknown LTE state %d — resetting", lte_ctx.state);
    lte_change_state(NETWORK_STATE_SIM_INIT);
    break;
  }

  size_t delay_ms;
  if (lte_ctx.state != NETWORK_STATE_ONLINE)
  {
    delay_ms = OFFLINE_POLL_MS;
  }
  else if (g_device_info.nvs_info.curr_state == DEVICE_STATE_LOCKED)
  {
    delay_ms = ONLINE_LOCKED_POLL_MS;
  }
  else if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    delay_ms = ONLINE_IDLE_POLL_MS;
  }
  else
  {
    delay_ms = ONLINE_POLL_MS;
  }
  OS_DELAY_MS(delay_ms);
}

static void lte_change_state(net_state_t new_state)
{
  LOG_DBG("LTE: %d → %d  (retry=%d)", lte_ctx.state, new_state, lte_ctx.retry_count);
  lte_ctx.prev_state     = lte_ctx.state;
  lte_ctx.state          = new_state;
  lte_ctx.state_enter_ms = OS_GET_TICK();
  lte_ctx.last_poll_ms   = 0;
  lte_ctx.last_log_ms    = 0;
}

static size_t lte_backoff(uint8_t retry)
{
  uint8_t shift   = (retry < 4) ? retry : 4;
  size_t  backoff = (size_t) BACKOFF_BASE_MS << shift;
  return (backoff > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : backoff;
}

static void lte_run_sim_init(void)
{
  LOG_DBG("LTE: SIM init");
  lte_ctx.sim_ready  = false;
  lte_ctx.mqtt_ready = false;

  if (bsp_sim_init() != STATUS_OK)
  {
    LOG_ERR("SIM init failed");
    lte_change_state(NETWORK_STATE_ERROR);
    sys_led_write_event(SYS_LED_EVT_ERROR_SIM);
    return;
  }
  lte_change_state(NETWORK_STATE_MQTT_INIT);
}

static void lte_run_sim_wait_ready(void)
{
  if (COUNT_MS(lte_ctx.state_enter_ms) >= SIM_READY_TIMEOUT_MS)
  {
    LOG_WRN("SIM not ready after %d ms", SIM_READY_TIMEOUT_MS);
    lte_change_state(NETWORK_STATE_ERROR);
    return;
  }

  if (COUNT_MS(lte_ctx.last_poll_ms) < OFFLINE_POLL_MS)
  {
    return;
  }
  lte_ctx.last_poll_ms = OS_GET_TICK();

  if (bsp_sim_is_ready())
  {
    LOG_DBG("SIM ready");
    lte_ctx.sim_ready = true;
    lte_change_state(NETWORK_STATE_MQTT_INIT);
    sys_led_clear_event(SYS_LED_EVT_ERROR_SIM);
  }
  else
  {
    LOG_WRN("SIM not ready (%d / %d ms)", COUNT_MS(lte_ctx.state_enter_ms), SIM_READY_TIMEOUT_MS);
  }
}

static void lte_run_mqtt_init(void)
{
  if (COUNT_MS(lte_ctx.state_enter_ms) >= MQTT_INIT_TIMEOUT_MS)
  {
    LOG_WRN("MQTT init timeout");
    lte_change_state(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_init() != STATUS_OK)
  {
    LOG_WRN("MQTT init failed");
    lte_change_state(NETWORK_STATE_ERROR);
    return;
  }

  if (bsp_sim_mqtt_sub(g_device_info.mqtt_cmd_topic, sys_network_adapter_lte_mqtt_cb) != STATUS_OK)
  {
    LOG_WRN("MQTT subscribe failed");
    lte_change_state(NETWORK_STATE_ERROR);
    return;
  }

  LOG_INF("MQTT connected");
  lte_ctx.mqtt_ready        = true;
  lte_ctx.retry_count       = 0;
  lte_ctx.last_keepalive_ms = OS_GET_TICK();
  lte_change_state(NETWORK_STATE_ONLINE);
}

static void lte_run_online(void)
{
  /* Keepalive when device is not actively sending telemetry */
  if (g_device_info.nvs_info.curr_state != DEVICE_STATE_ACTIVE)
  {
    if (COUNT_MS(lte_ctx.last_keepalive_ms) >= MQTT_KEEPALIVE_MS)
    {
      if (!bsp_sim_is_ready())
      {
        LOG_WRN("Keepalive: SIM lost");
        lte_change_state(NETWORK_STATE_ERROR);
        return;
      }

      mqtt_message_t mes = {
        .topic   = g_device_info.mqtt_noti_topic,
        .payload = NETWORK_KEEPALIVE_MES,
      };

      bool ok = false;
      OS_MUTEX_LOCK(lte_publish_mutex);
      for (uint8_t i = 0; i < MQTT_PUBLISH_RETRY; i++)
      {
        if (bsp_sim_mqtt_pub(&mes) == STATUS_OK)
        {
          ok = true;
          break;
        }
        OS_DELAY_MS(MQTT_PUBLISH_RETRY_DELAY_MS);
      }
      OS_MUTEX_UNLOCK(lte_publish_mutex);

      if (!ok)
      {
        LOG_WRN("Keepalive failed");
        lte_change_state(NETWORK_STATE_ERROR);
        return;
      }
      lte_ctx.last_keepalive_ms = OS_GET_TICK();
    }
  }
}

static void lte_run_error_backoff(void)
{
  size_t backoff = lte_backoff(lte_ctx.retry_count);
  size_t elapsed = COUNT_MS(lte_ctx.state_enter_ms);

  if (elapsed < backoff)
  {
    if (COUNT_MS(lte_ctx.last_log_ms) >= 5000)
    {
      LOG_DBG("LTE backoff — waiting %u ms (elapsed %u ms)", (unsigned) backoff, (unsigned) elapsed);
      lte_ctx.last_log_ms = OS_GET_TICK();
    }
    return;
  }

  lte_ctx.retry_count++;
  if (lte_ctx.retry_count >= RETRY_MAX_BEFORE_RESET)
  {
    LOG_WRN("Max retries (%d) — hard SIM reset", lte_ctx.retry_count);
    lte_ctx.retry_count = 0;
    lte_change_state(NETWORK_STATE_SIM_RESET);
  }
  else
  {
    LOG_DBG("Soft retry %d", lte_ctx.retry_count);
    lte_change_state(NETWORK_STATE_SIM_WAIT_READY);
  }
}

static void lte_run_sim_hard_reset(void)
{
  if (COUNT_MS(lte_ctx.state_enter_ms) < 10)
  {
    // TODO: Hardware reset via MOSFET power control
  }
  if (COUNT_MS(lte_ctx.state_enter_ms) >= SIM_HARD_RESET_DELAY_MS)
  {
    lte_change_state(NETWORK_STATE_SIM_INIT);
  }
}

/* End of file -------------------------------------------------------- */
