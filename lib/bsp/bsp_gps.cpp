/**
 * @file       bsp_gps.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      BSP driver for NEO-M8N GPS module using TinyGPSPlus
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_gps.h"

#include "device_info.h"
#include "os_lib.h"

/* Private defines ---------------------------------------------------- */
#define GPS_UBX_SYNC1     (0xB5)
#define GPS_UBX_SYNC2     (0x62)
#define GPS_UBX_CLASS_CFG (0x06)
#define GPS_UBX_CLASS_NAV (0x01)

#define GPS_UBX_CFG_PRT   (0x00)  // Port configuration
#define GPS_UBX_CFG_MSG   (0x01)  // Message configuration
#define GPS_UBX_CFG_RST   (0x04)  // Reset
#define GPS_UBX_CFG_RATE  (0x08)  // Navigation/measurement rate
#define GPS_UBX_CFG_CFG   (0x09)  // Save/load configuration
#define GPS_UBX_CFG_NAV5  (0x24)  // Navigation engine settings
#define GPS_UBX_CFG_PM2   (0x3B)  // Power management
#define GPS_UBX_CFG_RXM   (0x11)  // RX manager configuration

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static TinyGPSPlus        gps_handler;
static bsp_gps_callback_t gps_callback;
static bsp_gps_data_t     gps_data;
static volatile bool      is_gps_initialized = false;

/* Private function prototypes ---------------------------------------- */
/**
 * @brief UART callback for GPS data reception
 *
 * @param[in] uart_num UART handler number
 * @param[in] data Received data buffer
 * @param[in] len Length of received data
 *
 * @return none
 */
static void gps_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len);

/**
 * @brief Update GPS data buffer
 */
static void gps_update_data_buffer(void);

/**
 * @brief Send UBX protocol command to GPS
 *
 * @param[in] msg_class UBX message class
 * @param[in] msg_id UBX message ID
 * @param[in] payload Payload data
 * @param[in] payload_len Payload length
 *
 * @return none
 */
static void gps_send_ubx_command(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);

/**
 * @brief Calculate UBX checksum (Fletcher algorithm)
 * @param buffer Data buffer
 * @param len Data length
 * @param ck_a Pointer to checksum A
 * @param ck_b Pointer to checksum B
 *
 * @return none
 */
static void gps_calculate_checksum(uint8_t *buffer, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_gps_init(bsp_gps_callback_t callback)
{
  if (callback != NULL)
    gps_callback = callback;

  bsp_uart_config_t uart_cfg = { .port     = GPS_UART_HANDLER,
                                 .tx_pin   = GPS_UART_TX,
                                 .rx_pin   = GPS_UART_RX,
                                 .baudrate = GPS_UART_BAUDRATE,
                                 .callback = gps_uart_callback };

  bsp_uart_init(&uart_cfg);

  memset(&gps_data, 0, sizeof(gps_data));

  is_gps_initialized = true;
  return STATUS_OK;
}

status_function_t bsp_gps_deinit(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  is_gps_initialized = false;
  memset(&gps_data, 0, sizeof(gps_data));

  return STATUS_OK;
}

status_function_t bsp_gps_get_data(bsp_gps_data_t *data)
{
  assert(data != NULL);
  assert(is_gps_initialized);

  *data = gps_data;

  if (!gps_data.location_valid)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

bool bsp_gps_is_fixed(void)
{
  return gps_handler.location.isValid() && gps_handler.location.age() < 5000;
}

size_t bsp_gps_get_satellites(void)
{
  if (gps_handler.satellites.isValid())
  {
    return gps_handler.satellites.value();
  }
  return 0;
}

status_function_t bsp_gps_set_new_sample_rate(bsp_gps_sample_rate_t rate)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // measRate(2), navRate(2), timeRef(2)
  uint8_t  payload[6];
  uint16_t meas_rate = (uint16_t) rate;

  payload[0] = meas_rate & 0xFF;
  payload[1] = (meas_rate >> 8) & 0xFF;
  payload[2] = 0x01;
  payload[3] = 0x00;
  payload[4] = 0x01;
  payload[5] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RATE, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_set_power_mode(bsp_gps_power_mode_t mode)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RXM payload
  uint8_t payload[2] = { 0x00, 0x00 };

  switch (mode)
  {
  case BSP_GPS_POWER_FULL: payload[1] = 0x00; break;
  case BSP_GPS_POWER_SAVE: payload[1] = 0x01; break;
  case BSP_GPS_POWER_ECO: payload[1] = 0x04; break;
  default: return STATUS_ERROR;
  }

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RXM, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_set_dynamic_model(bsp_gps_dynamic_model_t model)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[36];
  memset(payload, 0, sizeof(payload));
  payload[0] = 0x01;
  payload[1] = 0x00;
  switch (model)
  {
  case BSP_GPS_DYN_PORTABLE: payload[2] = 0; break;
  case BSP_GPS_DYN_STATIONARY: payload[2] = 2; break;
  case BSP_GPS_DYN_PEDESTRIAN: payload[2] = 3; break;
  case BSP_GPS_DYN_AUTOMOTIVE: payload[2] = 4; break;
  case BSP_GPS_DYN_SEA: payload[2] = 5; break;
  case BSP_GPS_DYN_AIRBORNE_1G: payload[2] = 6; break;
  case BSP_GPS_DYN_AIRBORNE_2G: payload[2] = 7; break;
  case BSP_GPS_DYN_AIRBORNE_4G: payload[2] = 8; break;
  default: return STATUS_ERROR;
  }

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_NAV5, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_configure_nmea(uint8_t msg_mask)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  const uint8_t nmea_msg_ids[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

  for (int i = 0; i < 6; i++)
  {
    uint8_t rate = (msg_mask & (1 << i)) ? 1 : 0;
    uint8_t payload[8];

    payload[0] = 0xF0;             // NMEA class
    payload[1] = nmea_msg_ids[i];  // Message ID
    payload[2] = 0;                // DDC rate
    payload[3] = rate;             // UART1 rate
    payload[4] = 0;                // UART2 rate
    payload[5] = 0;                // USB rate
    payload[6] = 0;                // SPI rate
    payload[7] = 0;                // Reserved

    gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_MSG, payload, sizeof(payload));
    OS_DELAY_MS(50);
  }

  return STATUS_OK;
}

status_function_t bsp_gps_set_baudrate(size_t baudrate)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // Validate baudrate
  if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400 && baudrate != 57600 && baudrate != 115200)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));

  payload[0] = 0x01;

  // Mode: 8N1
  payload[4] = 0xC0;
  payload[5] = 0x08;
  payload[6] = 0x00;
  payload[7] = 0x00;

  // Baudrate
  payload[8]  = (baudrate >> 0) & 0xFF;
  payload[9]  = (baudrate >> 8) & 0xFF;
  payload[10] = (baudrate >> 16) & 0xFF;
  payload[11] = (baudrate >> 24) & 0xFF;

  // Input protocols: UBX + NMEA
  payload[12] = 0x03;
  payload[13] = 0x00;
  payload[14] = 0x03;
  payload[15] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_PRT, payload, sizeof(payload));

  OS_DELAY_MS(100);

  return STATUS_OK;
}

status_function_t bsp_gps_reset_default(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[13];
  memset(payload, 0, sizeof(payload));

  payload[0]  = 0xFF;
  payload[1]  = 0xFF;
  payload[2]  = 0x00;
  payload[3]  = 0x00;
  payload[8]  = 0xFF;
  payload[9]  = 0xFF;
  payload[10] = 0x00;
  payload[11] = 0x00;
  payload[12] = 0x17;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_CFG, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_cold_start(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[4];
  payload[0] = 0xFF;
  payload[1] = 0xFF;
  payload[2] = 0x02;
  payload[3] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_warm_start(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[4];
  payload[0] = 0x01;
  payload[1] = 0x00;
  payload[2] = 0x02;
  payload[3] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_hot_start(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[4];
  payload[0] = 0x00;
  payload[1] = 0x00;
  payload[2] = 0x02;
  payload[3] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_save_config(void)
{
  if (!is_gps_initialized)
  {
    return STATUS_ERROR;
  }

  uint8_t payload[13];
  memset(payload, 0, sizeof(payload));

  payload[4]  = 0xFF;
  payload[5]  = 0xFF;
  payload[6]  = 0x00;
  payload[7]  = 0x00;
  payload[12] = 0x17;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_CFG, payload, sizeof(payload));

  return STATUS_OK;
}

TinyGPSPlus *bsp_gps_get_instance(void)
{
  return &gps_handler;
}

/* Private definitions ----------------------------------------------- */
static void gps_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len)
{
  if (uart_num != GPS_UART_HANDLER)
  {
    return;
  }

#ifdef BSP_GPS_DEBUG_RAW_NMEA
  // Debug: print only printable ASCII characters (NMEA sentences)
  for (size_t i = 0; i < len; i++)
  {
    if (data[i] >= 0x20 && data[i] <= 0x7E)  // Printable ASCII
    {
      Serial.print((char) data[i]);
    }
    else if (data[i] == '\r' || data[i] == '\n')
    {
      Serial.print((char) data[i]);
    }
  }
#endif

  // Feed data to TinyGPSPlus parser
  for (size_t i = 0; i < len; i++)
  {
    // Parse each byte
    if (gps_handler.encode(data[i]))
    {
      // New complete NMEA sentence parsed
      if (gps_handler.location.isUpdated())
      {
        // Update data buffer
        gps_update_data_buffer();

        // Trigger to high layer callback if location is valid
        if (gps_data.location_valid && gps_callback != NULL)
        {
          gps_callback(&gps_data);
        }
      }
    }
  }
}

static void gps_update_data_buffer(void)
{
  // Location data
  if (gps_handler.location.isValid())
  {
    gps_data.latitude       = gps_handler.location.lat();
    gps_data.longitude      = gps_handler.location.lng();
    gps_data.location_valid = true;
  }
  else
  {
    gps_data.location_valid = false;
  }

  // Altitude
  if (gps_handler.altitude.isValid())
  {
    gps_data.altitude = gps_handler.altitude.meters();
  }

  // Speed
  if (gps_handler.speed.isValid())
  {
    gps_data.speed_kmph = gps_handler.speed.kmph();
  }

  // Course
  if (gps_handler.course.isValid())
  {
    gps_data.course = gps_handler.course.deg();
  }

  // Satellites
  if (gps_handler.satellites.isValid())
  {
    gps_data.satellites = gps_handler.satellites.value();
  }

  // HDOP
  if (gps_handler.hdop.isValid())
  {
    gps_data.hdop = gps_handler.hdop.hdop();
  }

  // Date
  if (gps_handler.date.isValid())
  {
    gps_data.year       = gps_handler.date.year();
    gps_data.month      = gps_handler.date.month();
    gps_data.day        = gps_handler.date.day();
    gps_data.date_valid = true;
  }
  else
  {
    gps_data.date_valid = false;
  }

  // Time
  if (gps_handler.time.isValid())
  {
    gps_data.hour       = gps_handler.time.hour();
    gps_data.minute     = gps_handler.time.minute();
    gps_data.second     = gps_handler.time.second();
    gps_data.time_valid = true;
  }
  else
  {
    gps_data.time_valid = false;
  }
}

static void gps_send_ubx_command(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
  // Calculate total message size: sync(2) + class(1) + id(1) + len(2) + payload + checksum(2)
  uint16_t total_len = 6 + payload_len + 2;
  uint8_t *buffer    = (uint8_t *) malloc(total_len);

  if (buffer == NULL)
  {
    return;
  }

  buffer[0] = GPS_UBX_SYNC1;
  buffer[1] = GPS_UBX_SYNC2;
  buffer[2] = msg_class;
  buffer[3] = msg_id;
  buffer[4] = payload_len & 0xFF;
  buffer[5] = (payload_len >> 8) & 0xFF;
  if (payload != NULL && payload_len > 0)
  {
    memcpy(&buffer[6], payload, payload_len);
  }

  uint8_t ck_a, ck_b;
  gps_calculate_checksum(&buffer[2], 4 + payload_len, &ck_a, &ck_b);
  buffer[6 + payload_len] = ck_a;
  buffer[7 + payload_len] = ck_b;

  // Send command via UART
  bsp_uart_write(GPS_UART_HANDLER, (const char *) buffer, total_len);

  free(buffer);
}

static void gps_calculate_checksum(uint8_t *buffer, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
  *ck_a = 0;
  *ck_b = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    *ck_a += buffer[i];
    *ck_b += *ck_a;
  }
}

/* End of file -------------------------------------------------------- */
