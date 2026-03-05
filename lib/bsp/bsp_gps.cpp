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

/* Private defines ---------------------------------------------------- */
#define GPS_UBX_SYNC1     0xB5
#define GPS_UBX_SYNC2     0x62
#define GPS_UBX_CLASS_CFG 0x06
#define GPS_UBX_CLASS_NAV 0x01

#define GPS_UBX_CFG_PRT   0x00  // Port configuration
#define GPS_UBX_CFG_MSG   0x01  // Message configuration
#define GPS_UBX_CFG_RST   0x04  // Reset
#define GPS_UBX_CFG_RATE  0x08  // Navigation/measurement rate
#define GPS_UBX_CFG_CFG   0x09  // Save/load configuration
#define GPS_UBX_CFG_NAV5  0x24  // Navigation engine settings
#define GPS_UBX_CFG_PM2   0x3B  // Power management
#define GPS_UBX_CFG_RXM   0x11  // RX manager configuration

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static TinyGPSPlus      s_gps;         // TinyGPSPlus instance
static bsp_gps_config_t s_gps_config;  // GPS configuration
static bsp_gps_data_t   s_gps_data;    // Latest GPS data buffer
static volatile bool    s_gps_initialized    = false;
static volatile bool    s_new_data_available = false;

/* Private function prototypes ---------------------------------------- */
static void gps_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len);
static void gps_update_data_buffer(void);
static void gps_send_ubx_command(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);
static void gps_calculate_checksum(uint8_t *buffer, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

/* Function definitions ----------------------------------------------- */
status_function_t bsp_gps_init(bsp_gps_config_t *config)
{
  if (config == NULL)
  {
    return STATUS_ERROR;
  }

  // Store configuration
  s_gps_config = *config;

  // Set default baudrate if not specified
  if (s_gps_config.baudrate == 0)
  {
    s_gps_config.baudrate = BSP_GPS_DEFAULT_BAUDRATE;
  }

  // Initialize UART with GPS callback
  bsp_uart_config_t uart_cfg = { .port     = s_gps_config.uart_port,
                                 .tx_pin   = s_gps_config.tx_pin,
                                 .rx_pin   = s_gps_config.rx_pin,
                                 .baudrate = s_gps_config.baudrate,
                                 .callback = gps_uart_callback };

  bsp_uart_init(&uart_cfg);

  // Initialize GPS data buffer
  memset(&s_gps_data, 0, sizeof(s_gps_data));

  s_gps_initialized = true;

  return STATUS_OK;
}

status_function_t bsp_gps_deinit(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  s_gps_initialized = false;
  memset(&s_gps_data, 0, sizeof(s_gps_data));

  return STATUS_OK;
}

status_function_t bsp_gps_get_data(bsp_gps_data_t *data)
{
  if (data == NULL || !s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  *data = s_gps_data;

  if (!s_gps_data.location_valid)
  {
    return STATUS_ERROR;
  }

  return STATUS_OK;
}

bool bsp_gps_has_fix(void)
{
  return s_gps.location.isValid() && s_gps.location.age() < 5000;
}

uint32_t bsp_gps_get_satellites(void)
{
  if (s_gps.satellites.isValid())
  {
    return s_gps.satellites.value();
  }
  return 0;
}

status_function_t bsp_gps_set_update_rate(bsp_gps_update_rate_t rate)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RATE payload: measRate(2), navRate(2), timeRef(2)
  uint8_t  payload[6];
  uint16_t meas_rate = (uint16_t) rate;

  payload[0] = meas_rate & 0xFF;         // measRate LSB
  payload[1] = (meas_rate >> 8) & 0xFF;  // measRate MSB
  payload[2] = 0x01;                     // navRate = 1 cycle
  payload[3] = 0x00;
  payload[4] = 0x01;  // timeRef = GPS time
  payload[5] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RATE, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_set_power_mode(bsp_gps_power_mode_t mode)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RXM payload: reserved(1), lpMode(1)
  uint8_t payload[2] = { 0x00, 0x00 };

  switch (mode)
  {
  case BSP_GPS_POWER_FULL:
    payload[1] = 0x00;  // Continuous mode
    break;
  case BSP_GPS_POWER_SAVE:
    payload[1] = 0x01;  // Power save mode
    break;
  case BSP_GPS_POWER_ECO:
    payload[1] = 0x04;  // Eco mode
    break;
  default: return STATUS_ERROR;
  }

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RXM, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_set_dynamic_model(bsp_gps_dynamic_model_t model)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-NAV5 payload (36 bytes)
  uint8_t payload[36];
  memset(payload, 0, sizeof(payload));

  // Mask: only set dynamic model (bit 0)
  payload[0] = 0x01;
  payload[1] = 0x00;

  // Dynamic model
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
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // NMEA message IDs for UBX-CFG-MSG
  // Class 0xF0 (NMEA standard messages)
  const uint8_t nmea_msg_ids[] = {
    0x00,  // GGA
    0x01,  // GLL
    0x02,  // GSA
    0x03,  // GSV
    0x04,  // RMC
    0x05   // VTG
  };

  // Configure each NMEA message
  for (int i = 0; i < 6; i++)
  {
    uint8_t rate = (msg_mask & (1 << i)) ? 1 : 0;

    // UBX-CFG-MSG payload: msgClass(1), msgID(1), rate[6](6 bytes for each port)
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
    vTaskDelay(pdMS_TO_TICKS(50));  // Small delay between commands
  }

  return STATUS_OK;
}

status_function_t bsp_gps_set_baudrate(uint32_t baudrate)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // Validate baudrate
  if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400 && baudrate != 57600 && baudrate != 115200)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-PRT payload for UART1 (20 bytes)
  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));

  payload[0] = 0x01;  // Port ID: UART1

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

  // Output protocols: UBX + NMEA
  payload[14] = 0x03;
  payload[15] = 0x00;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_PRT, payload, sizeof(payload));

  // Note: After this command, need to reinitialize UART with new baudrate
  vTaskDelay(pdMS_TO_TICKS(100));

  return STATUS_OK;
}

status_function_t bsp_gps_factory_reset(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-CFG: clear all sections and load defaults
  uint8_t payload[13];
  memset(payload, 0, sizeof(payload));

  // clearMask: clear all (ioPort, msgConf, navConf, rxmConf, etc.)
  payload[0] = 0xFF;
  payload[1] = 0xFF;
  payload[2] = 0x00;
  payload[3] = 0x00;

  // saveMask: 0 (don't save)
  // clearMask is already set above

  // loadMask: load defaults
  payload[8]  = 0xFF;
  payload[9]  = 0xFF;
  payload[10] = 0x00;
  payload[11] = 0x00;

  // deviceMask: all devices
  payload[12] = 0x17;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_CFG, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_cold_start(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RST: Cold start (clear all data)
  uint8_t payload[4];
  payload[0] = 0xFF;  // navBbrMask LSB: clear all
  payload[1] = 0xFF;  // navBbrMask MSB
  payload[2] = 0x02;  // resetMode: controlled software reset (GPS only)
  payload[3] = 0x00;  // reserved

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_warm_start(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RST: Warm start (clear ephemeris, keep almanac)
  uint8_t payload[4];
  payload[0] = 0x01;  // navBbrMask LSB: clear ephemeris only
  payload[1] = 0x00;  // navBbrMask MSB
  payload[2] = 0x02;  // resetMode: controlled software reset (GPS only)
  payload[3] = 0x00;  // reserved

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_hot_start(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-RST: Hot start (keep all data)
  uint8_t payload[4];
  payload[0] = 0x00;  // navBbrMask LSB: clear nothing
  payload[1] = 0x00;  // navBbrMask MSB
  payload[2] = 0x02;  // resetMode: controlled software reset (GPS only)
  payload[3] = 0x00;  // reserved

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RST, payload, sizeof(payload));

  return STATUS_OK;
}

status_function_t bsp_gps_save_config(void)
{
  if (!s_gps_initialized)
  {
    return STATUS_ERROR;
  }

  // UBX-CFG-CFG: Save current configuration
  uint8_t payload[13];
  memset(payload, 0, sizeof(payload));

  // clearMask: 0 (don't clear anything)
  // saveMask: save all sections
  payload[4] = 0xFF;
  payload[5] = 0xFF;
  payload[6] = 0x00;
  payload[7] = 0x00;

  // loadMask: 0 (don't load)
  // deviceMask: all non-volatile memory devices
  payload[12] = 0x17;

  gps_send_ubx_command(GPS_UBX_CLASS_CFG, GPS_UBX_CFG_CFG, payload, sizeof(payload));

  return STATUS_OK;
}

TinyGPSPlus *bsp_gps_get_instance(void)
{
  return &s_gps;
}

/* Private definitions ----------------------------------------------- */

/**
 * @brief UART callback for GPS data reception
 * @param uart_num UART port number
 * @param data Received data buffer
 * @param len Length of received data
 */
static void gps_uart_callback(uart_port_t uart_num, uint8_t *data, size_t len)
{
  if (!s_gps_initialized || uart_num != s_gps_config.uart_port)
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
    if (s_gps.encode(data[i]))
    {
      // New complete NMEA sentence parsed
      if (s_gps.location.isUpdated())
      {
        // Update data buffer
        gps_update_data_buffer();

        // Trigger user callback if location is valid
        if (s_gps_data.location_valid && s_gps_config.callback != NULL)
        {
          s_gps_config.callback(&s_gps_data);
        }
      }
    }
  }
}

/**
 * @brief Update GPS data buffer from TinyGPSPlus
 */
static void gps_update_data_buffer(void)
{
  // Location data
  if (s_gps.location.isValid())
  {
    s_gps_data.latitude       = s_gps.location.lat();
    s_gps_data.longitude      = s_gps.location.lng();
    s_gps_data.location_valid = true;
  }
  else
  {
    s_gps_data.location_valid = false;
  }

  // Altitude
  if (s_gps.altitude.isValid())
  {
    s_gps_data.altitude = s_gps.altitude.meters();
  }

  // Speed
  if (s_gps.speed.isValid())
  {
    s_gps_data.speed_kmph = s_gps.speed.kmph();
  }

  // Course
  if (s_gps.course.isValid())
  {
    s_gps_data.course = s_gps.course.deg();
  }

  // Satellites
  if (s_gps.satellites.isValid())
  {
    s_gps_data.satellites = s_gps.satellites.value();
  }

  // HDOP
  if (s_gps.hdop.isValid())
  {
    s_gps_data.hdop = s_gps.hdop.hdop();
  }

  // Date
  if (s_gps.date.isValid())
  {
    s_gps_data.year       = s_gps.date.year();
    s_gps_data.month      = s_gps.date.month();
    s_gps_data.day        = s_gps.date.day();
    s_gps_data.date_valid = true;
  }
  else
  {
    s_gps_data.date_valid = false;
  }

  // Time
  if (s_gps.time.isValid())
  {
    s_gps_data.hour       = s_gps.time.hour();
    s_gps_data.minute     = s_gps.time.minute();
    s_gps_data.second     = s_gps.time.second();
    s_gps_data.time_valid = true;
  }
  else
  {
    s_gps_data.time_valid = false;
  }
}

/**
 * @brief Send UBX protocol command to GPS
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Payload data
 * @param payload_len Payload length
 */
static void gps_send_ubx_command(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
  // Calculate total message size: sync(2) + class(1) + id(1) + len(2) + payload + checksum(2)
  uint16_t total_len = 6 + payload_len + 2;
  uint8_t *buffer    = (uint8_t *) malloc(total_len);

  if (buffer == NULL)
  {
    return;
  }

  // Build UBX message
  buffer[0] = GPS_UBX_SYNC1;
  buffer[1] = GPS_UBX_SYNC2;
  buffer[2] = msg_class;
  buffer[3] = msg_id;
  buffer[4] = payload_len & 0xFF;
  buffer[5] = (payload_len >> 8) & 0xFF;

  // Copy payload
  if (payload != NULL && payload_len > 0)
  {
    memcpy(&buffer[6], payload, payload_len);
  }

  // Calculate checksum (over class, id, length, and payload)
  uint8_t ck_a, ck_b;
  gps_calculate_checksum(&buffer[2], 4 + payload_len, &ck_a, &ck_b);
  buffer[6 + payload_len] = ck_a;
  buffer[7 + payload_len] = ck_b;

  // Send command via UART
  bsp_uart_write(s_gps_config.uart_port, (const char *) buffer, total_len);

  free(buffer);
}

/**
 * @brief Calculate UBX checksum (Fletcher algorithm)
 * @param buffer Data buffer
 * @param len Data length
 * @param ck_a Pointer to checksum A
 * @param ck_b Pointer to checksum B
 */
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
