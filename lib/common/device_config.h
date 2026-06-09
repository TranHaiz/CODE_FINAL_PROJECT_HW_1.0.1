/**
 * @file       device_config.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-01-17
 * @author     Hai Tran
 *
 * @brief      Device pin, handler, i2c address,.. configuration definitions
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _DEVICE_CONFIG_H_
#define _DEVICE_CONFIG_H_
/* Includes ----------------------------------------------------------- */
/* Public defines ----------------------------------------------------- */

// ------------------------------ Device configuration ------------------------------
#define FIRRMWARE_MAJOR_VERSION         (1)
#define FIRRMWARE_MINOR_VERSION         (1)
#define FIRRMWARE_PATCH_VERSION         (3)

#define DEFAULT_DEVICE_NAME             "haq-trk-000"
#define DEVICE_NAME_MAX_LEN             (32)
#define DEVICE_NVS_NAMESPACE            "bsp_device"
#define DEVICE_NVS_KEY_INFO             "dev_info"
#define DEVICE_NVS_KEY_MAGIC            "dev_magic"
#define DEVICE_VERSION_LEN              (6)   // example: 1.0.0 (5 chars + 1 null terminator)
#define DEVICE_SERIAL_NUMBER_MAX_LEN    (33)  // 32 chars + 1 null terminator
#define DEVICE_MAGIC_NUMBER             (0xDEADBEEF)

#define DEVICE_NORMAL_MODE              (false)
#define DEVICE_FUSION_DEBUG_MODE        (false)
#define DEVICE_FUSION_DEBUG_VIA_NETWORK (false)
#define DEVICE_FUSION_DEBUG_LOG_ENABLED (false)

#if (DEVICE_FUSION_DEBUG_LOG_ENABLED && !DEVICE_FUSION_DEBUG_MODE)
#error "DEVICE_FUSION_DEBUG_LOG_ENABLED requires DEVICE_FUSION_DEBUG_MODE to also be true"
#endif

#define DEVICE_FUSION_FILTER_EMA          (0)
#define DEVICE_FUSION_FILTER_BTW          (1)
#define DEVICE_FUSION_ACC_FILTER          DEVICE_FUSION_FILTER_BTW
#define DEVICE_FUSION_COMPASS_FILTER      DEVICE_FUSION_FILTER_BTW
#define DEVICE_FUSION_TUNING_MODE_ENABLED (false)  // Enable live tuning of fusion parameters

// Fusion algorithm version (see sys_fusion.cpp vs sys_fusion_legacy.cpp)
#define DEVICE_FUSION_ALGO_LEGACY         (0)  // v1.1.2: direct compass yaw, CF_WC=2.0
#define DEVICE_FUSION_ALGO_V2             (1)  // yaw CF + compass Butterworth + fix distance + tuning BLE
#define DEVICE_FUSION_ALGO                DEVICE_FUSION_ALGO_LEGACY

// Yaw heading source (V2 algo only)
#define DEVICE_FUSION_YAW_CF_ENABLED      (true)  // true = gyro_z + compass CF; false = direct compass heading
#define DEVICE_IDLE_MODE_ENABLED          (true)   // Return to idle mode after a period of inactivity
#define DEVICE_NETWORK_ENABLED            (true)   // Enable network communication (MQTT, Firebase, etc.)
#define DEVICE_NETWORK_TOTAL_KM_ENABLED   (true)   // Enable total km in network messages
#define DEVICE_INPUT_ENABLED              (true)   // Enable input handling (buttons, touch, etc.)
#define DEVICE_UI_ENABLED                 (true)   // Enable UI rendering on the screen
#define DEVICE_LOCK_DEBUG_MODE_ENABLED    (false)  // Use stop and pause button to simulate lock and pause in debug mode

#define DEVICE_SIM_EG800K                 (true)
#define DEVICE_SIM_A7680C                 (false)

#if (DEVICE_SIM_EG800K && DEVICE_SIM_A7680C)
#error \
  "Only one SIM module can be selected. Please set either DEVICE_SIM_EG800K or DEVICE_SIM_A7680C to true, and the other to false."
#endif

// ------------------------------ LOG service configuration ------------------------------
#define LOG_ENABLE                        (1)
#define LOG_USB_ENABLE                    (1)
#define LOG_SDCARD_ENABLE                 (1)

// Log levels
#define LOG_LEVEL_MAIN                    LOG_LEVEL_INFO
#define LOG_LEVEL_BSP_BATT                LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_ACC                 LOG_LEVEL_ERROR
#define LOG_LEVEL_DEVICE_INFO             LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_COMPASS             LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_BUZZER              LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_DUST_SENSOR         LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_LED                 LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_SDCARD              LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_SIM                 LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_TEMP_HUM            LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_RTC                 LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_USB                 LOG_LEVEL_ERROR
#define LOG_LEVEL_BSP_BLE                 LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_CMD_USB             LOG_LEVEL_INFO
#define LOG_LEVEL_SYS_NETWORK             LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_MANAGER             LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_UI                  LOG_LEVEL_ERROR
#define LOG_LEVEL_SYS_INPUT               LOG_LEVEL_ERROR
#define LOG_LEVEL_SYS_FUSION              LOG_LEVEL_ERROR
#define LOG_LEVEL_SYS_BUTTON              LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_CMD                 LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_LED                 LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_BUZZER              LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_BLE                 LOG_LEVEL_DBG
#define LOG_LEVEL_SYS_ERROR               LOG_LEVEL_DBG

// Network configuration
#define CONFIG_FIREBASE_SERVER            (false)
#define CONFIG_MQTT_SERVER                (true)
#define NETWORK_SWITCH_LOST_THRESHOLD     (10)
#define DEVICE_BLE_FALLBACK_ENABLED       (true)

#define MQTT_MAX_TOPIC_LEN                (64)

#define DEVICE_LOG_SD_PATH_MAX_LEN        (128)

// ------------------------------ IL9341 screen configuration ------------------------------
#define SCREEN_SKIP_LOCK_SCREEN           (false)
#define SCREEN_ROTATION_0                 (false)
#define SCREEN_ROTATION_90                (false)
#define SCREEN_ROTATION_180               (false)
#define SCREEN_ROTATION_270               (true)

#define SCREEN_TOUCH_CS                   (3)
#define SCREEN_TOUCH_IRQ                  (8)
#define SCREEN_MISO                       (13)
#define SCREEN_BL                         (46)
#define SCREEN_SCLK                       (12)
#define SCREEN_MOSI                       (11)
#define SCREEN_DC                         (9)
#define SCREEN_RST                        (10)
#define SCREEN_CS                         (14)

// ------------------------------ UART configuration ------------------------------
#define SIM_UART_HANDLER                  UART_NUM_2
#define SIM_UART_TX                       (42)  // Connect to SIM RX
#define SIM_UART_RX                       (2)   // Connect to SIM TX
#define SIM_UART_BAUDRATE                 (115200)

#define GPS_UART_HANDLER                  UART_NUM_1
#define GPS_UART_TX                       (43)  // Connect to GPS RX
#define GPS_UART_RX                       (44)  // Connect to GPS TX
#define GPS_UART_BAUDRATE                 (9600)

// ------------------------------ GP2Y1010AU0F Dust Sensor configuration ------------------------------
#define DUST_SENSOR_LED_PIN               (15)
#define DUST_SENSOR_AOOUT_PIN             (16)

// ------------------------------ LSM6DS3 Accelerometer I2C configuration ------------------------------
#define ACC_I2C_SDA_PIN                   (4)
#define ACC_I2C_SCL_PIN                   (5)
#define ACC_INT_PIN                       (1)
#define ACC_I2C_ADDR                      (0x6B)
#define ACC_I2C_ADDR_ALT                  (0x6A)
#define ACC_MOTION_DETECT_THRESHOLD_LEVEL (3)  // 1: low, 2: medium, 3: high

// ------------------------------ IMU axis sign / body frame remap ------------------------------
#define ACC_AXIS_SIGN_X                   (+1)  // +1 mean back to front, -1 mean front to back
#define ACC_AXIS_SIGN_Y                   (-1)  // +1 mean left to right, -1 mean right to left
#define ACC_AXIS_SIGN_Z                   (-1)  // +1 mean down, -1 mean up
#define GYRO_AXIS_SIGN_X                  (-1)  // +1 mean right side down 30 deg => roll = 30 deg
#define GYRO_AXIS_SIGN_Y                  (-1)  // +1 mean nose up 30 deg => pitch = 30 deg
#define GYRO_AXIS_SIGN_Z                  (-1)  // +1 mean frome topview, rotaion right => heading increasing

// ------------------------------ HMC5883L Compass I2C configuration ------------------------------
#define COMPASS_I2C_SDA_PIN               (4)
#define COMPASS_I2C_SCL_PIN               (5)
#define COMPASS_I2C_ADDR                  (0x1E)
#define COMPASS_I2C_CLOCK                 (100000)

#define COMPASS_AXIS_SIGN_X               (+1)
#define COMPASS_AXIS_SIGN_Y               (-1)
#define COMPASS_AXIS_SIGN_Z               (+1)

// ------------------------------ SHT31 Temperature and Humidity Sensor I2C configuration ------------------------------
#define TEMP_HUM_I2C_SDA_PIN              (4)
#define TEMP_HUM_I2C_SCL_PIN              (5)
#define TEMP_HUM_I2C_ADDR                 (0x44)
#define TEMP_HUM_I2C_CLOCK                (100000)

// ------------------------------ DS1307 RTC I2C configuration ------------------------------
#define RTC_I2C_SDA_PIN                   (40)
#define RTC_I2C_SCL_PIN                   (39)
#define RTC_I2C_ADDR                      (0x68)

// ------------------------------ Battery monitor (INA226) I2C configuration ------------------------------
#define BATT_I2C_SDA_PIN                  (40)
#define BATT_I2C_SCL_PIN                  (39)
#define BATT_I2C_ADDR                     (0x40)

// ------------------------------ SD Card SPI configuration ------------------------------
#define BSP_SDCARD_DEFAULT_CS_PIN         (21)
#define BSP_SDCARD_DEFAULT_SCK_PIN        (47)
#define BSP_SDCARD_DEFAULT_MOSI_PIN       (48)
#define BSP_SDCARD_DEFAULT_MISO_PIN       (45)
#define SDCARD_MIN_SPI_FREQ               (1000000)   // 1MHz min
#define SDCARD_NORMAL_SPI_FREQ            (4000000)   // 4MHz normal
#define SDCARD_MAX_SPI_FREQ               (24000000)  // 24MHz max

// Other pin definitions
#define LED_PIN                           (38)
#define BUZZER_PIN                        (6)
#define IO_BUTTON_PIN                     (17)

// ------------------------------ Network notification ------------------------------
#define NETWORK_NOTI_USERLOCK_PAYLOAD     "STOP_RENTAL"
#define NETWORK_NOTI_USERPAUSE_PAYLOAD    "NOTI_PAUSE"
#define NETWORK_NOTI_HELP                 "NOTI_HELP"
#define NETWORK_NOTI_DEVICE_STOLEN        "NOTI_STOLEN"
#define NETWORK_NOTI_LOW_BATT             "NOTI_LOW_BATT"
#define NETWORK_KEEPALIVE_MES             "KEEPALIVE"  // sent as "KEEPALIVE=n%" (n = battery %)
#define NETWORK_KEEPALIVE_MSG_MAX_LEN     (24)         // fits "KEEPALIVE=100%" + null
#define NETWORK_DEVICE_RESP_OK_PAYLOAD    "OK"

// ------------------------------ Battery ------------------------------
#define BATT_LEVEL_THRESHOLD_LOW          (10.0f)
#define BATT_LEVEL_THRESHOLD_CLEAR_LOW    (15.0f)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /*End file _DEVICE_CONFIG_H_*/

/* End of file -------------------------------------------------------- */
