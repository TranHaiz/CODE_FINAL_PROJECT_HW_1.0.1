/**
 * @file       sys_ui.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    2.0.0 (Full UI Layer)
 * @date       2026-03-03
 * @author     Hai Tran
 *
 * @brief      UI system implementation
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui.h"

#include "bsp_dust_sensor.h"
#include "bsp_led.h"
#include "bsp_rtc.h"
#include "bsp_sdcard.h"
#include "common_type.h"
#include "fifo.h"
#include "log_service.h"
#include "os_lib.h"
#include "sys_input.h"
#include "sys_manager.h"
#include "sys_ui_widget.h"

#include <math.h>
#include <string.h>

#ifndef PI
#define PI (3.14159265358979323846)
#endif

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(sys_ui, LOG_LEVEL_SYS_UI);

#define SYS_UI_LED_DEFAULT_BRIGHTNESS              (80)

// Platform UI settings
#define SYS_UI_COLOR_BG                            SYS_UI_WIDGET_COLOR_DARK_NAVY
#define SYS_UI_COLOR_BG_CARD                       SYS_UI_WIDGET_COLOR_BLUE_GRAY
#define SYS_UI_COLOR_PRIMARY                       SYS_UI_WIDGET_COLOR_CYAN
#define SYS_UI_COLOR_ACCENT                        SYS_UI_WIDGET_COLOR_GOLD
#define SYS_UI_COLOR_SUCCESS                       SYS_UI_WIDGET_COLOR_GREEN
#define SYS_UI_COLOR_WARNING                       SYS_UI_WIDGET_COLOR_ORANGE
#define SYS_UI_COLOR_DANGER                        SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_COLOR_TEXT                          ui_ctx.text_color
#define SYS_UI_COLOR_TEXT_DIM                      ui_ctx.text_dim_color
#define SYS_UI_DATA_HISTORY_SAMPLES                (100)
#define SYS_UI_BRIGHTNESS_PERCENT_OFF              (0)

// Timing
#define SYS_UI_COUNTDOWN_MS                        (1000)
#define SYS_UI_ENVIRONMENT_MS                      (3000)

// Data limits
#define SYS_UI_MAX_RENTAL_HISTORY                  (4)
#define SYS_UI_MAX_DISTANCE_LOG                    (16)
#define SYS_UI_MAX_TEMP_SAMPLES                    (120)

// Top control buttons
#define SYS_UI_CTRL_BTN_Y                          (5)
#define SYS_UI_CTRL_BTN_H                          (25)
#define SYS_UI_SETTINGS_BTN_X                      (10)
#define SYS_UI_SETTINGS_BTN_W                      (90)
#define SYS_UI_OUT_BTN_X                           (220)
#define SYS_UI_OUT_BTN_W                           (90)

// Speedometer arc
#define SYS_UI_SPEEDO_CX                           (110)
#define SYS_UI_SPEEDO_CY                           (120)
#define SYS_UI_SPEEDO_OUTER_R                      (74)
#define SYS_UI_SPEEDO_INNER_R                      (54)

// Compass panel
#define SYS_UI_MAP_PANEL_X                         (2)
#define SYS_UI_MAP_PANEL_Y                         (183)
#define SYS_UI_MAP_PANEL_W                         (50)
#define SYS_UI_MAP_PANEL_H                         (52)
#define SYS_UI_COMPASS_CX                          (22)
#define SYS_UI_COMPASS_CY                          (210)
#define SYS_UI_COMPASS_R                           (15)
#define SYS_UI_HEADING_TEXT_X                      (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_HEADING_TEXT_Y                      (SYS_UI_COMPASS_CY - 12)
#define SYS_UI_COMPASS_DEG_X                       (SYS_UI_COMPASS_CX - 12)
#define SYS_UI_COMPASS_DEG_Y                       (SYS_UI_COMPASS_CY + 5)
#define SYS_UI_COMPASS_PANEL_BG_COLOR              ui_ctx.compass_bg

// Warning label
#define SYS_UI_WARNING_LABEL_X                     (SYS_UI_MAP_PANEL_X + SYS_UI_MAP_PANEL_W + 10)
#define SYS_UI_WARNING_LABEL_Y                     (SYS_UI_MAP_PANEL_Y)
#define SYS_UI_WARNING_LABEL_TEXT                  "OUT OF ZONE"
#define SYS_UI_WARNING_LABEL_TEXT_COLOR            SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_WARNING_LABEL_FONT                  (&lv_font_montserrat_14)

// Warning label for error out of max money
#define SYS_UI_NOTI_RENTAL_LIMIT_LABEL_X           (SYS_UI_MAP_PANEL_X + SYS_UI_MAP_PANEL_W + 5)
#define SYS_UI_NOTI_RENTAL_LIMIT_LABEL_Y           (SYS_UI_MAP_PANEL_Y)
#define SYS_UI_NOTI_RENTAL_LIMIT_LABEL_TEXT        "STOP, RETURN ZONE"
#define SYS_UI_NOTI_RENTAL_LIMIT_LABEL_TEXT_COLOR  SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_NOTI_RENTAL_LIMIT_LABEL_FONT        (&lv_font_montserrat_14)

// Warning add fund label
#define SYS_UI_NOTI_WARN_ADD_FUND_LABEL_X          (SYS_UI_MAP_PANEL_X + SYS_UI_MAP_PANEL_W + 5)
#define SYS_UI_NOTI_WARN_ADD_FUND_LABEL_Y          (SYS_UI_MAP_PANEL_Y)
#define SYS_UI_NOTI_WARN_ADD_FUND_LABEL_TEXT       "PLEASE ADD FUNDS"
#define SYS_UI_NOTI_WARN_ADD_FUND_LABEL_TEXT_COLOR SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_NOTI_WARN_ADD_FUND_LABEL_FONT       (&lv_font_montserrat_14)

// Noti add fund label
#define SYS_UI_SHOULD_ADD_FUND_LABEL_X             (SYS_UI_MAP_PANEL_X + SYS_UI_MAP_PANEL_W + 5)
#define SYS_UI_SHOULD_ADD_FUND_LABEL_Y             (SYS_UI_MAP_PANEL_Y)
#define SYS_UI_SHOULD_ADD_FUND_LABEL_TEXT          "SHOULD ADD FUNDS"
#define SYS_UI_SHOULD_ADD_FUND_LABEL_TEXT_COLOR    SYS_UI_WIDGET_COLOR_RED
#define SYS_UI_SHOULD_ADD_FUND_LABEL_FONT          (&lv_font_montserrat_14)

// Right panel info cards
#define SYS_UI_CARD_X                              (218)
#define SYS_UI_CARD_W                              (100)
#define SYS_UI_TIME_CARD_Y                         (34)
#define SYS_UI_TIME_CARD_H                         (52)
#define SYS_UI_DIST_CARD_Y                         (91)
#define SYS_UI_DIST_CARD_H                         (40)
#define SYS_UI_ENV_CARD_Y                          (136)
#define SYS_UI_ENV_CARD_H                          (100)

// Shared sub-screen back button
#define SYS_UI_BACK_BTN_X                          (10)
#define SYS_UI_BACK_BTN_Y                          (10)
#define SYS_UI_BACK_BTN_W                          (60)
#define SYS_UI_BACK_BTN_H                          (25)
#define SYS_UI_BACK_BTN_LABEL                      "BACK"
#define SYS_UI_BACK_BTN_COLOR                      SYS_UI_COLOR_ACCENT
#define SYS_UI_BACK_BTN_TEXT_COLOR                 SYS_UI_COLOR_TEXT

// Screen out
#define SYS_UI_PAUSE_BTN_X                         (10)
#define SYS_UI_PAUSE_BTN_Y                         (105)
#define SYS_UI_PAUSE_BTN_W                         (120)
#define SYS_UI_PAUSE_BTN_H                         (50)
#define SYS_UI_PAUSE_BTN_LABEL                     "PAUSE"
#define SYS_UI_PAUSE_BTN_COLOR                     SYS_UI_WIDGET_COLOR_ORANGE
#define SYS_UI_PAUSE_BTN_TEXT_COLOR                SYS_UI_COLOR_TEXT
#define SYS_UI_STOP_BTN_X                          (SYS_UI_PAUSE_BTN_X + SYS_UI_PAUSE_BTN_W + 50)
#define SYS_UI_STOP_BTN_Y                          (105)
#define SYS_UI_STOP_BTN_W                          (120)
#define SYS_UI_STOP_BTN_H                          (50)
#define SYS_UI_STOP_BTN_LABEL                      "STOP"
#define SYS_UI_STOP_BTN_COLOR                      SYS_UI_COLOR_SUCCESS
#define SYS_UI_STOP_BTN_TEXT_COLOR                 SYS_UI_COLOR_TEXT
#define SYS_UI_OUT_LABEL_X                         (50)
#define SYS_UI_OUT_LABEL_Y                         (81)
#define SYS_UI_OUT_LABEL_TEXT                      "Do you want to quit the bike?"

// Settings screen
#define SYS_UI_SWATCH_ROW_Y                        (130)
#define SYS_UI_SWATCH_SIZE                         (36)
#define SYS_UI_SWATCH_SPAN                         (50)
#define SYS_UI_SWATCH_X                            (40)
#define SYS_UI_SWATCH_LABEL_Y_OFFSET               (3)

// Time screen title
#define SYS_UI_EXTEND_LABEL                        "ACTIVE HISTORY"
#define SYS_UI_EXTEND_LABEL_FONT                   (&lv_font_montserrat_18)
#define SYS_UI_EXTEND_LABEL_X                      (SYS_UI_BACK_BTN_X + SYS_UI_BACK_BTN_W + 10)
#define SYS_UI_EXTEND_LABEL_Y                      (SYS_UI_BACK_BTN_Y)

// Main screen text + layout
#define SYS_UI_SETTINGS_BTN_LABEL                  "SETTINGS"
#define SYS_UI_OUT_BTN_LABEL                       "OUT"
#define SYS_UI_SPEEDO_START_ANGLE                  (135)
#define SYS_UI_SPEEDO_END_ANGLE                    (405)
#define SYS_UI_SPEEDO_BG_COLOR                     ui_ctx.speedo_bg
#define SYS_UI_SPEED_LABEL_X_OFFSET                (-20)
#define SYS_UI_SPEED_LABEL_Y_OFFSET                (-14)
#define SYS_UI_SPEED_LABEL_INIT                    "0"
#define SYS_UI_SPEED_LABEL_W                       (40)
#define SYS_UI_SPEED_UNIT_X_OFFSET                 (-12)
#define SYS_UI_SPEED_UNIT_Y_OFFSET                 (18)
#define SYS_UI_SPEED_UNIT_LABEL                    "km/h"
#define SYS_UI_COMPASS_NEEDLE_INSET                (4)
#define SYS_UI_COMPASS_NEEDLE_WIDTH                (2)
#define SYS_UI_COMPASS_DEG_LABEL_INIT              "0\xc2\xb0"
#define SYS_UI_COMPASS_DIR_LABEL_INIT              "N"
#define SYS_UI_COMPASS_DIR_OFFSET_DEG              (22.5f)
#define SYS_UI_COMPASS_DIR_SECTOR_DEG              (45.0f)
#define SYS_UI_TIME_CARD_LABEL_X                   (8)
#define SYS_UI_TIME_CARD_LABEL_Y                   (2)
#define SYS_UI_TIME_CARD_LABEL                     "ACTIVE TIME"
#define SYS_UI_TIME_LABEL_X                        (5)
#define SYS_UI_TIME_LABEL_Y                        (16)
#define SYS_UI_TIME_LABEL_INIT                     "0:00:00"
#define SYS_UI_TIME_UNIT_X                         (24)
#define SYS_UI_TIME_UNIT_Y                         (38)
#define SYS_UI_TIME_UNIT_LABEL                     ""
#define SYS_UI_DISTANCE_LABEL_X                    (5)
#define SYS_UI_DISTANCE_LABEL_Y                    (16)
#define SYS_UI_DISTANCE_LABEL_INIT                 "0.00"
#define SYS_UI_DISTANCE_UNIT_X                     (62)
#define SYS_UI_DISTANCE_UNIT_Y                     (20)
#define SYS_UI_DISTANCE_UNIT_LABEL                 "km"
#define SYS_UI_DISTANCE_TITLE_X                    (5)
#define SYS_UI_DISTANCE_TITLE_Y                    (2)
#define SYS_UI_DISTANCE_TITLE_LABEL                "DISTANCE"
#define SYS_UI_ENV_TITLE_X                         (6)
#define SYS_UI_ENV_TITLE_Y                         (0)
#define SYS_UI_ENV_TITLE_LABEL                     "ENV STATUS"
#define SYS_UI_TEMP_LABEL_X                        (5)
#define SYS_UI_TEMP_LABEL_Y                        (14)
#define SYS_UI_TEMP_LABEL_INIT                     "0.0\xc2\xb0\x43"
#define SYS_UI_HUM_LABEL_X                         (5)
#define SYS_UI_HUM_LABEL_Y                         (32)
#define SYS_UI_HUM_LABEL_INIT                      "0%"
#define SYS_UI_AQI_LABEL_X                         (5)
#define SYS_UI_AQI_LABEL_Y                         (50)
#define SYS_UI_AQI_LABEL_INIT                      "AQI: --"
#define SYS_UI_SPEED_LABEL_FORMAT                  "%d"
#define SYS_UI_TIME_LABEL_FORMAT                   "%d:%02d:%02d"
#define SYS_UI_DISTANCE_LABEL_FORMAT               "%.2f"
#define SYS_UI_TEMP_LABEL_FORMAT                   "%.1f \xc2\xb0 C"
#define SYS_UI_HUM_LABEL_FORMAT                    "%.1f%%"
#define SYS_UI_COMPASS_DEG_FORMAT                  "%.0f\xc2\xb0"
#define SYS_UI_SPEED_MIN_KPH                       (0)
#define SYS_UI_SPEED_MAX_KPH                       (40)
#define SYS_UI_SPEED_SAFE_MAX_KPH                  (20)
#define SYS_UI_SPEED_WARN_MAX_KPH                  (30)
#define SYS_UI_SPEED_ARC_MAX_VALUE                 (100)

// Settings screen text + layout
#define SYS_UI_SETTINGS_BACK_LABEL                 "< BACK"
#define SYS_UI_SETTINGS_TITLE_X                    (90)
#define SYS_UI_SETTINGS_TITLE_Y                    (13)
#define SYS_UI_SETTINGS_TITLE_LABEL                "SETTINGS"
#define SYS_UI_SETTINGS_BRIGHTNESS_X               (40)
#define SYS_UI_SETTINGS_BRIGHTNESS_Y               (52)
#define SYS_UI_SETTINGS_BRIGHTNESS_TEXT            "Brightness"
#define SYS_UI_SETTINGS_BG_X                       (40)
#define SYS_UI_SETTINGS_BG_Y                       (108)
#define SYS_UI_SETTINGS_BG_TEXT                    "Background"
#define SYS_UI_SETTINGS_SLIDER_BG_COLOR            ui_ctx.slider_bg
#define SYS_UI_SETTINGS_SLIDER_X                   (40)
#define SYS_UI_SETTINGS_SLIDER_Y                   (72)
#define SYS_UI_SETTINGS_SLIDER_W                   (220)
#define SYS_UI_SETTINGS_SLIDER_H                   (22)
#define SYS_UI_SETTINGS_SLIDER_MIN                 (5)
#define SYS_UI_SETTINGS_SLIDER_MAX                 (100)
#define SYS_UI_BRIGHTNESS_LABEL_X                  (268)
#define SYS_UI_BRIGHTNESS_LABEL_Y                  (72)
#define SYS_UI_BRIGHTNESS_LABEL_FORMAT             "%d%%"

// Time screen text + layout
#define SYS_UI_HISTORY_LABEL_X                     (40)
#define SYS_UI_HISTORY_LABEL_Y                     (55)
#define SYS_UI_HISTORY_LABEL_SPAN                  (20)
#define SYS_UI_REMAINING_LABEL_X                   (40)
#define SYS_UI_REMAINING_LABEL_Y                   (140)
#define SYS_UI_REMAINING_LABEL_INIT                "Remaining: 00:00"
#define SYS_UI_REMAINING_LABEL_FORMAT              "Remaining: %02d:%02d"
#define SYS_UI_TIME_DANGER_MIN                     (5)
#define SYS_UI_TIME_WARNING_MIN                    (10)
#define SYS_UI_COUNTDOWN_RESET_SEC                 (59)

// Fusion screen text + layout
#define SYS_UI_FUSION_MAX_SPEED_INIT               "Max: 0.0 km/h"
#define SYS_UI_FUSION_AVG_SPEED_INIT               "Avg: 0.0 km/h"
#define SYS_UI_FUSION_Y_LABEL_INIT                 "---"
#define SYS_UI_FUSION_X_START_INIT                 "0s"
#define SYS_UI_FUSION_X_END_INIT                   "---"
#define SYS_UI_FUSION_ZOOM_MINUS_LABEL             "Zoom-"
#define SYS_UI_FUSION_ZOOM_PLUS_LABEL              "Zoom+"
#define SYS_UI_FUSION_PAN_LEFT_LABEL               "<"
#define SYS_UI_FUSION_PAN_RIGHT_LABEL              ">"
#define SYS_UI_FUSION_INACTIVE_BTN_BG              (0x333333)
#define SYS_UI_FUSION_MIN_HOURS                    (0.001f)
#define SYS_UI_CHART_Y_PAD_RAW                     (5)
#define SYS_UI_CHART_Y_MIN_SPAN_RAW                (10)
#define SYS_UI_FUSION_X_END_OFFSET                 (30)
#define SYS_UI_FUSION_ZOOM_MIN                     (1)
#define SYS_UI_FUSION_ZOOM_MAX                     (4)

// ENV screen text + layout
#define SYS_UI_ENV_Y_LABEL_INIT                    "---"
#define SYS_UI_ENV_X_START_INIT                    "0s"
#define SYS_UI_ENV_X_END_INIT                      "---"
#define SYS_UI_ENV_ZOOM_MINUS_LABEL                "Zoom-"
#define SYS_UI_ENV_ZOOM_PLUS_LABEL                 "Zoom+"
#define SYS_UI_ENV_PAN_LEFT_LABEL                  "<"
#define SYS_UI_ENV_PAN_RIGHT_LABEL                 ">"
#define SYS_UI_ENV_X_END_OFFSET                    (30)
#define SYS_UI_ENV_UNIT_TEMP                       "C"
#define SYS_UI_ENV_UNIT_HUM                        "%"
#define SYS_UI_ENV_ZOOM_MIN                        (1)
#define SYS_UI_ENV_ZOOM_MAX                        (4)
#define SYS_UI_PAN_STEP                            (5)

// AQI status text
#define SYS_UI_AQI_STATUS_GOOD                     "Good"
#define SYS_UI_AQI_STATUS_FAIR                     "Fair"
#define SYS_UI_AQI_STATUS_POOR                     "Poor"
#define SYS_UI_AQI_STATUS_GOOD_MIN                 (80)
#define SYS_UI_AQI_STATUS_FAIR_MIN                 (50)
#define SYS_UI_AQI_LABEL_FORMAT                    "AQI:%.2f %s"

// Lock screen
#define SYS_UI_LOCK_BG_COLOR                       (0x000000)

// Runtime sampling
#define SYS_UI_SPEED_SAMPLE_INTERVAL               (60)

// ENV screen chart
#define SYS_UI_ENV_GRAPH_X                         (38)
#define SYS_UI_ENV_GRAPH_Y                         (50)
#define SYS_UI_ENV_GRAPH_W                         (270)
#define SYS_UI_ENV_GRAPH_H                         (140)
#define SYS_UI_ENV_GRAPH_POINTS                    (60)
#define SYS_UI_ENV_GRAPH_COLOR                     BSP_DISPLAY_RGB_TO_HEX(17, 17, 17)  // VERY DARK GRAY
// ENV screen bottom control buttons
#define SYS_UI_ENV_BTN_Y                           (210)
#define SYS_UI_ENV_BTN_W                           (60)
#define SYS_UI_ENV_BTN_H                           (24)
#define SYS_UI_ENV_BTN_GAP                         (8)
// ENV screen tab selector buttons (TEMP / HUM / DUST)
#define SYS_UI_ENV_TAB_Y                           SYS_UI_BACK_BTN_Y
#define SYS_UI_ENV_TAB_H                           SYS_UI_BACK_BTN_H
#define SYS_UI_ENV_TAB_W                           (70)
#define SYS_UI_ENV_TAB_GAP                         (5)
#define SYS_UI_ENV_TAB_TEMP_LABEL                  "TEMP"
#define SYS_UI_ENV_TAB_HUM_LABEL                   "HUM"
#define SYS_UI_ENV_TAB_DUST_LABEL                  "AQI"
#define SYS_UI_ENV_TAB_TEMP_X                      (SYS_UI_BACK_BTN_X + SYS_UI_BACK_BTN_W + (SYS_UI_ENV_TAB_GAP * 2))
#define SYS_UI_ENV_TAB_HUM_X                       (SYS_UI_ENV_TAB_TEMP_X + SYS_UI_ENV_TAB_W + SYS_UI_ENV_TAB_GAP)
#define SYS_UI_ENV_TAB_DUST_X                      (SYS_UI_ENV_TAB_HUM_X + SYS_UI_ENV_TAB_W + SYS_UI_ENV_TAB_GAP)
// ENV screen axis labels
#define SYS_UI_ENV_Y_LABEL_X                       (1)
#define SYS_UI_ENV_Y_LABEL_W                       (36)
#define SYS_UI_ENV_X_LABEL_Y                       (193)
#define SYS_UI_ENV_TAB_ACTIVE_COLOR                SYS_UI_COLOR_SUCCESS
#define SYS_UI_ENV_TAB_INACTIVE_COLOR              BSP_DISPLAY_RGB_TO_HEX(51, 51, 51)  // SUPER DARK GRAY

// Fusion screen
#define SYS_UI_FUSION_LABEL_X                      (SYS_UI_BACK_BTN_X + SYS_UI_BACK_BTN_W + 10)
#define SYS_UI_FUSION_LABEL_Y                      (SYS_UI_BACK_BTN_Y)
#define SYS_UI_FUSION_LABEL_TEXT                   "FUSION STATS"
#define SYS_UI_FUSION_LABEL_FONT                   (&lv_font_montserrat_18)
#define SYS_UI_FUSION_MAX_SPEED_TEXT               "Max Speed: %.1f km/h"
#define SYS_UI_FUSION_MAX_SPEED_X                  (30)
#define SYS_UI_FUSION_MAX_SPEED_Y                  (50)
#define SYS_UI_FUSION_AVG_SPEED_TEXT               "Avg Speed: %.1f km/h"
#define SYS_UI_FUSION_AVG_SPEED_X                  (30)
#define SYS_UI_FUSION_AVG_SPEED_Y                  (70)
#define SYS_UI_FUSION_CHART_X                      (40)
#define SYS_UI_FUSION_CHART_Y                      (90)
#define SYS_UI_FUSION_CHART_W                      (265)
#define SYS_UI_FUSION_CHART_H                      (105)
#define SYS_UI_FUSION_CHART_POINTS                 (60)
#define SYS_UI_FUSION_CHART_COLOR                  BSP_DISPLAY_RGB_TO_HEX(17, 17, 17)  // VERY DARK GRAY
#define SYS_UI_FUSION_CHART_POINT_COLOR            SYS_UI_WIDGET_COLOR_GOLD
#define SYS_UI_FUSION_Y_LABEL_X                    (1)
#define SYS_UI_FUSION_X_LABEL_Y                    (197)
#define SYS_UI_FUSION_BTN_Y                        (210)
#define SYS_UI_FUSION_BTN_W                        (60)
#define SYS_UI_FUSION_BTN_H                        (24)
#define SYS_UI_FUSION_BTN_GAP                      (8)

// Lock screen
#define SYS_UI_QR_PATH                             "/img/qr.bin"
#define SYS_UI_QR_LABEL                            "SCAN TO UNLOCK"
#define SYS_UI_QR_LABEL_X                          (50)
#define SYS_UI_QR_LABEL_Y                          (20)
#define SYS_UI_QR_LABEL_FAIL                       "[QR]"
#define SYS_UI_QR_LABEL_FAIL_X                     (100)
#define SYS_UI_QR_LABEL_FAIL_Y                     (80)
#define SYS_UI_QR_LABEL_FAIL_FONT                  (&lv_font_montserrat_28)
#define SYS_UI_QR_LABEL_FONT                       (&lv_font_montserrat_18)
#define SYS_UI_QR_WIDTH                            (160)
#define SYS_UI_QR_HEIGHT                           (160)
#define SYS_UI_QR_X                                (80)
#define SYS_UI_QR_Y                                (50)
#define SYS_UI_DEVICE_ID_LABEL_X                   (80)
#define SYS_UI_DEVICE_ID_LABEL_Y                   (215)
#define SYS_UI_DEVICE_ID_LABEL_FONT                (&lv_font_montserrat_10)

/* Background color palette for settings screen
 * INFO(index, R, G, B, label)              */
#define SYS_UI_BG_COLOR_TABLE(INFO) \
  INFO(0, 13, 27, 42, "Navy")       \
  INFO(1, 3, 4, 94, "Ocean")        \
  INFO(2, 10, 10, 10, "Black")      \
  INFO(3, 255, 255, 255, "White")
#define SYS_UI_BG_COLOR_COUNT         (4)
#define SYS_UI_CLAMP(val, minv, maxv) ((val) < (minv) ? (minv) : ((val) > (maxv) ? (maxv) : (val)))
// clang-format off
//                      field         | dark background               | light background
#define SYS_UI_THEME_TABLE(INFO)                                                                    \
  INFO(text_color,     SYS_UI_WIDGET_COLOR_WHITE,           SYS_UI_WIDGET_COLOR_DARK_NAVY       )   \
  INFO(text_dim_color, SYS_UI_WIDGET_COLOR_GRAY,            SYS_UI_WIDGET_COLOR_STEEL           )   \
  INFO(card_bg,        SYS_UI_WIDGET_COLOR_BLUE_GRAY,       SYS_UI_WIDGET_COLOR_LIGHT_CARD      )   \
  INFO(speedo_bg,      SYS_UI_WIDGET_COLOR_DARK_PANEL,      SYS_UI_WIDGET_COLOR_LIGHT_PANEL     )   \
  INFO(compass_bg,     SYS_UI_WIDGET_COLOR_DEEP_NAVY,       SYS_UI_WIDGET_COLOR_LIGHT_PANEL_DIM )   \
  INFO(slider_bg,      SYS_UI_WIDGET_COLOR_DARK_PANEL,      SYS_UI_WIDGET_COLOR_LIGHT_PANEL     )
// clang-format on

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  lv_obj_t *active_screen;
  // Main screen
  lv_obj_t *main_screen;
  lv_obj_t *speedometer_arc;
  lv_obj_t *speed_label;
  lv_obj_t *speed_unit_label;
  lv_obj_t *compass_arc;
  lv_obj_t *compass_needle;
  lv_obj_t *compass_deg_label;
  lv_obj_t *compass_dir_label;
  lv_obj_t *time_card;
  lv_obj_t *time_label;
  lv_obj_t *time_unit_label;
  lv_obj_t *distance_card;
  lv_obj_t *distance_label;
  lv_obj_t *distance_unit_label;
  lv_obj_t *env_card;
  lv_obj_t *temp_label;
  lv_obj_t *humidity_label;
  lv_obj_t *aqi_label;
  lv_obj_t *settings_btn;
  lv_obj_t *out_btn;
  lv_obj_t *warning_out_of_zone_label;
  lv_obj_t *warn_rental_limit_label;
  lv_obj_t *warn_add_fund_label;
  lv_obj_t *should_add_fund_panel;
  // Settings screen
  lv_obj_t *settings_screen;
  lv_obj_t *settings_title;
  lv_obj_t *settings_back_btn;
  lv_obj_t *brightness_slider;
  lv_obj_t *brightness_label;
  lv_obj_t *color_btns[SYS_UI_BG_COLOR_COUNT];
  // Out screen
  lv_obj_t *out_screen;
  lv_obj_t *out_back_btn;
  lv_obj_t *out_stop_btn;
  lv_obj_t *out_pause_btn;
  // Lock screen
  lv_obj_t      *lock_screen;
  lv_obj_t      *lock_qr_img;
  lv_image_dsc_t lock_qr_dsc;
  // Time history screen
  lv_obj_t *time_history_screen;
  lv_obj_t *time_history_title;
  lv_obj_t *time_back_btn;
  lv_obj_t *time_history_time_label;
  lv_obj_t *time_history_date_label;
  // Fusion screen
  lv_obj_t          *fusion_screen;
  lv_obj_t          *distance_title;
  lv_obj_t          *distance_back_btn;
  lv_obj_t          *max_speed_lable;
  lv_obj_t          *avg_speed_label;
  lv_obj_t          *speed_chart;
  lv_chart_series_t *speed_series;
  lv_obj_t          *fusion_y_max_label;
  lv_obj_t          *fusion_y_mid_label;
  lv_obj_t          *fusion_y_min_label;
  lv_obj_t          *fusion_x_start_label;
  lv_obj_t          *fusion_x_end_label;
  lv_obj_t          *fusion_zoom_minus_btn;
  lv_obj_t          *fusion_zoom_plus_btn;
  lv_obj_t          *fusion_pan_left_btn;
  lv_obj_t          *fusion_pan_right_btn;
  // ENV history screen
  lv_obj_t          *env_screen;
  lv_obj_t          *temp_back_btn;
  lv_obj_t          *env_tab_temp_btn;
  lv_obj_t          *env_tab_hum_btn;
  lv_obj_t          *env_tab_dust_btn;
  lv_obj_t          *temp_chart;
  lv_chart_series_t *temp_series;
  lv_obj_t          *hum_chart;
  lv_chart_series_t *hum_series;
  lv_obj_t          *dust_chart;
  lv_chart_series_t *dust_series;
  lv_obj_t          *env_y_max_label;
  lv_obj_t          *env_y_mid_label;
  lv_obj_t          *env_y_min_label;
  lv_obj_t          *env_x_start_label;
  lv_obj_t          *env_x_end_label;
  lv_obj_t          *zoom_minus_btn;
  lv_obj_t          *zoom_plus_btn;
  lv_obj_t          *pan_left_btn;
  lv_obj_t          *pan_right_btn;
} sys_ui_widgets_t;

typedef struct
{
  lvgl_driver_t    lvgl;
  sys_ui_widgets_t widgets;
  // Timing
  size_t last_speed_update;
  size_t last_second_tick;
  size_t last_time_update_screen;
  size_t session_start_ms;
  size_t frame_counter;
  // Motion data
  float             current_speed;
  float             max_speed;
  float             target_speed;
  float             distance_km;
  int               prev_speed_int;
  float             compass_heading_deg;
  sys_fusion_data_t fusion;
  // Rental time
  int        active_hours;
  int        active_minutes;
  int        active_seconds;
  timeline_t unlock_time;
  // Environment
  float temperature_C;
  float humidity;
  float dust_value;
  // Device state
  int    battery_percent;
  int    brightness_percent;
  size_t background_color;
  size_t text_color;
  size_t text_dim_color;
  size_t card_bg;
  size_t speedo_bg;
  size_t compass_bg;
  size_t slider_bg;
  int    last_device_state;
  // Navigation
  sys_ui_view_t view;
  sys_ui_view_t last_view;
  bool          pending_main_redraw;
  bool          pending_settings_redraw;
  uint16_t      last_touch_x;
  uint16_t      last_touch_y;
  // History logs
  char   rental_history[SYS_UI_MAX_RENTAL_HISTORY][32];
  int    rental_history_count;
  float  distance_history[SYS_UI_MAX_DISTANCE_LOG];
  int    distance_history_count;
  float  temp_history[SYS_UI_MAX_TEMP_SAMPLES];
  size_t temp_timestamps[SYS_UI_MAX_TEMP_SAMPLES];
  int    temperature_sample_count;
  int    temperature_zoom;
  int    temperature_graph_offset;
  int    env_tab;  // 0=TEMP, 1=HUM, 2=DUST
  int    fusion_zoom;
  int    fusion_graph_offset;

  bool is_warning_out_of_zone_active;
  bool is_warning_money_active;
  bool is_add_fund_notification_active;
  bool is_should_add_fund_notification_active;

  volatile sys_ui_noti_label_type_t pending_noti_label;
  sys_ui_noti_label_type_t          current_noti_label;
} sys_ui_context_t;

/* Public variables --------------------------------------------------- */
sys_ui_data_status_t g_sys_ui_data_status = { 0 };

/* Private variables -------------------------------------------------- */
static sys_ui_context_t ui_ctx;

typedef struct
{
  float  value;
  size_t timestamp;
} sys_ui_env_sample_t;

static sys_ui_env_sample_t speed_buf[SYS_UI_DATA_HISTORY_SAMPLES];
static fifo_t              speed_fifo;
static sys_ui_env_sample_t env_temp_buf[SYS_UI_DATA_HISTORY_SAMPLES];
static fifo_t              env_temp_fifo;
static sys_ui_env_sample_t env_hum_buf[SYS_UI_DATA_HISTORY_SAMPLES];
static fifo_t              env_hum_fifo;
static sys_ui_env_sample_t env_dust_buf[SYS_UI_DATA_HISTORY_SAMPLES];
static fifo_t              env_dust_fifo;

OS_SEM_DEFINE_STATIC(sys_ui_wakeup_sem);

/* Private function prototypes ---------------------------------------- */
// Core
static void              sys_ui_process_active(void);
static void              sys_ui_process_idle(void);
static void              sys_ui_process_locked(void);
static void              sys_ui_show_screen(lv_obj_t *screen);
static void              sys_ui_change_screen(sys_ui_view_t view);
static void              sys_ui_init_data(void);
static void              sys_ui_init_history_data(void);
static void              sys_ui_init_all_widgets(void);
static void              sys_ui_reset_all_widgets(sys_ui_widgets_t *screen);
static void              sys_ui_register_callbacks(void);
static void              sys_ui_apply_notification_label(void);
static status_function_t sys_ui_image_draw(lv_obj_t     *parent,
                                           const char   *path,
                                           uint16_t      width,
                                           uint16_t      height,
                                           int16_t       x,
                                           int16_t       y,
                                           lv_img_dsc_t *out_dsc,
                                           lv_obj_t    **out_obj);
// Main screen
static void sys_ui_main_screen_create(void);
static void sys_ui_main_screen_update_speed(int speed_kph);
static void sys_ui_main_screen_update_time(int hours, int minutes, int seconds);
static void sys_ui_main_screen_update_distance(float distance_km);
static void sys_ui_main_screen_update_env(float temperature_C, float humidity, float dust_value);
static void sys_ui_main_screen_update_compass(void);
static void sys_ui_main_screen_update_speed_n_distance(void);
static void sys_ui_main_screen_update_countup(void);
static void sys_ui_main_screen_cb_settings_btn(lv_event_t *event);
static void sys_ui_main_screen_cb_out_btn(lv_event_t *event);
static void sys_ui_main_screen_cb_time_card(lv_event_t *event);
static void sys_ui_main_screen_cb_distance_card(lv_event_t *event);
static void sys_ui_main_screen_cb_env_card(lv_event_t *event);
// Settings screen
static void sys_ui_settings_screen_create(void);
static void sys_ui_settings_screen_draw(void);
static void sys_ui_settings_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_settings_screen_cb_brightness_slider(lv_event_t *event);
static void sys_ui_settings_screen_cb_color_btn(lv_event_t *event);
// Out screen
static void sys_ui_out_screen_create(void);
static void sys_ui_out_screen_draw(void);
static void sys_ui_out_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_out_screen_cb_stop_btn(lv_event_t *event);
static void sys_ui_out_screen_cb_pause_btn(lv_event_t *event);
// Lock screen
static void sys_ui_lock_screen_create(void);
static void sys_ui_lock_screen_draw(void);
#if SCREEN_SKIP_LOCK_SCREEN
static void sys_ui_lock_screen_cb_debug(lv_event_t *event);
#endif
// Time history screen
static void sys_ui_time_screen_create(void);
static void sys_ui_time_screen_update(void);
static void sys_ui_time_screen_cb_back_btn(lv_event_t *event);
// Fusion screen
static void sys_ui_fusion_screen_create(void);
static void sys_ui_fusion_screen_update(void);
static void sys_ui_fusion_screen_refresh(void);
static void sys_ui_fusion_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_fusion_screen_cb_zoom_minus(lv_event_t *event);
static void sys_ui_fusion_screen_cb_zoom_plus(lv_event_t *event);
static void sys_ui_fusion_screen_cb_pan_left(lv_event_t *event);
static void sys_ui_fusion_screen_cb_pan_right(lv_event_t *event);
// Temperature screen
static void sys_ui_temp_screen_create(void);
static void sys_ui_temp_screen_update(void);
static void sys_ui_temp_screen_refresh(void);
static void sys_ui_temp_screen_update_tab_colors(void);
static void sys_ui_temp_screen_cb_back_btn(lv_event_t *event);
static void sys_ui_temp_screen_cb_tab_temp(lv_event_t *event);
static void sys_ui_temp_screen_cb_tab_hum(lv_event_t *event);
static void sys_ui_temp_screen_cb_tab_dust(lv_event_t *event);
static void sys_ui_temp_screen_cb_zoom_minus(lv_event_t *event);
static void sys_ui_temp_screen_cb_zoom_plus(lv_event_t *event);
static void sys_ui_temp_screen_cb_pan_left(lv_event_t *event);
static void sys_ui_temp_screen_cb_pan_right(lv_event_t *event);
// History logging
static void                sys_ui_log_speed_sample(float value, size_t timestamp);
static void                sys_ui_log_temp_sample(float value, size_t timestamp);
static void                sys_ui_log_hum_sample(float value, size_t timestamp);
static void                sys_ui_log_dust_sample(float value, size_t timestamp);
static void                sys_ui_env_fifo_push(fifo_t *fifo, float value, size_t timestamp);
static sys_ui_env_sample_t sys_ui_env_fifo_get(const fifo_t *fifo, int index);
// LVGL driver
static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);
// Theme
static void sys_ui_update_theme_colors(void);

/* Function definitions ----------------------------------------------- */
static void sys_ui_update_theme_colors(void)
{
  uint8_t  r    = (uint8_t) ((ui_ctx.background_color >> 16) & 0xFF);
  uint8_t  g    = (uint8_t) ((ui_ctx.background_color >> 8) & 0xFF);
  uint8_t  b    = (uint8_t) (ui_ctx.background_color & 0xFF);
  uint32_t luma = (77u * r + 150u * g + 29u * b) >> 8;

  if (luma > 128)
  {
#define INFO(field, dark_val, light_val) ui_ctx.field = (light_val);
    SYS_UI_THEME_TABLE(INFO)
#undef INFO
  }
  else
  {
#define INFO(field, dark_val, light_val) ui_ctx.field = (dark_val);
    SYS_UI_THEME_TABLE(INFO)
#undef INFO
  }
}

void sys_ui_init(void)
{
  memset(&ui_ctx, 0, sizeof(ui_ctx));

  OS_SEM_CREATE(sys_ui_wakeup_sem);
  ui_ctx.prev_speed_int     = -1;
  ui_ctx.battery_percent    = 85;
  ui_ctx.brightness_percent = 80;
  ui_ctx.background_color   = SYS_UI_COLOR_BG;
  sys_ui_update_theme_colors();
  ui_ctx.session_start_ms    = OS_GET_TICK();
  ui_ctx.last_device_state   = g_device_info.nvs_info.curr_state;
  ui_ctx.view                = SYS_UI_VIEW_LOCK;
  ui_ctx.last_view           = SYS_UI_VIEW_UNKNOWN;
  ui_ctx.pending_main_redraw = false;
  ui_ctx.temperature_zoom    = 1;
  ui_ctx.fusion_zoom         = 1;

  sys_ui_reset_all_widgets(&ui_ctx.widgets);
  bsp_display_init();

  ui_ctx.lvgl.user_data = &ui_ctx;
  lvgl_driver_init(&ui_ctx.lvgl, bsp_display_get_driver());
  lvgl_driver_set_touch_callback(&ui_ctx.lvgl, sys_ui_lvgl_touch_read_cb);

  sys_ui_init_data();
  sys_ui_init_history_data();
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
  sys_ui_init_all_widgets();
  sys_ui_register_callbacks();

  size_t now                     = OS_GET_TICK();
  ui_ctx.session_start_ms        = now;
  ui_ctx.last_speed_update       = now;
  ui_ctx.last_second_tick        = now;
  ui_ctx.last_time_update_screen = now;

  bsp_touch_init();

  LOG_DBG("sys_ui_init: complete");
}

void sys_ui_process(void)
{
  int curr_state = g_device_info.nvs_info.curr_state;

  if (curr_state != ui_ctx.last_device_state)
  {
    ui_ctx.last_device_state = curr_state;
    if (curr_state == DEVICE_STATE_ACTIVE)
    {
      size_t now              = OS_GET_TICK();
      ui_ctx.active_hours     = 0;
      ui_ctx.active_minutes   = 0;
      ui_ctx.active_seconds   = 0;
      ui_ctx.last_second_tick = now;
      ui_ctx.session_start_ms = now;
      if (bsp_rtc_get(&ui_ctx.unlock_time) == STATUS_OK)
      {
        sys_ui_main_screen_update_time(ui_ctx.active_hours, ui_ctx.active_minutes, ui_ctx.active_seconds);
      }
    }
  }

  switch (curr_state)
  {
  case DEVICE_STATE_IDLE:
  {
    sys_ui_process_idle();
    break;
  }
  case DEVICE_STATE_LOCKED:
  {
    if (ui_ctx.last_view != SYS_UI_VIEW_LOCK)
    {
      sys_ui_change_screen(SYS_UI_VIEW_LOCK);
      ui_ctx.last_view = SYS_UI_VIEW_LOCK;
    }
    sys_ui_process_locked();
    lvgl_driver_task(&ui_ctx.lvgl);
    break;
  }
  case DEVICE_STATE_ACTIVE:
  {
    if (ui_ctx.last_view != SYS_UI_VIEW_MAIN)
    {
      sys_ui_change_screen(SYS_UI_VIEW_MAIN);
      ui_ctx.last_view = SYS_UI_VIEW_MAIN;
    }
    sys_ui_process_active();
    lvgl_driver_task(&ui_ctx.lvgl);
    break;
  }
  default: break;
  }
}

void sys_ui_lock(void)
{
  LOG_DBG("sys_ui_lock: locking");
  ui_ctx.view = SYS_UI_VIEW_UNKNOWN;
}

void sys_ui_unlock(void)
{
  LOG_DBG("sys_ui_unlock: unlocking");
  ui_ctx.view = SYS_UI_VIEW_UNKNOWN;
}

void sys_ui_wakeup(void)
{
  OS_SEM_GIVE(sys_ui_wakeup_sem);
}

void sys_ui_change_time_active(void)
{
  timeline_t now;
  if (bsp_rtc_get(&now) == STATUS_OK)
  {
    // TODO: handle time rental overflow
  }
}

void sys_ui_update_notification_label(sys_ui_noti_label_type_t label_type)
{
  if (label_type < SYS_UI_NOTI_LABEL_MAX)
  {
    ui_ctx.pending_noti_label = label_type;
  }
}

static void sys_ui_apply_notification_label(void)
{
  sys_ui_noti_label_type_t label_type = ui_ctx.pending_noti_label;
  if (label_type == ui_ctx.current_noti_label)
  {
    return;
  }
  if (ui_ctx.widgets.warn_add_fund_label == nullptr || ui_ctx.widgets.should_add_fund_panel == nullptr
      || ui_ctx.widgets.warn_rental_limit_label == nullptr || ui_ctx.widgets.warning_out_of_zone_label == nullptr)
  {
    return;
  }
  ui_ctx.current_noti_label = label_type;

  switch (label_type)
  {
  case SYS_UI_NOTI_LABEL_NONE:
  {
    lv_obj_add_flag(ui_ctx.widgets.warn_add_fund_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.should_add_fund_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warn_rental_limit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
    break;
  }
  case SYS_UI_NOTI_LABEL_WARN_ADD_FUND:
  {
    lv_obj_clear_flag(ui_ctx.widgets.warn_add_fund_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.should_add_fund_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warn_rental_limit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
    break;
  }
  case SYS_UI_NOTI_LABEL_SHOULD_ADD_FUND:
  {
    lv_obj_add_flag(ui_ctx.widgets.warn_add_fund_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ctx.widgets.should_add_fund_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warn_rental_limit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
    break;
  }
  case SYS_UI_NOTI_LABEL_OUT_OF_ZONE:
  {
    lv_obj_add_flag(ui_ctx.widgets.warn_add_fund_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.should_add_fund_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warn_rental_limit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
    break;
  }
  case SYS_UI_NOTI_LABEL_RENTAL_LIMIT:
  {
    lv_obj_add_flag(ui_ctx.widgets.warn_add_fund_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.should_add_fund_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ctx.widgets.warn_rental_limit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
    break;
  }
  default: break;
  }
}
/* Private definitions ----------------------------------------------- */
static void sys_ui_show_screen(lv_obj_t *screen)
{
  if (screen == nullptr)
  {
    return;
  }

  ui_ctx.widgets.active_screen = screen;
  lv_screen_load(screen);
}

static void sys_ui_change_screen(sys_ui_view_t view)
{
  ui_ctx.view = view;

  switch (view)
  {
  case SYS_UI_VIEW_LOCK:
  {
    sys_ui_lock_screen_draw();
    break;
  }
  case SYS_UI_VIEW_MAIN:
  {
    if (ui_ctx.pending_main_redraw)
    {
      if (ui_ctx.widgets.main_screen != nullptr)
      {
        lv_obj_del(ui_ctx.widgets.main_screen);
      }
      if (ui_ctx.widgets.settings_screen != nullptr)
      {
        lv_obj_del(ui_ctx.widgets.settings_screen);
      }
      sys_ui_reset_all_widgets(&ui_ctx.widgets);
    }
    ui_ctx.pending_main_redraw = false;
    sys_ui_init_all_widgets();
    sys_ui_register_callbacks();
    break;
  }
  case SYS_UI_VIEW_SETTINGS:
  {
    sys_ui_settings_screen_draw();
    if (ui_ctx.widgets.settings_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.settings_back_btn, sys_ui_settings_screen_cb_back_btn, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.brightness_slider != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.brightness_slider, sys_ui_settings_screen_cb_brightness_slider,
                          LV_EVENT_VALUE_CHANGED, nullptr);
    }
    for (int i = 0; i < SYS_UI_BG_COLOR_COUNT; ++i)
    {
      if (ui_ctx.widgets.color_btns[i] != nullptr)
      {
        lv_obj_add_event_cb(ui_ctx.widgets.color_btns[i], sys_ui_settings_screen_cb_color_btn, LV_EVENT_CLICKED,
                            (void *) (intptr_t) i);
      }
    }
    break;
  }
  case SYS_UI_VIEW_OUT:
  {
    sys_ui_out_screen_draw();
    if (ui_ctx.widgets.out_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.out_back_btn, sys_ui_out_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
      lv_obj_add_event_cb(ui_ctx.widgets.out_stop_btn, sys_ui_out_screen_cb_stop_btn, LV_EVENT_CLICKED, nullptr);
      lv_obj_add_event_cb(ui_ctx.widgets.out_pause_btn, sys_ui_out_screen_cb_pause_btn, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    sys_ui_time_screen_update();
    if (ui_ctx.widgets.time_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.time_back_btn, sys_ui_time_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_fusion_screen_update();
    if (ui_ctx.widgets.distance_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.distance_back_btn, sys_ui_fusion_screen_cb_back_btn, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.fusion_zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.fusion_zoom_minus_btn, sys_ui_fusion_screen_cb_zoom_minus, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.fusion_zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.fusion_zoom_plus_btn, sys_ui_fusion_screen_cb_zoom_plus, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.fusion_pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.fusion_pan_left_btn, sys_ui_fusion_screen_cb_pan_left, LV_EVENT_CLICKED,
                          nullptr);
    }
    if (ui_ctx.widgets.fusion_pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.fusion_pan_right_btn, sys_ui_fusion_screen_cb_pan_right, LV_EVENT_CLICKED,
                          nullptr);
    }
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_temp_screen_update();
    if (ui_ctx.widgets.temp_back_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.temp_back_btn, sys_ui_temp_screen_cb_back_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.env_tab_temp_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.env_tab_temp_btn, sys_ui_temp_screen_cb_tab_temp, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.env_tab_hum_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.env_tab_hum_btn, sys_ui_temp_screen_cb_tab_hum, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.env_tab_dust_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.env_tab_dust_btn, sys_ui_temp_screen_cb_tab_dust, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_minus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_minus_btn, sys_ui_temp_screen_cb_zoom_minus, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.zoom_plus_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.zoom_plus_btn, sys_ui_temp_screen_cb_zoom_plus, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_left_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_left_btn, sys_ui_temp_screen_cb_pan_left, LV_EVENT_CLICKED, nullptr);
    }
    if (ui_ctx.widgets.pan_right_btn != nullptr)
    {
      lv_obj_add_event_cb(ui_ctx.widgets.pan_right_btn, sys_ui_temp_screen_cb_pan_right, LV_EVENT_CLICKED, nullptr);
    }
    break;
  }
  default: break;
  }
}

static void sys_ui_init_data(void)
{
  ui_ctx.active_hours   = 0;
  ui_ctx.active_minutes = 0;
  ui_ctx.active_seconds = 0;
  ui_ctx.current_speed  = 0.0f;
  ui_ctx.target_speed   = 12.0f;
  ui_ctx.distance_km    = 0.0f;

  ui_ctx.temperature_C   = 28.0f + static_cast<float>(random(0, 50)) / 10.0f;
  ui_ctx.humidity        = 60.0f + static_cast<float>(random(0, 300)) / 10.0f;
  ui_ctx.dust_value      = 70 + random(0, 30);
  ui_ctx.battery_percent = 80 + random(0, 20);

  if (bsp_rtc_get(&ui_ctx.unlock_time) != STATUS_OK)
  {
    memset(&ui_ctx.unlock_time, 0, sizeof(ui_ctx.unlock_time));
  }

  const char *seed[SYS_UI_MAX_RENTAL_HISTORY] = {
    "Active 09:05",
    "Active 17:40",
    "Active 08:15",
    "Active 19:50",
  };
  ui_ctx.rental_history_count = SYS_UI_MAX_RENTAL_HISTORY;
  for (int i = 0; i < SYS_UI_MAX_RENTAL_HISTORY; ++i)
  {
    strncpy(ui_ctx.rental_history[i], seed[i], sizeof(ui_ctx.rental_history[i]) - 1);
    ui_ctx.rental_history[i][sizeof(ui_ctx.rental_history[i]) - 1] = '\0';
  }
}

static void sys_ui_init_history_data(void)
{
  fifo_init(&speed_fifo, speed_buf, sizeof(sys_ui_env_sample_t), SYS_UI_DATA_HISTORY_SAMPLES);
  fifo_init(&env_temp_fifo, env_temp_buf, sizeof(sys_ui_env_sample_t), SYS_UI_DATA_HISTORY_SAMPLES);
  fifo_init(&env_hum_fifo, env_hum_buf, sizeof(sys_ui_env_sample_t), SYS_UI_DATA_HISTORY_SAMPLES);
  fifo_init(&env_dust_fifo, env_dust_buf, sizeof(sys_ui_env_sample_t), SYS_UI_DATA_HISTORY_SAMPLES);

  size_t base = OS_GET_TICK();
  for (int i = 0; i < SYS_UI_DATA_HISTORY_SAMPLES; ++i)
  {
    sys_ui_env_fifo_push(&speed_fifo, 10.0f + 8.0f * sinf((float) i / 10.0f), base + (size_t) i * 1000U);
    sys_ui_env_fifo_push(&env_temp_fifo, 26.0f + 2.0f * sinf((float) i / 10.0f), base + (size_t) i * 1000U);
    sys_ui_env_fifo_push(&env_hum_fifo, 65.0f + 10.0f * sinf((float) i / 8.0f), base + (size_t) i * 1000U);
    sys_ui_env_fifo_push(&env_dust_fifo, 70.0f + 20.0f * sinf((float) i / 12.0f), base + (size_t) i * 1000U);
  }
}

static void sys_ui_init_all_widgets(void)
{
  if (ui_ctx.widgets.lock_screen == nullptr)
  {
    sys_ui_lock_screen_create();
  }

  if (ui_ctx.view == SYS_UI_VIEW_LOCK)
  {
    sys_ui_show_screen(ui_ctx.widgets.lock_screen);
    return;
  }

  if (ui_ctx.widgets.main_screen == nullptr)
  {
    sys_ui_main_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.main_screen);
  sys_ui_main_screen_update_compass();
  sys_ui_main_screen_update_time(ui_ctx.active_hours, ui_ctx.active_minutes, ui_ctx.active_seconds);
  sys_ui_main_screen_update_distance(ui_ctx.distance_km);
  sys_ui_main_screen_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.dust_value);
  sys_ui_main_screen_update_speed(static_cast<int>(ui_ctx.current_speed));
}

static void sys_ui_reset_all_widgets(sys_ui_widgets_t *screen)
{
  memset(screen, 0, sizeof(sys_ui_widgets_t));
}

static void sys_ui_register_callbacks(void)
{
  if (ui_ctx.widgets.settings_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.settings_btn, sys_ui_main_screen_cb_settings_btn, LV_EVENT_CLICKED, nullptr);
  }
  if (ui_ctx.widgets.out_btn != nullptr)
  {
    lv_obj_add_event_cb(ui_ctx.widgets.out_btn, sys_ui_main_screen_cb_out_btn, LV_EVENT_CLICKED, nullptr);
  }
  if (ui_ctx.widgets.time_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.time_card, sys_ui_main_screen_cb_time_card, nullptr);
  }
  if (ui_ctx.widgets.distance_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.distance_card, sys_ui_main_screen_cb_distance_card, nullptr);
  }
  if (ui_ctx.widgets.env_card != nullptr)
  {
    sys_ui_widget_set_clickable(ui_ctx.widgets.env_card, sys_ui_main_screen_cb_env_card, nullptr);
  }
}

static status_function_t sys_ui_image_draw(lv_obj_t       *parent,
                                           const char     *path,
                                           uint16_t        width,
                                           uint16_t        height,
                                           int16_t         x,
                                           int16_t         y,
                                           lv_image_dsc_t *out_dsc,
                                           lv_obj_t      **out_obj)
{
  if ((parent == nullptr) || (path == nullptr) || (out_dsc == nullptr) || (out_obj == nullptr))
  {
    LOG_ERR("sys_ui_image_draw: invalid arguments");
    return STATUS_ERROR;
  }

  // 1. Allocate PSRAM buffer
  size_t   buf_size = (size_t) width * height * 2U;
  uint8_t *buf      = (uint8_t *) heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  if (buf == nullptr)
  {
    LOG_ERR("sys_ui_image_draw: PSRAM alloc failed (%u bytes)", buf_size);
    return STATUS_ERROR;
  }

  // 2. Read file from SD into PSRAM buffer
  bsp_sdcard_file_t file;
  size_t            read_len = 0U;

  if (bsp_sdcard_open(path, BSP_SDCARD_MODE_READ, &file) != STATUS_OK)
  {
    LOG_ERR("sys_ui_image_draw: cannot open %s", path);
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  if (bsp_sdcard_read(&file, buf, buf_size, &read_len) != STATUS_OK || read_len != buf_size)
  {
    LOG_ERR("sys_ui_image_draw: read failed, got %u / %u bytes", read_len, buf_size);
    bsp_sdcard_close(&file);
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  bsp_sdcard_close(&file);

  // 3. Bind buffer
  out_dsc->header.magic  = LV_IMAGE_HEADER_MAGIC;
  out_dsc->header.cf     = LV_COLOR_FORMAT_RGB565;
  out_dsc->header.w      = width;
  out_dsc->header.h      = height;
  out_dsc->header.stride = width * 2U;  // RGB565: 2 bytes/pixel
  out_dsc->data_size     = (uint32_t) buf_size;
  out_dsc->data          = buf;

  // 4. Create LVGL image object and attach descriptor
  *out_obj = lv_image_create(parent);
  if (*out_obj == nullptr)
  {
    LOG_ERR("sys_ui_image_draw: lv_image_create failed");
    heap_caps_free(buf);
    return STATUS_ERROR;
  }

  lv_image_set_src(*out_obj, out_dsc);
  lv_obj_set_pos(*out_obj, x, y);

  LOG_DBG("sys_ui_image_draw: OK %s (%ux%u) pos(%d,%d)", path, width, height, x, y);
  return STATUS_OK;
}

static void sys_ui_main_screen_create(void)
{
  ui_ctx.widgets.main_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  // Top buttons
  ui_ctx.widgets.settings_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.main_screen, SYS_UI_SETTINGS_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_SETTINGS_BTN_W, SYS_UI_CTRL_BTN_H,
    SYS_UI_SETTINGS_BTN_LABEL, SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
  ui_ctx.widgets.out_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.main_screen, SYS_UI_OUT_BTN_X, SYS_UI_CTRL_BTN_Y, SYS_UI_OUT_BTN_W,
                                SYS_UI_CTRL_BTN_H, SYS_UI_OUT_BTN_LABEL, SYS_UI_COLOR_WARNING, SYS_UI_COLOR_BG);

  // Speedometer arc
  ui_ctx.widgets.speedometer_arc = sys_ui_widget_create_arc(
    ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX, SYS_UI_SPEEDO_CY, SYS_UI_SPEEDO_OUTER_R, SYS_UI_SPEEDO_START_ANGLE,
    SYS_UI_SPEEDO_END_ANGLE, SYS_UI_SPEEDO_BG_COLOR, SYS_UI_COLOR_SUCCESS);
  ui_ctx.widgets.speed_label = sys_ui_widget_create_label(
    ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX + SYS_UI_SPEED_LABEL_X_OFFSET,
    SYS_UI_SPEEDO_CY + SYS_UI_SPEED_LABEL_Y_OFFSET, SYS_UI_SPEED_LABEL_INIT, SYS_UI_COLOR_TEXT, &lv_font_montserrat_28);
  lv_obj_set_width(ui_ctx.widgets.speed_label, SYS_UI_SPEED_LABEL_W);
  lv_obj_set_style_text_align(ui_ctx.widgets.speed_label, LV_TEXT_ALIGN_CENTER, 0);
  ui_ctx.widgets.speed_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_SPEEDO_CX + SYS_UI_SPEED_UNIT_X_OFFSET,
                               SYS_UI_SPEEDO_CY + SYS_UI_SPEED_UNIT_Y_OFFSET, SYS_UI_SPEED_UNIT_LABEL,
                               SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);

  // Compass panel
  sys_ui_widget_create_panel(ui_ctx.widgets.main_screen, SYS_UI_MAP_PANEL_X, SYS_UI_MAP_PANEL_Y, SYS_UI_MAP_PANEL_W,
                             SYS_UI_MAP_PANEL_H, SYS_UI_COMPASS_PANEL_BG_COLOR);
  static lv_point_precise_t needle_points[2] = {
    { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
    { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY - SYS_UI_COMPASS_R + SYS_UI_COMPASS_NEEDLE_INSET },
  };
  ui_ctx.widgets.compass_needle = lv_line_create(ui_ctx.widgets.main_screen);
  lv_line_set_points(ui_ctx.widgets.compass_needle, needle_points, 2);
  lv_obj_set_style_line_width(ui_ctx.widgets.compass_needle, SYS_UI_COMPASS_NEEDLE_WIDTH, 0);
  lv_obj_set_style_line_color(ui_ctx.widgets.compass_needle, lv_color_hex(SYS_UI_COLOR_ACCENT), 0);
  ui_ctx.widgets.compass_deg_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_HEADING_TEXT_X, SYS_UI_HEADING_TEXT_Y,
                               SYS_UI_COMPASS_DEG_LABEL_INIT, SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);
  ui_ctx.widgets.compass_dir_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_COMPASS_DEG_X, SYS_UI_COMPASS_DEG_Y,
                               SYS_UI_COMPASS_DIR_LABEL_INIT, SYS_UI_COLOR_ACCENT, &lv_font_montserrat_18);

  // Warning label
  ui_ctx.widgets.warning_out_of_zone_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_WARNING_LABEL_X, SYS_UI_WARNING_LABEL_Y,
                               SYS_UI_WARNING_LABEL_TEXT, SYS_UI_WARNING_LABEL_TEXT_COLOR, SYS_UI_WARNING_LABEL_FONT);

  ui_ctx.widgets.warn_rental_limit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_NOTI_RENTAL_LIMIT_LABEL_X,
                               SYS_UI_NOTI_RENTAL_LIMIT_LABEL_Y, SYS_UI_NOTI_RENTAL_LIMIT_LABEL_TEXT,
                               SYS_UI_NOTI_RENTAL_LIMIT_LABEL_TEXT_COLOR, SYS_UI_NOTI_RENTAL_LIMIT_LABEL_FONT);

  ui_ctx.widgets.warn_add_fund_label =
    sys_ui_widget_create_label(ui_ctx.widgets.main_screen, SYS_UI_NOTI_WARN_ADD_FUND_LABEL_X,
                               SYS_UI_NOTI_WARN_ADD_FUND_LABEL_Y, SYS_UI_NOTI_WARN_ADD_FUND_LABEL_TEXT,
                               SYS_UI_NOTI_WARN_ADD_FUND_LABEL_TEXT_COLOR, SYS_UI_NOTI_WARN_ADD_FUND_LABEL_FONT);

  ui_ctx.widgets.should_add_fund_panel = sys_ui_widget_create_label(
    ui_ctx.widgets.main_screen, SYS_UI_SHOULD_ADD_FUND_LABEL_X, SYS_UI_SHOULD_ADD_FUND_LABEL_Y,
    SYS_UI_SHOULD_ADD_FUND_LABEL_TEXT, SYS_UI_SHOULD_ADD_FUND_LABEL_TEXT_COLOR, SYS_UI_SHOULD_ADD_FUND_LABEL_FONT);

  ui_ctx.current_noti_label = SYS_UI_NOTI_LABEL_MAX;
  sys_ui_update_notification_label(SYS_UI_NOTI_LABEL_NONE);

  // Time card
  ui_ctx.widgets.time_card =
    sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_TIME_CARD_Y, SYS_UI_CARD_W,
                              SYS_UI_TIME_CARD_H, ui_ctx.card_bg, SYS_UI_COLOR_PRIMARY);
  sys_ui_widget_create_label(ui_ctx.widgets.time_card, SYS_UI_TIME_CARD_LABEL_X, SYS_UI_TIME_CARD_LABEL_Y,
                             SYS_UI_TIME_CARD_LABEL, SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_10);
  ui_ctx.widgets.time_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, SYS_UI_TIME_LABEL_X, SYS_UI_TIME_LABEL_Y,
                               SYS_UI_TIME_LABEL_INIT, SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_20);
  ui_ctx.widgets.time_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.time_card, SYS_UI_TIME_UNIT_X, SYS_UI_TIME_UNIT_Y, SYS_UI_TIME_UNIT_LABEL,
                               SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Distance card
  ui_ctx.widgets.distance_card =
    sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_DIST_CARD_Y, SYS_UI_CARD_W,
                              SYS_UI_DIST_CARD_H, ui_ctx.card_bg, SYS_UI_COLOR_ACCENT);
  sys_ui_widget_create_label(ui_ctx.widgets.distance_card, SYS_UI_DISTANCE_TITLE_X, SYS_UI_DISTANCE_TITLE_Y,
                             SYS_UI_DISTANCE_TITLE_LABEL, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.distance_label =
    sys_ui_widget_create_label(ui_ctx.widgets.distance_card, SYS_UI_DISTANCE_LABEL_X, SYS_UI_DISTANCE_LABEL_Y,
                               SYS_UI_DISTANCE_LABEL_INIT, SYS_UI_COLOR_ACCENT, &lv_font_montserrat_16);
  ui_ctx.widgets.distance_unit_label =
    sys_ui_widget_create_label(ui_ctx.widgets.distance_card, SYS_UI_DISTANCE_UNIT_X, SYS_UI_DISTANCE_UNIT_Y,
                               SYS_UI_DISTANCE_UNIT_LABEL, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Environment card
  ui_ctx.widgets.env_card =
    sys_ui_widget_create_card(ui_ctx.widgets.main_screen, SYS_UI_CARD_X, SYS_UI_ENV_CARD_Y, SYS_UI_CARD_W,
                              SYS_UI_ENV_CARD_H, ui_ctx.card_bg, SYS_UI_COLOR_TEXT_DIM);
  sys_ui_widget_create_label(ui_ctx.widgets.env_card, SYS_UI_ENV_TITLE_X, SYS_UI_ENV_TITLE_Y, SYS_UI_ENV_TITLE_LABEL,
                             SYS_UI_COLOR_TEXT, &lv_font_montserrat_10);
  ui_ctx.widgets.temp_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_card, SYS_UI_TEMP_LABEL_X, SYS_UI_TEMP_LABEL_Y,
                               SYS_UI_TEMP_LABEL_INIT, SYS_UI_COLOR_WARNING, &lv_font_montserrat_12);
  ui_ctx.widgets.humidity_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_card, SYS_UI_HUM_LABEL_X, SYS_UI_HUM_LABEL_Y, SYS_UI_HUM_LABEL_INIT,
                               SYS_UI_COLOR_PRIMARY, &lv_font_montserrat_12);
  ui_ctx.widgets.aqi_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_card, SYS_UI_AQI_LABEL_X, SYS_UI_AQI_LABEL_Y, SYS_UI_AQI_LABEL_INIT,
                               SYS_UI_COLOR_SUCCESS, &lv_font_montserrat_10);
}

static void sys_ui_main_screen_update_speed(int speed_kph)
{
  speed_kph = SYS_UI_CLAMP(speed_kph, SYS_UI_SPEED_MIN_KPH, SYS_UI_SPEED_MAX_KPH);

  if (ui_ctx.prev_speed_int == speed_kph)
  {
    return;
  }
  ui_ctx.prev_speed_int = speed_kph;

  if (ui_ctx.widgets.speedometer_arc != nullptr)
  {
    int    arc_value = (speed_kph * SYS_UI_SPEED_ARC_MAX_VALUE) / SYS_UI_SPEED_MAX_KPH;
    size_t arc_color = SYS_UI_COLOR_DANGER;
    if (speed_kph <= SYS_UI_SPEED_SAFE_MAX_KPH)
      arc_color = SYS_UI_COLOR_SUCCESS;
    else if (speed_kph <= SYS_UI_SPEED_WARN_MAX_KPH)
      arc_color = SYS_UI_COLOR_WARNING;
    sys_ui_widget_set_arc_value(ui_ctx.widgets.speedometer_arc, arc_value);
    sys_ui_widget_set_arc_color(ui_ctx.widgets.speedometer_arc, arc_color);
  }

  if (ui_ctx.widgets.speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.speed_label, SYS_UI_SPEED_LABEL_FORMAT, speed_kph);
  }
}

static void sys_ui_main_screen_update_time(int hours, int minutes, int seconds)
{
  if (ui_ctx.widgets.time_label == nullptr)
  {
    return;
  }

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.time_label, SYS_UI_TIME_LABEL_FORMAT, hours, minutes, seconds);
  sys_ui_widget_set_label_color(ui_ctx.widgets.time_label, SYS_UI_COLOR_SUCCESS);
}

static void sys_ui_main_screen_update_distance(float distance_km)
{
  if (ui_ctx.widgets.distance_label == nullptr)
  {
    return;
  }

  sys_ui_widget_set_label_text_format(ui_ctx.widgets.distance_label, SYS_UI_DISTANCE_LABEL_FORMAT, distance_km);
}

static void sys_ui_main_screen_update_env(float temperature_C, float humidity, float dust_value)
{
  if (ui_ctx.widgets.temp_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.temp_label, SYS_UI_TEMP_LABEL_FORMAT, temperature_C);
  }

  if (ui_ctx.widgets.humidity_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.humidity_label, SYS_UI_HUM_LABEL_FORMAT, humidity);
  }

  if (ui_ctx.widgets.aqi_label != nullptr)
  {
    bsp_dust_aqi_level_t aqi_level = bsp_dust_sensor_get_aqi_level(dust_value);
    char                *status;
    size_t               aqi_color;

    switch (aqi_level)
    {
    case BSP_DUST_AQI_EXCELLENT:
    case BSP_DUST_AQI_GOOD:
    {
      status    = SYS_UI_AQI_STATUS_GOOD;
      aqi_color = SYS_UI_COLOR_SUCCESS;
      break;
    }
    case BSP_DUST_AQI_MODERATE:
    case BSP_DUST_AQI_POOR:
    {
      status    = SYS_UI_AQI_STATUS_FAIR;
      aqi_color = SYS_UI_COLOR_WARNING;
      break;
    }
    case BSP_DUST_AQI_UNHEALTHY:
    case BSP_DUST_AQI_HAZARDOUS:
    {
      status    = SYS_UI_AQI_STATUS_POOR;
      aqi_color = SYS_UI_COLOR_DANGER;
      break;
    }
    default:
    {
      break;
    }
    }

    sys_ui_widget_set_label_text_format(ui_ctx.widgets.aqi_label, SYS_UI_AQI_LABEL_FORMAT, dust_value, status);
    sys_ui_widget_set_label_color(ui_ctx.widgets.aqi_label, aqi_color);
  }

  size_t ts = OS_GET_TICK();
  sys_ui_log_temp_sample(temperature_C, ts);
  sys_ui_log_hum_sample(humidity, ts);
  sys_ui_log_dust_sample((float) dust_value, ts);
}

static void sys_ui_main_screen_update_compass(void)
{
  if (ui_ctx.widgets.compass_deg_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.compass_deg_label, SYS_UI_COMPASS_DEG_FORMAT,
                                        ui_ctx.compass_heading_deg);
  }

  if (ui_ctx.widgets.compass_dir_label != nullptr)
  {
    static const char *dirs[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    int                idx =
      (int) ((ui_ctx.compass_heading_deg + SYS_UI_COMPASS_DIR_OFFSET_DEG) / SYS_UI_COMPASS_DIR_SECTOR_DEG) & 0x7;
    sys_ui_widget_set_label_text(ui_ctx.widgets.compass_dir_label, dirs[idx]);
  }

  if (ui_ctx.widgets.compass_needle != nullptr)
  {
    float              rad = (ui_ctx.compass_heading_deg - 90.0f) * (float) PI / 180.0f;
    int                x2  = SYS_UI_COMPASS_CX + (int) ((SYS_UI_COMPASS_R - SYS_UI_COMPASS_NEEDLE_INSET) * cosf(rad));
    int                y2  = SYS_UI_COMPASS_CY + (int) ((SYS_UI_COMPASS_R - SYS_UI_COMPASS_NEEDLE_INSET) * sinf(rad));
    lv_point_precise_t points[2] = {
      { SYS_UI_COMPASS_CX, SYS_UI_COMPASS_CY },
      { x2, y2 },
    };
    lv_line_set_points(ui_ctx.widgets.compass_needle, points, 2);
  }
}

static void sys_ui_main_screen_update_warning(void)
{
  if (ui_ctx.widgets.warning_out_of_zone_label == nullptr)
  {
    return;
  }

  if (ui_ctx.is_warning_out_of_zone_active)
  {
    lv_obj_clear_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
  }
  else
  {
    lv_obj_add_flag(ui_ctx.widgets.warning_out_of_zone_label, LV_OBJ_FLAG_HIDDEN);
  }
}

static void sys_ui_main_screen_update_speed_n_distance(void)
{
  ui_ctx.current_speed       = ui_ctx.fusion.velocity_kmh;
  ui_ctx.max_speed           = (ui_ctx.max_speed > ui_ctx.current_speed) ? ui_ctx.max_speed : ui_ctx.current_speed;
  ui_ctx.distance_km         = ui_ctx.fusion.distance_m / 1000.0f;
  ui_ctx.compass_heading_deg = ui_ctx.fusion.heading_deg;
  sys_ui_main_screen_update_speed((int) ui_ctx.current_speed);
  sys_ui_main_screen_update_distance(ui_ctx.distance_km);
}

static void sys_ui_main_screen_update_countup(void)
{
  ui_ctx.active_seconds++;
  if (ui_ctx.active_seconds >= 60)
  {
    ui_ctx.active_seconds = 0;
    ui_ctx.active_minutes++;
  }
  if (ui_ctx.active_minutes >= 60)
  {
    ui_ctx.active_minutes = 0;
    ui_ctx.active_hours++;
  }
}

static void sys_ui_main_screen_cb_settings_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_settings_btn");
  sys_ui_change_screen(SYS_UI_VIEW_SETTINGS);
}

static void sys_ui_main_screen_cb_out_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_out_btn");
  sys_ui_change_screen(SYS_UI_VIEW_OUT);
}

static void sys_ui_main_screen_cb_time_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_time_card");
  sys_ui_change_screen(SYS_UI_VIEW_TIME);
}

static void sys_ui_main_screen_cb_distance_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_distance_card");
  sys_ui_change_screen(SYS_UI_VIEW_DISTANCE);
}

static void sys_ui_main_screen_cb_env_card(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_main_screen_cb_env_card");
  sys_ui_change_screen(SYS_UI_VIEW_TEMPERATURE);
}

static void sys_ui_settings_screen_create(void)
{
  ui_ctx.widgets.settings_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.settings_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.settings_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, SYS_UI_SETTINGS_BACK_LABEL, SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);
  ui_ctx.widgets.settings_title =
    sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, SYS_UI_SETTINGS_TITLE_X, SYS_UI_SETTINGS_TITLE_Y,
                               SYS_UI_SETTINGS_TITLE_LABEL, SYS_UI_COLOR_TEXT, &lv_font_montserrat_18);

  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, SYS_UI_SETTINGS_BRIGHTNESS_X, SYS_UI_SETTINGS_BRIGHTNESS_Y,
                             SYS_UI_SETTINGS_BRIGHTNESS_TEXT, SYS_UI_COLOR_TEXT_DIM, nullptr);
  ui_ctx.widgets.brightness_slider = sys_ui_widget_create_slider(
    ui_ctx.widgets.settings_screen, SYS_UI_SETTINGS_SLIDER_X, SYS_UI_SETTINGS_SLIDER_Y, SYS_UI_SETTINGS_SLIDER_W,
    SYS_UI_SETTINGS_SLIDER_H, SYS_UI_SETTINGS_SLIDER_MIN, SYS_UI_SETTINGS_SLIDER_MAX, ui_ctx.brightness_percent,
    SYS_UI_SETTINGS_SLIDER_BG_COLOR, SYS_UI_COLOR_PRIMARY);
  ui_ctx.widgets.brightness_label =
    sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, SYS_UI_BRIGHTNESS_LABEL_X, SYS_UI_BRIGHTNESS_LABEL_Y, "",
                               SYS_UI_COLOR_TEXT, nullptr);
  sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, SYS_UI_BRIGHTNESS_LABEL_FORMAT,
                                      ui_ctx.brightness_percent);

  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen, SYS_UI_SETTINGS_BG_X, SYS_UI_SETTINGS_BG_Y,
                             SYS_UI_SETTINGS_BG_TEXT, SYS_UI_COLOR_TEXT_DIM, nullptr);

  // clang-format off
#define INFO(idx, r, g, b, lbl)                                                                                    \
  ui_ctx.widgets.color_btns[idx] = lv_btn_create(ui_ctx.widgets.settings_screen);                                 \
  lv_obj_set_pos(ui_ctx.widgets.color_btns[idx], SYS_UI_SWATCH_X + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y);\
  lv_obj_set_size(ui_ctx.widgets.color_btns[idx], SYS_UI_SWATCH_SIZE, SYS_UI_SWATCH_SIZE);                        \
  lv_obj_set_style_bg_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(BSP_DISPLAY_RGB_TO_HEX(r, g, b)), 0);    \
  lv_obj_set_style_border_color(ui_ctx.widgets.color_btns[idx], lv_color_hex(SYS_UI_COLOR_TEXT_DIM), 0);          \
  lv_obj_set_style_border_width(ui_ctx.widgets.color_btns[idx], 1, 0);                                            \
  lv_obj_set_style_radius(ui_ctx.widgets.color_btns[idx], 4, 0);                                                  \
  sys_ui_widget_create_label(ui_ctx.widgets.settings_screen,                                                      \
    SYS_UI_SWATCH_X + (idx) * SYS_UI_SWATCH_SPAN, SYS_UI_SWATCH_ROW_Y + SYS_UI_SWATCH_SIZE + SYS_UI_SWATCH_LABEL_Y_OFFSET,\
    lbl, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  SYS_UI_BG_COLOR_TABLE(INFO)
#undef INFO
  // clang-format on
}

static void sys_ui_settings_screen_draw(void)
{
  if (ui_ctx.widgets.settings_screen == nullptr)
  {
    sys_ui_settings_screen_create();
  }

  if (ui_ctx.widgets.brightness_slider != nullptr)
  {
    lv_slider_set_value(ui_ctx.widgets.brightness_slider, ui_ctx.brightness_percent, LV_ANIM_OFF);
  }
  if (ui_ctx.widgets.brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, SYS_UI_BRIGHTNESS_LABEL_FORMAT,
                                        ui_ctx.brightness_percent);
  }

  sys_ui_show_screen(ui_ctx.widgets.settings_screen);
}

static void sys_ui_settings_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_settings_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_settings_screen_cb_brightness_slider(lv_event_t *event)
{
  lv_obj_t *slider          = (lv_obj_t *) lv_event_get_target(event);
  ui_ctx.brightness_percent = lv_slider_get_value(slider);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);

  if (ui_ctx.widgets.brightness_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.brightness_label, SYS_UI_BRIGHTNESS_LABEL_FORMAT,
                                        ui_ctx.brightness_percent);
  }

  LOG_DBG("sys_ui_settings_screen_cb_brightness_slider: %d%%", ui_ctx.brightness_percent);
}

static void sys_ui_settings_screen_cb_color_btn(lv_event_t *event)
{
  int btn_index = (int) (intptr_t) lv_event_get_user_data(event);

#define INFO(idx, r, g, b, lbl) BSP_DISPLAY_RGB_TO_HEX(r, g, b),
  static const size_t bg_colors[SYS_UI_BG_COLOR_COUNT] = { SYS_UI_BG_COLOR_TABLE(INFO) };
#undef INFO

  if (btn_index >= 0 && btn_index < SYS_UI_BG_COLOR_COUNT)
  {
    ui_ctx.background_color = bg_colors[btn_index];
    sys_ui_update_theme_colors();
    ui_ctx.pending_settings_redraw = true;
    ui_ctx.pending_main_redraw     = true;
    LOG_DBG("sys_ui_settings_screen_cb_color_btn: index=%d color=0x%06X luma=%s", btn_index,
            (unsigned) ui_ctx.background_color, (ui_ctx.text_color == SYS_UI_WIDGET_COLOR_WHITE) ? "dark" : "light");
  }
}

static void sys_ui_out_screen_create(void)
{
  ui_ctx.widgets.out_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  ui_ctx.widgets.out_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.out_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, SYS_UI_BACK_BTN_LABEL, SYS_UI_BACK_BTN_COLOR, SYS_UI_COLOR_BG);
  sys_ui_widget_create_label(ui_ctx.widgets.out_screen, SYS_UI_OUT_LABEL_X, SYS_UI_OUT_LABEL_Y, SYS_UI_OUT_LABEL_TEXT,
                             SYS_UI_COLOR_TEXT, nullptr);
  ui_ctx.widgets.out_stop_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.out_screen, SYS_UI_STOP_BTN_X, SYS_UI_STOP_BTN_Y, SYS_UI_STOP_BTN_W,
                                SYS_UI_STOP_BTN_H, SYS_UI_STOP_BTN_LABEL, SYS_UI_STOP_BTN_COLOR, SYS_UI_COLOR_BG);

  ui_ctx.widgets.out_pause_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.out_screen, SYS_UI_PAUSE_BTN_X, SYS_UI_PAUSE_BTN_Y, SYS_UI_PAUSE_BTN_W,
                                SYS_UI_PAUSE_BTN_H, SYS_UI_PAUSE_BTN_LABEL, SYS_UI_PAUSE_BTN_COLOR, SYS_UI_COLOR_BG);
}

static void sys_ui_out_screen_draw(void)
{
  if (ui_ctx.widgets.out_screen == nullptr)
  {
    sys_ui_out_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.out_screen);
}

static void sys_ui_out_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_out_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_out_screen_cb_stop_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_out_screen_cb_stop_btn");
  sys_manager_write_event(SYS_MANAGER_EVT_USER_LOCK);
}

static void sys_ui_out_screen_cb_pause_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_out_screen_cb_pause_btn");
  sys_manager_write_event(SYS_MANAGER_EVT_USER_PAUSE);
}

static void sys_ui_time_screen_create(void)
{
  if (ui_ctx.widgets.time_history_screen != nullptr)
  {
    return;
  }

  ui_ctx.widgets.time_history_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(ui_ctx.widgets.time_history_screen, lv_color_hex(ui_ctx.background_color), LV_PART_MAIN);

  ui_ctx.widgets.time_back_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.time_history_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W, SYS_UI_BACK_BTN_H,
    SYS_UI_BACK_BTN_LABEL, SYS_UI_BACK_BTN_COLOR, SYS_UI_BACK_BTN_TEXT_COLOR);

  ui_ctx.widgets.time_history_title =
    sys_ui_widget_create_label(ui_ctx.widgets.time_history_screen, SYS_UI_EXTEND_LABEL_X, SYS_UI_EXTEND_LABEL_Y,
                               SYS_UI_EXTEND_LABEL, SYS_UI_COLOR_TEXT, SYS_UI_EXTEND_LABEL_FONT);

  ui_ctx.widgets.time_history_time_label = sys_ui_widget_create_label(
    ui_ctx.widgets.time_history_screen, 80, 80, "00:00:00", SYS_UI_COLOR_TEXT, &lv_font_montserrat_28);

  ui_ctx.widgets.time_history_date_label = sys_ui_widget_create_label(
    ui_ctx.widgets.time_history_screen, 80, 120, "01/01/2024", SYS_UI_COLOR_TEXT, &lv_font_montserrat_28);
}

static void sys_ui_time_screen_update(void)
{
  if (ui_ctx.widgets.time_history_screen == nullptr)
  {
    sys_ui_time_screen_create();
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", ui_ctx.unlock_time.hour, ui_ctx.unlock_time.minute,
           ui_ctx.unlock_time.second);
  lv_label_set_text(ui_ctx.widgets.time_history_time_label, buf);

  snprintf(buf, sizeof(buf), "%02d/%02d/%04d", ui_ctx.unlock_time.day, ui_ctx.unlock_time.month,
           ui_ctx.unlock_time.year);
  lv_label_set_text(ui_ctx.widgets.time_history_date_label, buf);

  sys_ui_show_screen(ui_ctx.widgets.time_history_screen);
}

static void sys_ui_time_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_time_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_fusion_screen_create(void)
{
  ui_ctx.widgets.fusion_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  // Back button + title
  ui_ctx.widgets.distance_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.fusion_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, SYS_UI_BACK_BTN_LABEL, SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);
  ui_ctx.widgets.distance_title =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_LABEL_X, SYS_UI_FUSION_LABEL_Y,
                               SYS_UI_FUSION_LABEL_TEXT, SYS_UI_COLOR_TEXT, SYS_UI_FUSION_LABEL_FONT);

  // Stats labels (max speed / avg speed)
  ui_ctx.widgets.max_speed_lable =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_MAX_SPEED_X, SYS_UI_FUSION_MAX_SPEED_Y,
                               SYS_UI_FUSION_MAX_SPEED_INIT, SYS_UI_COLOR_TEXT, nullptr);
  ui_ctx.widgets.avg_speed_label =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_AVG_SPEED_X, SYS_UI_FUSION_AVG_SPEED_Y,
                               SYS_UI_FUSION_AVG_SPEED_INIT, SYS_UI_COLOR_TEXT, nullptr);

  // Speed history chart
  ui_ctx.widgets.speed_chart = sys_ui_widget_create_chart(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X, SYS_UI_FUSION_CHART_Y, SYS_UI_FUSION_CHART_W,
    SYS_UI_FUSION_CHART_H, SYS_UI_FUSION_CHART_POINTS, SYS_UI_FUSION_CHART_COLOR);
  ui_ctx.widgets.speed_series =
    sys_ui_widget_add_chart_series(ui_ctx.widgets.speed_chart, SYS_UI_FUSION_CHART_POINT_COLOR);

  // Y-axis labels (max / mid / min speed)
  ui_ctx.widgets.fusion_y_max_label =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_Y_LABEL_X, SYS_UI_FUSION_CHART_Y,
                               SYS_UI_FUSION_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.fusion_y_mid_label = sys_ui_widget_create_label(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_Y_LABEL_X, SYS_UI_FUSION_CHART_Y + SYS_UI_FUSION_CHART_H / 2 - 5,
    SYS_UI_FUSION_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.fusion_y_min_label = sys_ui_widget_create_label(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_Y_LABEL_X, SYS_UI_FUSION_CHART_Y + SYS_UI_FUSION_CHART_H - 10,
    SYS_UI_FUSION_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // X-axis labels (time span)
  ui_ctx.widgets.fusion_x_start_label =
    sys_ui_widget_create_label(ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X, SYS_UI_FUSION_X_LABEL_Y,
                               SYS_UI_FUSION_X_START_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.fusion_x_end_label = sys_ui_widget_create_label(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X + SYS_UI_FUSION_CHART_W - SYS_UI_FUSION_X_END_OFFSET,
    SYS_UI_FUSION_X_LABEL_Y, SYS_UI_FUSION_X_END_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Zoom / pan buttons
  ui_ctx.widgets.fusion_zoom_minus_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X, SYS_UI_FUSION_BTN_Y, SYS_UI_FUSION_BTN_W, SYS_UI_FUSION_BTN_H,
    SYS_UI_FUSION_ZOOM_MINUS_LABEL, SYS_UI_FUSION_INACTIVE_BTN_BG, SYS_UI_COLOR_TEXT);
  ui_ctx.widgets.fusion_zoom_plus_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X + SYS_UI_FUSION_BTN_W + SYS_UI_FUSION_BTN_GAP,
    SYS_UI_FUSION_BTN_Y, SYS_UI_FUSION_BTN_W, SYS_UI_FUSION_BTN_H, SYS_UI_FUSION_ZOOM_PLUS_LABEL, SYS_UI_COLOR_SUCCESS,
    SYS_UI_COLOR_BG);
  ui_ctx.widgets.fusion_pan_left_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X + 2 * (SYS_UI_FUSION_BTN_W + SYS_UI_FUSION_BTN_GAP),
    SYS_UI_FUSION_BTN_Y, SYS_UI_FUSION_BTN_W, SYS_UI_FUSION_BTN_H, SYS_UI_FUSION_PAN_LEFT_LABEL, SYS_UI_COLOR_PRIMARY,
    SYS_UI_COLOR_BG);
  ui_ctx.widgets.fusion_pan_right_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.fusion_screen, SYS_UI_FUSION_CHART_X + 3 * (SYS_UI_FUSION_BTN_W + SYS_UI_FUSION_BTN_GAP),
    SYS_UI_FUSION_BTN_Y, SYS_UI_FUSION_BTN_W, SYS_UI_FUSION_BTN_H, SYS_UI_FUSION_PAN_RIGHT_LABEL, SYS_UI_COLOR_PRIMARY,
    SYS_UI_COLOR_BG);
}

static void sys_ui_fusion_screen_update(void)
{
  if (ui_ctx.widgets.fusion_screen == nullptr)
  {
    sys_ui_fusion_screen_create();
  }

  // Update stats labels
  size_t now   = OS_GET_TICK();
  float  hours = (float) (now - ui_ctx.session_start_ms) / 3600000.0f;
  if (hours < SYS_UI_FUSION_MIN_HOURS)
    hours = SYS_UI_FUSION_MIN_HOURS;

  if (ui_ctx.widgets.max_speed_lable != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.max_speed_lable, SYS_UI_FUSION_MAX_SPEED_TEXT, ui_ctx.max_speed);
  }
  if (ui_ctx.widgets.avg_speed_label != nullptr)
  {
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.avg_speed_label, SYS_UI_FUSION_AVG_SPEED_TEXT,
                                        ui_ctx.distance_km / hours);
  }

  // If no data yet, just show screen
  int count = (int) fifo_count(&speed_fifo);
  if (ui_ctx.widgets.speed_chart == nullptr || ui_ctx.widgets.speed_series == nullptr || count <= 1)
  {
    sys_ui_show_screen(ui_ctx.widgets.fusion_screen);
    return;
  }

  int zoom         = ui_ctx.fusion_zoom;
  int graph_offset = SYS_UI_CLAMP(ui_ctx.fusion_graph_offset, 0, count - 1);
  int window       = SYS_UI_CLAMP(count / zoom, 2, count);
  graph_offset     = SYS_UI_CLAMP(graph_offset, 0, count - window);

  float min_v = sys_ui_env_fifo_get(&speed_fifo, graph_offset).value;
  float max_v = min_v;
  for (int i = 0; i < window; ++i)
  {
    float v = sys_ui_env_fifo_get(&speed_fifo, graph_offset + i).value;
    if (v < min_v)
      min_v = v;
    if (v > max_v)
      max_v = v;
  }
  float mid_v = (min_v + max_v) * 0.5f;

  // Y-axis labels
  if (ui_ctx.widgets.fusion_y_max_label != nullptr)
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.fusion_y_max_label, "%.0f", max_v);
  if (ui_ctx.widgets.fusion_y_mid_label != nullptr)
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.fusion_y_mid_label, "%.0f", mid_v);
  if (ui_ctx.widgets.fusion_y_min_label != nullptr)
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.fusion_y_min_label, "%.0f", min_v);

  // X-axis labels
  if (ui_ctx.widgets.fusion_x_start_label != nullptr && ui_ctx.widgets.fusion_x_end_label != nullptr && window > 1)
  {
    size_t t0   = sys_ui_env_fifo_get(&speed_fifo, graph_offset).timestamp;
    size_t t1   = sys_ui_env_fifo_get(&speed_fifo, graph_offset + window - 1).timestamp;
    float  span = (float) (t1 - t0) / 1000.0f;
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.fusion_x_start_label, SYS_UI_FUSION_X_START_INIT);
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.fusion_x_end_label, "%.0fs", span);
  }

  // Plot speed (x10 scale for float precision)
  int y_min_raw = (int) (min_v * 10) - SYS_UI_CHART_Y_PAD_RAW;
  int y_max_raw = (int) (max_v * 10) + SYS_UI_CHART_Y_PAD_RAW;
  if (y_max_raw <= y_min_raw)
    y_max_raw = y_min_raw + SYS_UI_CHART_Y_MIN_SPAN_RAW;
  lv_chart_set_point_count(ui_ctx.widgets.speed_chart, window);
  lv_chart_set_range(ui_ctx.widgets.speed_chart, LV_CHART_AXIS_PRIMARY_Y, y_min_raw, y_max_raw);
  for (int i = 0; i < window; ++i)
  {
    int raw = (int) (sys_ui_env_fifo_get(&speed_fifo, graph_offset + i).value * 10);
    lv_chart_set_value_by_id(ui_ctx.widgets.speed_chart, ui_ctx.widgets.speed_series, i, raw);
  }
  lv_chart_refresh(ui_ctx.widgets.speed_chart);

  sys_ui_show_screen(ui_ctx.widgets.fusion_screen);
}

static void sys_ui_fusion_screen_refresh(void)
{
  int count                  = (int) fifo_count(&speed_fifo);
  int max_pan                = (count > 2) ? (count - 2) : 0;
  ui_ctx.fusion_graph_offset = SYS_UI_CLAMP(ui_ctx.fusion_graph_offset, 0, max_pan);
  sys_ui_fusion_screen_update();
}

static void sys_ui_fusion_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_fusion_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_fusion_screen_cb_zoom_minus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.fusion_zoom > SYS_UI_FUSION_ZOOM_MIN)
  {
    --ui_ctx.fusion_zoom;
    sys_ui_fusion_screen_refresh();
  }
}

static void sys_ui_fusion_screen_cb_zoom_plus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.fusion_zoom < SYS_UI_FUSION_ZOOM_MAX)
  {
    ++ui_ctx.fusion_zoom;
    sys_ui_fusion_screen_refresh();
  }
}

static void sys_ui_fusion_screen_cb_pan_left(lv_event_t *event)
{
  (void) event;
  ui_ctx.fusion_graph_offset -= SYS_UI_PAN_STEP;
  sys_ui_fusion_screen_refresh();
}

static void sys_ui_fusion_screen_cb_pan_right(lv_event_t *event)
{
  (void) event;
  ui_ctx.fusion_graph_offset += SYS_UI_PAN_STEP;
  sys_ui_fusion_screen_refresh();
}

static void sys_ui_temp_screen_create(void)
{
  ui_ctx.widgets.env_screen = sys_ui_widget_create_screen(ui_ctx.background_color);

  // Back button
  ui_ctx.widgets.temp_back_btn =
    sys_ui_widget_create_button(ui_ctx.widgets.env_screen, SYS_UI_BACK_BTN_X, SYS_UI_BACK_BTN_Y, SYS_UI_BACK_BTN_W,
                                SYS_UI_BACK_BTN_H, SYS_UI_BACK_BTN_LABEL, SYS_UI_COLOR_ACCENT, SYS_UI_COLOR_BG);

  // Tab selector buttons (TEMP/HUM/DUST) — top row, right of BACK (all use white text)
  ui_ctx.widgets.env_tab_temp_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_TAB_TEMP_X, SYS_UI_ENV_TAB_Y, SYS_UI_ENV_TAB_W, SYS_UI_ENV_TAB_H,
    SYS_UI_ENV_TAB_TEMP_LABEL, SYS_UI_ENV_TAB_ACTIVE_COLOR, SYS_UI_COLOR_TEXT);
  ui_ctx.widgets.env_tab_hum_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_TAB_HUM_X, SYS_UI_ENV_TAB_Y, SYS_UI_ENV_TAB_W, SYS_UI_ENV_TAB_H,
    SYS_UI_ENV_TAB_HUM_LABEL, SYS_UI_ENV_TAB_INACTIVE_COLOR, SYS_UI_COLOR_TEXT);
  ui_ctx.widgets.env_tab_dust_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_TAB_DUST_X, SYS_UI_ENV_TAB_Y, SYS_UI_ENV_TAB_W, SYS_UI_ENV_TAB_H,
    SYS_UI_ENV_TAB_DUST_LABEL, SYS_UI_ENV_TAB_INACTIVE_COLOR, SYS_UI_COLOR_TEXT);

  // Charts (only temp visible initially, hum and dust hidden)
  ui_ctx.widgets.temp_chart =
    sys_ui_widget_create_chart(ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X, SYS_UI_ENV_GRAPH_Y, SYS_UI_ENV_GRAPH_W,
                               SYS_UI_ENV_GRAPH_H, SYS_UI_ENV_GRAPH_POINTS, SYS_UI_ENV_GRAPH_COLOR);
  ui_ctx.widgets.temp_series = sys_ui_widget_add_chart_series(ui_ctx.widgets.temp_chart, SYS_UI_COLOR_ACCENT);

  ui_ctx.widgets.hum_chart =
    sys_ui_widget_create_chart(ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X, SYS_UI_ENV_GRAPH_Y, SYS_UI_ENV_GRAPH_W,
                               SYS_UI_ENV_GRAPH_H, SYS_UI_ENV_GRAPH_POINTS, SYS_UI_ENV_GRAPH_COLOR);
  ui_ctx.widgets.hum_series = sys_ui_widget_add_chart_series(ui_ctx.widgets.hum_chart, SYS_UI_COLOR_PRIMARY);
  lv_obj_add_flag(ui_ctx.widgets.hum_chart, LV_OBJ_FLAG_HIDDEN);

  ui_ctx.widgets.dust_chart =
    sys_ui_widget_create_chart(ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X, SYS_UI_ENV_GRAPH_Y, SYS_UI_ENV_GRAPH_W,
                               SYS_UI_ENV_GRAPH_H, SYS_UI_ENV_GRAPH_POINTS, SYS_UI_ENV_GRAPH_COLOR);
  ui_ctx.widgets.dust_series = sys_ui_widget_add_chart_series(ui_ctx.widgets.dust_chart, SYS_UI_COLOR_WARNING);
  lv_obj_add_flag(ui_ctx.widgets.dust_chart, LV_OBJ_FLAG_HIDDEN);

  // Y-axis labels (max / mid / min) — left of chart
  ui_ctx.widgets.env_y_max_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_screen, SYS_UI_ENV_Y_LABEL_X, SYS_UI_ENV_GRAPH_Y,
                               SYS_UI_ENV_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.env_y_mid_label = sys_ui_widget_create_label(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_Y_LABEL_X, SYS_UI_ENV_GRAPH_Y + SYS_UI_ENV_GRAPH_H / 2 - 5,
    SYS_UI_ENV_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.env_y_min_label = sys_ui_widget_create_label(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_Y_LABEL_X, SYS_UI_ENV_GRAPH_Y + SYS_UI_ENV_GRAPH_H - 10,
    SYS_UI_ENV_Y_LABEL_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // X-axis labels (start / end time) — below chart
  ui_ctx.widgets.env_x_start_label =
    sys_ui_widget_create_label(ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X, SYS_UI_ENV_X_LABEL_Y,
                               SYS_UI_ENV_X_START_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);
  ui_ctx.widgets.env_x_end_label = sys_ui_widget_create_label(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X + SYS_UI_ENV_GRAPH_W - SYS_UI_ENV_X_END_OFFSET, SYS_UI_ENV_X_LABEL_Y,
    SYS_UI_ENV_X_END_INIT, SYS_UI_COLOR_TEXT_DIM, &lv_font_montserrat_10);

  // Bottom control buttons (Zoom- / Zoom+ / < / >)
  ui_ctx.widgets.zoom_minus_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X, SYS_UI_ENV_BTN_Y, SYS_UI_ENV_BTN_W, SYS_UI_ENV_BTN_H,
    SYS_UI_ENV_ZOOM_MINUS_LABEL, SYS_UI_ENV_TAB_INACTIVE_COLOR, SYS_UI_COLOR_TEXT);
  ui_ctx.widgets.zoom_plus_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X + SYS_UI_ENV_BTN_W + SYS_UI_ENV_BTN_GAP, SYS_UI_ENV_BTN_Y,
    SYS_UI_ENV_BTN_W, SYS_UI_ENV_BTN_H, SYS_UI_ENV_ZOOM_PLUS_LABEL, SYS_UI_COLOR_SUCCESS, SYS_UI_COLOR_BG);
  ui_ctx.widgets.pan_left_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X + 2 * (SYS_UI_ENV_BTN_W + SYS_UI_ENV_BTN_GAP), SYS_UI_ENV_BTN_Y,
    SYS_UI_ENV_BTN_W, SYS_UI_ENV_BTN_H, SYS_UI_ENV_PAN_LEFT_LABEL, SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
  ui_ctx.widgets.pan_right_btn = sys_ui_widget_create_button(
    ui_ctx.widgets.env_screen, SYS_UI_ENV_GRAPH_X + 3 * (SYS_UI_ENV_BTN_W + SYS_UI_ENV_BTN_GAP), SYS_UI_ENV_BTN_Y,
    SYS_UI_ENV_BTN_W, SYS_UI_ENV_BTN_H, SYS_UI_ENV_PAN_RIGHT_LABEL, SYS_UI_COLOR_PRIMARY, SYS_UI_COLOR_BG);
}

static void sys_ui_temp_screen_update(void)
{
  // 1. Create screen if not exists
  if (ui_ctx.widgets.env_screen == nullptr)
  {
    sys_ui_temp_screen_create();
  }

  int tab = ui_ctx.env_tab;  // 0=TEMP, 1=HUM, 2=DUST

  // 2. Select which dataset to display
  fifo_t            *hist_fifo = nullptr;
  int                count     = 0;
  lv_obj_t          *chart     = nullptr;
  lv_chart_series_t *series    = nullptr;

  if (tab == 0)
  {
    hist_fifo = &env_temp_fifo;
    chart     = ui_ctx.widgets.temp_chart;
    series    = ui_ctx.widgets.temp_series;
  }
  else if (tab == 1)
  {
    hist_fifo = &env_hum_fifo;
    chart     = ui_ctx.widgets.hum_chart;
    series    = ui_ctx.widgets.hum_series;
  }
  else
  {
    hist_fifo = &env_dust_fifo;
    chart     = ui_ctx.widgets.dust_chart;
    series    = ui_ctx.widgets.dust_series;
  }
  count = (hist_fifo != nullptr) ? (int) fifo_count(hist_fifo) : 0;

  // 3. Show active chart, hide others
  if (ui_ctx.widgets.temp_chart != nullptr)
  {
    if (tab == 0)
      lv_obj_clear_flag(ui_ctx.widgets.temp_chart, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(ui_ctx.widgets.temp_chart, LV_OBJ_FLAG_HIDDEN);
  }
  if (ui_ctx.widgets.hum_chart != nullptr)
  {
    if (tab == 1)
      lv_obj_clear_flag(ui_ctx.widgets.hum_chart, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(ui_ctx.widgets.hum_chart, LV_OBJ_FLAG_HIDDEN);
  }
  if (ui_ctx.widgets.dust_chart != nullptr)
  {
    if (tab == 2)
      lv_obj_clear_flag(ui_ctx.widgets.dust_chart, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(ui_ctx.widgets.dust_chart, LV_OBJ_FLAG_HIDDEN);
  }

  // 4. If no data, just show screen
  if (chart == nullptr || series == nullptr || count <= 1)
  {
    sys_ui_show_screen(ui_ctx.widgets.env_screen);
    return;
  }

  int zoom         = ui_ctx.temperature_zoom;
  int graph_offset = SYS_UI_CLAMP(ui_ctx.temperature_graph_offset, 0, count - 1);
  int window       = SYS_UI_CLAMP(count / zoom, 2, count);
  graph_offset     = SYS_UI_CLAMP(graph_offset, 0, count - window);

  float min_val = sys_ui_env_fifo_get(hist_fifo, graph_offset).value;
  float max_val = min_val;
  for (int i = 0; i < window; ++i)
  {
    float v = sys_ui_env_fifo_get(hist_fifo, graph_offset + i).value;
    if (v < min_val)
      min_val = v;
    if (v > max_val)
      max_val = v;
  }
  float mid_val = (min_val + max_val) * 0.5f;

  // 5. Update Y-axis labels with units
  if (ui_ctx.widgets.env_y_max_label != nullptr && ui_ctx.widgets.env_y_mid_label != nullptr
      && ui_ctx.widgets.env_y_min_label != nullptr)
  {
    const char *unit = (tab == 0) ? SYS_UI_ENV_UNIT_TEMP : ((tab == 1) ? SYS_UI_ENV_UNIT_HUM : "");
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.env_y_max_label, "%.0f%s", max_val, unit);
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.env_y_mid_label, "%.0f%s", mid_val, unit);
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.env_y_min_label, "%.0f%s", min_val, unit);
  }

  // 6. Update X-axis labels with time span
  if (ui_ctx.widgets.env_x_start_label != nullptr && ui_ctx.widgets.env_x_end_label != nullptr && window > 1)
  {
    size_t t0     = sys_ui_env_fifo_get(hist_fifo, graph_offset).timestamp;
    size_t t1     = sys_ui_env_fifo_get(hist_fifo, graph_offset + window - 1).timestamp;
    float  span_s = (float) (t1 - t0) / 1000.0f;
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.env_x_start_label, SYS_UI_ENV_X_START_INIT);
    sys_ui_widget_set_label_text_format(ui_ctx.widgets.env_x_end_label, "%.0fs", span_s);
  }

  // 7. Plot chart data (use *10 scale for float precision, except dust is already int)
  lv_chart_set_point_count(chart, window);
  int y_min_raw = (tab == 2) ? (int) min_val - SYS_UI_CHART_Y_PAD_RAW : (int) (min_val * 10) - SYS_UI_CHART_Y_PAD_RAW;
  int y_max_raw = (tab == 2) ? (int) max_val + SYS_UI_CHART_Y_PAD_RAW : (int) (max_val * 10) + SYS_UI_CHART_Y_PAD_RAW;
  if (y_max_raw <= y_min_raw)
    y_max_raw = y_min_raw + SYS_UI_CHART_Y_MIN_SPAN_RAW;
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min_raw, y_max_raw);
  for (int i = 0; i < window; ++i)
  {
    float v   = sys_ui_env_fifo_get(hist_fifo, graph_offset + i).value;
    int   raw = (tab == 2) ? (int) v : (int) (v * 10);
    lv_chart_set_value_by_id(chart, series, i, raw);
  }
  lv_chart_refresh(chart);

  sys_ui_show_screen(ui_ctx.widgets.env_screen);
}

static void sys_ui_temp_screen_update_tab_colors(void)
{
  int tab = ui_ctx.env_tab;
  if (ui_ctx.widgets.env_tab_temp_btn != nullptr)
  {
    lv_obj_set_style_bg_color(ui_ctx.widgets.env_tab_temp_btn,
                              lv_color_hex(tab == 0 ? SYS_UI_ENV_TAB_ACTIVE_COLOR : SYS_UI_ENV_TAB_INACTIVE_COLOR), 0);
  }
  if (ui_ctx.widgets.env_tab_hum_btn != nullptr)
  {
    lv_obj_set_style_bg_color(ui_ctx.widgets.env_tab_hum_btn,
                              lv_color_hex(tab == 1 ? SYS_UI_ENV_TAB_ACTIVE_COLOR : SYS_UI_ENV_TAB_INACTIVE_COLOR), 0);
  }
  if (ui_ctx.widgets.env_tab_dust_btn != nullptr)
  {
    lv_obj_set_style_bg_color(ui_ctx.widgets.env_tab_dust_btn,
                              lv_color_hex(tab == 2 ? SYS_UI_ENV_TAB_ACTIVE_COLOR : SYS_UI_ENV_TAB_INACTIVE_COLOR), 0);
  }
}

static void sys_ui_temp_screen_refresh(void)
{
  fifo_t *fifo    = (ui_ctx.env_tab == 0) ? &env_temp_fifo : ((ui_ctx.env_tab == 1) ? &env_hum_fifo : &env_dust_fifo);
  int     count   = (int) fifo_count(fifo);
  int     max_pan = (count > 2) ? (count - 2) : 0;
  ui_ctx.temperature_graph_offset = SYS_UI_CLAMP(ui_ctx.temperature_graph_offset, 0, max_pan);
  sys_ui_temp_screen_update();
}

static void sys_ui_temp_screen_cb_back_btn(lv_event_t *event)
{
  (void) event;
  LOG_DBG("sys_ui_temp_screen_cb_back_btn");
  sys_ui_change_screen(SYS_UI_VIEW_MAIN);
}

static void sys_ui_temp_screen_cb_tab_temp(lv_event_t *event)
{
  (void) event;
  ui_ctx.env_tab                  = 0;
  ui_ctx.temperature_zoom         = SYS_UI_ENV_ZOOM_MIN;
  ui_ctx.temperature_graph_offset = 0;
  sys_ui_temp_screen_update_tab_colors();
  sys_ui_temp_screen_refresh();
}

static void sys_ui_temp_screen_cb_tab_hum(lv_event_t *event)
{
  (void) event;
  ui_ctx.env_tab                  = 1;
  ui_ctx.temperature_zoom         = SYS_UI_ENV_ZOOM_MIN;
  ui_ctx.temperature_graph_offset = 0;
  sys_ui_temp_screen_update_tab_colors();
  sys_ui_temp_screen_refresh();
}

static void sys_ui_temp_screen_cb_tab_dust(lv_event_t *event)
{
  (void) event;
  ui_ctx.env_tab                  = 2;
  ui_ctx.temperature_zoom         = SYS_UI_ENV_ZOOM_MIN;
  ui_ctx.temperature_graph_offset = 0;
  sys_ui_temp_screen_update_tab_colors();
  sys_ui_temp_screen_refresh();
}

static void sys_ui_temp_screen_cb_zoom_minus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom > SYS_UI_ENV_ZOOM_MIN)
  {
    --ui_ctx.temperature_zoom;
    sys_ui_temp_screen_refresh();
    LOG_DBG("sys_ui_temp_screen_cb_zoom_minus: zoom=%d", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_temp_screen_cb_zoom_plus(lv_event_t *event)
{
  (void) event;
  if (ui_ctx.temperature_zoom < SYS_UI_ENV_ZOOM_MAX)
  {
    ++ui_ctx.temperature_zoom;
    sys_ui_temp_screen_refresh();
    LOG_DBG("sys_ui_temp_screen_cb_zoom_plus: zoom=%d", ui_ctx.temperature_zoom);
  }
}

static void sys_ui_temp_screen_cb_pan_left(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_graph_offset -= SYS_UI_PAN_STEP;
  sys_ui_temp_screen_refresh();
  LOG_DBG("sys_ui_temp_screen_cb_pan_left: pan=%d", ui_ctx.temperature_graph_offset);
}

static void sys_ui_temp_screen_cb_pan_right(lv_event_t *event)
{
  (void) event;
  ui_ctx.temperature_graph_offset += SYS_UI_PAN_STEP;
  sys_ui_temp_screen_refresh();
  LOG_DBG("sys_ui_temp_screen_cb_pan_right: pan=%d", ui_ctx.temperature_graph_offset);
}

static void sys_ui_log_speed_sample(float value, size_t timestamp)
{
  sys_ui_env_fifo_push(&speed_fifo, value, timestamp);
}

static void sys_ui_env_fifo_push(fifo_t *fifo, float value, size_t timestamp)
{
  if (fifo_is_full(fifo))
  {
    sys_ui_env_sample_t discard;
    fifo_pop(fifo, &discard);
  }
  sys_ui_env_sample_t s = { value, timestamp };
  fifo_push(fifo, &s);
}

static sys_ui_env_sample_t sys_ui_env_fifo_get(const fifo_t *fifo, int index)
{
  sys_ui_env_sample_t item = { 0.0f, 0U };
  size_t              phys = (fifo->head + (size_t) index) % fifo->capacity;
  memcpy(&item, fifo->buffer + phys * fifo->item_size, sizeof(item));
  return item;
}

static void sys_ui_log_temp_sample(float value, size_t timestamp)
{
  sys_ui_env_fifo_push(&env_temp_fifo, value, timestamp);
}

static void sys_ui_log_hum_sample(float value, size_t timestamp)
{
  sys_ui_env_fifo_push(&env_hum_fifo, value, timestamp);
}

static void sys_ui_log_dust_sample(float value, size_t timestamp)
{
  sys_ui_env_fifo_push(&env_dust_fifo, value, timestamp);
}

static void sys_ui_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  sys_ui_context_t *ctx = (sys_ui_context_t *) lv_indev_get_user_data(indev);
  if (ctx == nullptr)
  {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  bsp_touch_point_t point = { 0 };
  bsp_touch_read(&point);

  if (point.touched)
  {
    data->point.x = point.x;
    data->point.y = point.y;
    data->state   = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }

  if (g_device_info.nvs_info.curr_state == DEVICE_STATE_IDLE)
  {
    OS_SEM_GIVE_FROM_ISR(sys_input_wakeup_sem);
  }
}

static void sys_ui_lock_screen_create(void)
{
  char buf[128];
  ui_ctx.widgets.lock_screen = sys_ui_widget_create_screen(SYS_UI_LOCK_BG_COLOR);

  snprintf(buf, sizeof(buf), "%s (%03u)", SYS_UI_QR_LABEL, g_device_info.nvs_info.device_id);
  sys_ui_widget_create_label(ui_ctx.widgets.lock_screen, SYS_UI_QR_LABEL_X, SYS_UI_QR_LABEL_Y, buf, SYS_UI_COLOR_TEXT,
                             SYS_UI_QR_LABEL_FONT);
  if (sys_ui_image_draw(ui_ctx.widgets.lock_screen, SYS_UI_QR_PATH, SYS_UI_QR_WIDTH, SYS_UI_QR_HEIGHT, SYS_UI_QR_X,
                        SYS_UI_QR_Y, &ui_ctx.widgets.lock_qr_dsc, &ui_ctx.widgets.lock_qr_img)
      != STATUS_OK)
  {
    LOG_ERR("sys_ui_lock_screen_create: QR image load failed");
    ui_ctx.widgets.lock_qr_img =
      sys_ui_widget_create_label(ui_ctx.widgets.lock_screen, SYS_UI_QR_LABEL_FAIL_X, SYS_UI_QR_LABEL_FAIL_Y,
                                 SYS_UI_QR_LABEL_FAIL, SYS_UI_COLOR_TEXT_DIM, SYS_UI_QR_LABEL_FAIL_FONT);
  }

#if SCREEN_SKIP_LOCK_SCREEN
  if (ui_ctx.widgets.lock_qr_img != nullptr)
  {
    lv_obj_add_flag(ui_ctx.widgets.lock_qr_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_ctx.widgets.lock_qr_img, sys_ui_lock_screen_cb_debug, LV_EVENT_CLICKED, nullptr);
  }
#endif
}

static void sys_ui_lock_screen_draw(void)
{
  if (ui_ctx.widgets.lock_screen == nullptr)
  {
    sys_ui_lock_screen_create();
  }

  sys_ui_show_screen(ui_ctx.widgets.lock_screen);
}

static void sys_ui_process_active(void)
{
  size_t now = OS_GET_TICK();

  switch (ui_ctx.view)
  {
  case SYS_UI_VIEW_MAIN:
  {
    if (now - ui_ctx.last_second_tick >= SYS_UI_COUNTDOWN_MS)
    {
      ui_ctx.last_second_tick = now;
      sys_ui_main_screen_update_countup();
      sys_ui_main_screen_update_time(ui_ctx.active_hours, ui_ctx.active_minutes, ui_ctx.active_seconds);
    }

    if (g_sys_ui_data_status.is_fusion_data_ready_for_ui)
    {
      sys_input_get_fusion_data(&ui_ctx.fusion);
      sys_ui_main_screen_update_speed_n_distance();
      sys_ui_main_screen_update_compass();
      ui_ctx.frame_counter++;

      if ((ui_ctx.frame_counter % SYS_UI_SPEED_SAMPLE_INTERVAL) == 0U)
      {
        sys_ui_log_speed_sample(ui_ctx.current_speed, OS_GET_TICK());
      }

      g_sys_ui_data_status.is_fusion_data_ready_for_ui = false;
    }

    if (g_sys_ui_data_status.is_dust_data_ready_for_ui || g_sys_ui_data_status.is_temp_hum_data_ready_for_ui)
    {
      sys_input_data_t env = { 0 };
      if (sys_input_get_env_data(&env) == STATUS_OK)
      {
        ui_ctx.temperature_C = env.temp_hum.temperature;
        ui_ctx.humidity      = env.temp_hum.humidity;
        ui_ctx.dust_value    = env.dust_value;
      }
      sys_ui_main_screen_update_env(ui_ctx.temperature_C, ui_ctx.humidity, ui_ctx.dust_value);
      g_sys_ui_data_status.is_dust_data_ready_for_ui     = false;
      g_sys_ui_data_status.is_temp_hum_data_ready_for_ui = false;
    }

    sys_ui_apply_notification_label();
    break;
  }
  case SYS_UI_VIEW_TIME:
  {
    break;
  }
  case SYS_UI_VIEW_DISTANCE:
  {
    sys_ui_fusion_screen_update();
    break;
  }
  case SYS_UI_VIEW_TEMPERATURE:
  {
    sys_ui_temp_screen_update();
    break;
  }
  case SYS_UI_VIEW_LOCK:
  case SYS_UI_VIEW_SETTINGS:
  case SYS_UI_VIEW_OUT:
  {
    // Static screens — no periodic update needed
    break;
  }
  default: break;
  }

  if ((ui_ctx.view == SYS_UI_VIEW_SETTINGS) && ui_ctx.pending_settings_redraw)
  {
    if (ui_ctx.widgets.settings_screen != nullptr)
    {
      lv_obj_del(ui_ctx.widgets.settings_screen);
      ui_ctx.widgets.settings_screen = nullptr;
    }
    ui_ctx.pending_settings_redraw = false;
    sys_ui_change_screen(SYS_UI_VIEW_SETTINGS);
  }
}

static void sys_ui_process_idle(void)
{
  bsp_display_set_brightness_percent(SYS_UI_BRIGHTNESS_PERCENT_OFF);
  OS_SEM_TAKE(sys_ui_wakeup_sem, OS_MAX_DELAY);
  bsp_display_set_brightness_percent(ui_ctx.brightness_percent);
}
static void sys_ui_process_locked(void)
{
  // Do nothing, just show the lock screen
}

#if SCREEN_SKIP_LOCK_SCREEN
static void sys_ui_lock_screen_cb_debug(lv_event_t *event)
{
  (void) event;
  sys_manager_write_event(SYS_MANAGER_EVT_UNLOCKED);
}
#endif

/* End of file -------------------------------------------------------- */