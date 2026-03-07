/**
 * @file       sys_ui_widget.h
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      Basic UI widget creation API using LVGL.
 *             Provides standard widget builders for cards, buttons, labels, etc.
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_UI_WIDGET_H_
#define _SYS_UI_WIDGET_H_
/* Includes ----------------------------------------------------------- */
#include "bsp_display.h"

#include <lvgl.h>

/* Public defines ----------------------------------------------------- */
// Widget colors
#define SYS_UI_WIDGET_COLOR_BG       BSP_DISPLAY_RGB_TO_HEX(0, 0, 0)        // Black
#define SYS_UI_WIDGET_COLOR_BG_CARD  BSP_DISPLAY_RGB_TO_HEX(0, 255, 0)      // Green
#define SYS_UI_WIDGET_COLOR_PRIMARY  BSP_DISPLAY_RGB_TO_HEX(0, 255, 255)    // Cyan
#define SYS_UI_WIDGET_COLOR_ACCENT   BSP_DISPLAY_RGB_TO_HEX(255, 255, 0)    // Yellow
#define SYS_UI_WIDGET_COLOR_SUCCESS  BSP_DISPLAY_RGB_TO_HEX(0, 255, 0)      // Green
#define SYS_UI_WIDGET_COLOR_WARNING  BSP_DISPLAY_RGB_TO_HEX(255, 165, 0)    // Orange
#define SYS_UI_WIDGET_COLOR_DANGER   BSP_DISPLAY_RGB_TO_HEX(255, 0, 0)      // Red
#define SYS_UI_WIDGET_COLOR_TEXT     BSP_DISPLAY_RGB_TO_HEX(255, 255, 255)  // White
#define SYS_UI_WIDGET_COLOR_TEXT_DIM BSP_DISPLAY_RGB_TO_HEX(128, 128, 128)  // Gray

/* Default widget styles */
#define SYS_UI_WIDGET_CARD_RADIUS    (4)   // Card corner radius
#define SYS_UI_WIDGET_CARD_BORDER    (1)   // Card border width
#define SYS_UI_WIDGET_CARD_PAD       (4)   // Card padding
#define SYS_UI_WIDGET_BTN_RADIUS     (6)   // Button corner radius
#define SYS_UI_WIDGET_ARC_WIDTH      (10)  // Arc thickness

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Widget style configuration
 */
typedef struct
{
  uint32_t bg_color;
  uint32_t border_color;
  uint32_t text_color;
  int      border_width;
  int      radius;
  int      padding;
} sys_ui_widget_info_t;

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Create a new screen with background color
 * @param bg_color Background color (hex)
 * @return Screen object pointer
 */
lv_obj_t *sys_ui_widget_create_screen(uint32_t bg_color);

/**
 * @brief Create a card widget (container with border)
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] border_color Border color (hex)
 *
 * @return Card object pointer
 */
lv_obj_t *sys_ui_widget_create_card(lv_obj_t *parent, int x, int y, int w, int h, uint32_t border_color);

/**
 * @brief Create a card with custom style
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] style Style configuration
 *
 * @return Card object pointer
 */
lv_obj_t *
sys_ui_widget_createCardStyled(lv_obj_t *parent, int x, int y, int w, int h, const sys_ui_widget_info_t *style);

/**
 * @brief Create a button with label
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] label Button text
 * @param[in] bg_color Background color (hex)
 * @param[in] text_color Text color (hex)
 *
 * @return Button object pointer
 */
lv_obj_t *sys_ui_widget_createButton(lv_obj_t   *parent,
                                     int         x,
                                     int         y,
                                     int         w,
                                     int         h,
                                     const char *label,
                                     uint32_t    bg_color,
                                     uint32_t    text_color);

/**
 * @brief Create a label
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] text Label text
 * @param[in] color Text color (hex)
 * @param[in] font Font pointer (NULL, auto set default)
 *
 * @return Label object pointer
 */
lv_obj_t *
sys_ui_widget_createLabel(lv_obj_t *parent, int x, int y, const char *text, uint32_t color, const lv_font_t *font);

/**
 * @brief Create a panel (container without border)
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] bg_color Background color (hex)
 *
 * @return Panel object pointer
 */
lv_obj_t *sys_ui_widget_createPanel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t bg_color);

/**
 * @brief Create an arc (for speedometer, gauges)
 *
 * @param[in] parent Parent object
 * @param[in] cx Center X position
 * @param[in] cy Center Y position
 * @param[in] radius Arc radius
 * @param[in] start_angle Start angle in degrees
 * @param[in] end_angle End angle in degrees
 * @param[in] bg_color Background arc color
 * @param[in] indicator_color Indicator color
 *
 * @return Arc object pointer
 */
lv_obj_t *sys_ui_widget_createArc(lv_obj_t *parent,
                                  int       cx,
                                  int       cy,
                                  int       radius,
                                  int       start_angle,
                                  int       end_angle,
                                  uint32_t  bg_color,
                                  uint32_t  indicator_color);

/**
 * @brief Create a bar (for battery, progress)
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] min_val Minimum value
 * @param[in] max_val Maximum value
 * @param[in] bg_color Background color
 * @param[in] indicator_color Indicator color
 *
 * @return Bar object pointer
 */
lv_obj_t *sys_ui_widget_createBar(lv_obj_t *parent,
                                  int       x,
                                  int       y,
                                  int       w,
                                  int       h,
                                  int       min_val,
                                  int       max_val,
                                  uint32_t  bg_color,
                                  uint32_t  indicator_color);

/**
 * @brief Create a slider
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] min_val Minimum value
 * @param[in] max_val Maximum value
 * @param[in] initial_val Initial value
 * @param[in] bg_color Background color
 * @param[in] indicator_color Indicator color
 *
 * @return Slider object pointer
 */
lv_obj_t *sys_ui_widget_create_slider(lv_obj_t *parent,
                                      int       x,
                                      int       y,
                                      int       w,
                                      int       h,
                                      int       min_val,
                                      int       max_val,
                                      int       initial_val,
                                      uint32_t  bg_color,
                                      uint32_t  indicator_color);

/**
 * @brief Create a chart
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] w Width
 * @param[in] h Height
 * @param[in] point_count Number of data points
 * @param[in] bg_color Background color
 *
 * @return Chart object pointer
 */
lv_obj_t *sys_ui_widget_createChart(lv_obj_t *parent, int x, int y, int w, int h, int point_count, uint32_t bg_color);

/**
 * @brief Add series to chart
 *
 * @param[in] chart Chart object
 * @param[in] color Series color
 *
 * @return Chart series pointer
 */
lv_chart_series_t *sys_ui_widget_addChartSeries(lv_obj_t *chart, uint32_t color);

/**
 * @brief Create a status LED (small circular indicator)
 *
 * @param[in] parent Parent object
 * @param[in] x X position
 * @param[in] y Y position
 * @param[in] size LED size (diameter)
 * @param[in] color LED color
 *
 * @return LED object pointer
 */
lv_obj_t *sys_ui_widget_createLed(lv_obj_t *parent, int x, int y, int size, uint32_t color);

/**
 * @brief Set widget clickable and add click callback
 *
 * @param[in] obj Widget object
 * @param[in] callback Click callback function
 * @param[in] user_data User data for callback
 *
 * @return none
 */
void sys_ui_widget_set_clickable(lv_obj_t *obj, lv_event_cb_t callback, void *user_data);

/**
 * @brief Update label text
 *
 * @param[in] label Label object
 * @param[in] text New text
 *
 * @return none
 */
void sys_ui_widget_set_label_text(lv_obj_t *label, const char *text);

/**
 * @brief Update label text with format
 *
 * @param[in] label Label object
 * @param[in] fmt Format string
 * @param[in] ... Format arguments
 *
 * @return none
 */
void sys_ui_widget_set_label_text_format(lv_obj_t *label, const char *fmt, ...);

/**
 * @brief Update bar value
 *
 * @param[in] bar Bar object
 * @param[in] value New value
 *
 * @return none
 */
void sys_ui_widget_set_bar_value(lv_obj_t *bar, int value);

/**
 * @brief Update arc value
 *
 * @param[in] arc Arc object
 * @param[in] value New value
 *
 * @return none
 */
void sys_ui_widget_set_arc_value(lv_obj_t *arc, int value);

/**
 * @brief Update arc indicator color
 *
 * @param[in] arc Arc object
 * @param[in] color New color
 *
 * @return none
 */
void sys_ui_widget_set_arc_color(lv_obj_t *arc, uint32_t color);

/**
 * @brief Update label color
 *
 * @param[in] label Label object
 * @param[in] color New color
 *[in]
 * @return none
 */
void sys_ui_widget_set_label_color(lv_obj_t *label, uint32_t color);

#endif /*End file _SYS_UI_WIDGET_H_*/

/* End of file -------------------------------------------------------- */
