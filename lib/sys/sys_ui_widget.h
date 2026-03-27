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
#define SYS_UI_WIDGET_COLOR_BG       BSP_DISPLAY_RGB_TO_HEX(13,  27,  42)   // Dark navy
#define SYS_UI_WIDGET_COLOR_BG_CARD  BSP_DISPLAY_RGB_TO_HEX(22,  40,  60)   // Card surface
#define SYS_UI_WIDGET_COLOR_PRIMARY  BSP_DISPLAY_RGB_TO_HEX(0,   180, 216)  // Vivid cyan
#define SYS_UI_WIDGET_COLOR_ACCENT   BSP_DISPLAY_RGB_TO_HEX(255, 183, 3)    // Amber gold
#define SYS_UI_WIDGET_COLOR_SUCCESS  BSP_DISPLAY_RGB_TO_HEX(6,   214, 160)  // Mint green
#define SYS_UI_WIDGET_COLOR_WARNING  BSP_DISPLAY_RGB_TO_HEX(249, 160, 63)   // Warm orange
#define SYS_UI_WIDGET_COLOR_DANGER   BSP_DISPLAY_RGB_TO_HEX(239, 71,  111)  // Coral red
#define SYS_UI_WIDGET_COLOR_TEXT     BSP_DISPLAY_RGB_TO_HEX(255, 255, 255)  // White
#define SYS_UI_WIDGET_COLOR_TEXT_DIM BSP_DISPLAY_RGB_TO_HEX(141, 153, 174)  // Steel gray

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
  size_t bg_color;
  size_t border_color;
  size_t text_color;
  int    border_width;
  int    radius;
  int    padding;
} sys_ui_widget_info_t;

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Create a new screen with background color
 * @param bg_color Background color (hex)
 * @return Screen object pointer
 */
lv_obj_t *sys_ui_widget_create_screen(size_t bg_color);

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
lv_obj_t *sys_ui_widget_create_card(lv_obj_t *parent, int x, int y, int w, int h, size_t border_color);

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
sys_ui_widget_create_card_styled(lv_obj_t *parent, int x, int y, int w, int h, const sys_ui_widget_info_t *style);

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
lv_obj_t *sys_ui_widget_create_button(lv_obj_t   *parent,
                                      int         x,
                                      int         y,
                                      int         w,
                                      int         h,
                                      const char *label,
                                      size_t      bg_color,
                                      size_t      text_color);

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
sys_ui_widget_create_label(lv_obj_t *parent, int x, int y, const char *text, size_t color, const lv_font_t *font);

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
lv_obj_t *sys_ui_widget_create_panel(lv_obj_t *parent, int x, int y, int w, int h, size_t bg_color);

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
lv_obj_t *sys_ui_widget_create_arc(lv_obj_t *parent,
                                   int       cx,
                                   int       cy,
                                   int       radius,
                                   int       start_angle,
                                   int       end_angle,
                                   size_t    bg_color,
                                   size_t    indicator_color);

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
lv_obj_t *sys_ui_widget_create_bar(lv_obj_t *parent,
                                   int       x,
                                   int       y,
                                   int       w,
                                   int       h,
                                   int       min_val,
                                   int       max_val,
                                   size_t    bg_color,
                                   size_t    indicator_color);

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
                                      size_t    bg_color,
                                      size_t    indicator_color);

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
lv_obj_t *sys_ui_widget_create_chart(lv_obj_t *parent, int x, int y, int w, int h, int point_count, size_t bg_color);

/**
 * @brief Add series to chart
 *
 * @param[in] chart Chart object
 * @param[in] color Series color
 *
 * @return Chart series pointer
 */
lv_chart_series_t *sys_ui_widget_addChart_series(lv_obj_t *chart, size_t color);

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
lv_obj_t *sys_ui_widget_create_led(lv_obj_t *parent, int x, int y, int size, size_t color);

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
void sys_ui_widget_set_arc_color(lv_obj_t *arc, size_t color);

/**
 * @brief Update label color
 *
 * @param[in] label Label object
 * @param[in] color New color
 *[in]
 * @return none
 */
void sys_ui_widget_set_label_color(lv_obj_t *label, size_t color);

#endif /*End file _SYS_UI_WIDGET_H_*/

/* End of file -------------------------------------------------------- */
