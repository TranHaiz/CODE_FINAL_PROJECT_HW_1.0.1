/**
 * @file       sys_ui_widget.cpp
 * @copyright  Copyright (C) 2025 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2026-03-07
 * @author     Hai Tran
 *
 * @brief      Basic UI widget creation API implementation.
 *
 */

/* Includes ----------------------------------------------------------- */
#include "sys_ui_widget.h"

#include <stdarg.h>
#include <stdio.h>

/* Function definitions ----------------------------------------------- */
lv_obj_t *sys_ui_widget_create_screen(size_t bg_color)
{
  lv_obj_t *screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(screen, lv_color_hex(bg_color), 0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  return screen;
}

lv_obj_t *sys_ui_widget_create_card(lv_obj_t *parent, int x, int y, int w, int h, size_t border_color)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_size(card, w, h);
  lv_obj_set_style_bg_color(card, lv_color_hex(SYS_UI_WIDGET_COLOR_BG_CARD), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(border_color), 0);
  lv_obj_set_style_border_width(card, SYS_UI_WIDGET_CARD_BORDER, 0);
  lv_obj_set_style_radius(card, SYS_UI_WIDGET_CARD_RADIUS, 0);
  lv_obj_set_style_pad_all(card, SYS_UI_WIDGET_CARD_PAD, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  return card;
}

lv_obj_t *
sys_ui_widget_createCardStyled(lv_obj_t *parent, int x, int y, int w, int h, const sys_ui_widget_info_t *style)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_size(card, w, h);
  lv_obj_set_style_bg_color(card, lv_color_hex(style->bg_color), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(style->border_color), 0);
  lv_obj_set_style_border_width(card, style->border_width, 0);
  lv_obj_set_style_radius(card, style->radius, 0);
  lv_obj_set_style_pad_all(card, style->padding, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  return card;
}

lv_obj_t *sys_ui_widget_createButton(lv_obj_t   *parent,
                                     int         x,
                                     int         y,
                                     int         w,
                                     int         h,
                                     const char *label,
                                     size_t      bg_color,
                                     size_t      text_color)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_color(btn, lv_color_hex(bg_color), 0);
  lv_obj_set_style_radius(btn, SYS_UI_WIDGET_BTN_RADIUS, 0);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, label);
  lv_obj_set_style_text_color(lbl, lv_color_hex(text_color), 0);
  lv_obj_center(lbl);

  return btn;
}

lv_obj_t *
sys_ui_widget_createLabel(lv_obj_t *parent, int x, int y, const char *text, size_t color, const lv_font_t *font)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_pos(label, x, y);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  if (font != nullptr)
  {
    lv_obj_set_style_text_font(label, font, 0);
  }
  return label;
}

lv_obj_t *sys_ui_widget_createPanel(lv_obj_t *parent, int x, int y, int w, int h, size_t bg_color)
{
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_set_pos(panel, x, y);
  lv_obj_set_size(panel, w, h);
  lv_obj_set_style_bg_color(panel, lv_color_hex(bg_color), 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_radius(panel, 0, 0);
  lv_obj_set_style_pad_all(panel, 0, 0);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  return panel;
}

lv_obj_t *sys_ui_widget_createArc(lv_obj_t *parent,
                                  int       cx,
                                  int       cy,
                                  int       radius,
                                  int       start_angle,
                                  int       end_angle,
                                  size_t    bg_color,
                                  size_t    indicator_color)
{
  lv_obj_t *arc = lv_arc_create(parent);
  lv_obj_set_pos(arc, cx - radius, cy - radius);
  lv_obj_set_size(arc, radius * 2, radius * 2);
  lv_arc_set_rotation(arc, start_angle);
  lv_arc_set_bg_angles(arc, 0, end_angle - start_angle);
  lv_arc_set_range(arc, 0, 100);
  lv_arc_set_value(arc, 0);
  lv_obj_set_style_arc_width(arc, SYS_UI_WIDGET_ARC_WIDTH, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, SYS_UI_WIDGET_ARC_WIDTH, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(arc, lv_color_hex(bg_color), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(indicator_color), LV_PART_INDICATOR);
  lv_obj_remove_style(arc, nullptr, LV_PART_KNOB);
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  return arc;
}

lv_obj_t *sys_ui_widget_createBar(lv_obj_t *parent,
                                  int       x,
                                  int       y,
                                  int       w,
                                  int       h,
                                  int       min_val,
                                  int       max_val,
                                  size_t    bg_color,
                                  size_t    indicator_color)
{
  lv_obj_t *bar = lv_bar_create(parent);
  lv_obj_set_pos(bar, x, y);
  lv_obj_set_size(bar, w, h);
  lv_bar_set_range(bar, min_val, max_val);
  lv_bar_set_value(bar, min_val, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar, lv_color_hex(bg_color), 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(indicator_color), LV_PART_INDICATOR);
  return bar;
}

lv_obj_t *sys_ui_widget_create_slider(lv_obj_t *parent,
                                      int       x,
                                      int       y,
                                      int       w,
                                      int       h,
                                      int       min_val,
                                      int       max_val,
                                      int       initial_val,
                                      size_t    bg_color,
                                      size_t    indicator_color)
{
  lv_obj_t *slider = lv_slider_create(parent);
  lv_obj_set_pos(slider, x, y);
  lv_obj_set_size(slider, w, h);
  lv_slider_set_range(slider, min_val, max_val);
  lv_slider_set_value(slider, initial_val, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider, lv_color_hex(bg_color), LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider, lv_color_hex(indicator_color), LV_PART_INDICATOR);
  return slider;
}

lv_obj_t *sys_ui_widget_createChart(lv_obj_t *parent, int x, int y, int w, int h, int point_count, size_t bg_color)
{
  lv_obj_t *chart = lv_chart_create(parent);
  lv_obj_set_pos(chart, x, y);
  lv_obj_set_size(chart, w, h);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, point_count);
  lv_obj_set_style_bg_color(chart, lv_color_hex(bg_color), 0);
  return chart;
}

lv_chart_series_t *sys_ui_widget_addChartSeries(lv_obj_t *chart, size_t color)
{
  return lv_chart_add_series(chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
}

lv_obj_t *sys_ui_widget_createLed(lv_obj_t *parent, int x, int y, int size, size_t color)
{
  lv_obj_t *led = lv_obj_create(parent);
  lv_obj_set_pos(led, x, y);
  lv_obj_set_size(led, size, size);
  lv_obj_set_style_bg_color(led, lv_color_hex(color), 0);
  lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(led, 0, 0);
  return led;
}

void sys_ui_widget_set_clickable(lv_obj_t *obj, lv_event_cb_t callback, void *user_data)
{
  if (obj == nullptr)
    return;
  lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(obj, callback, LV_EVENT_CLICKED, user_data);
}

void sys_ui_widget_set_label_text(lv_obj_t *label, const char *text)
{
  if (label == nullptr)
    return;
  lv_label_set_text(label, text);
}

void sys_ui_widget_set_label_text_format(lv_obj_t *label, const char *fmt, ...)
{
  if (label == nullptr || fmt == nullptr)
    return;

  char    buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  lv_label_set_text(label, buf);
}

void sys_ui_widget_set_bar_value(lv_obj_t *bar, int value)
{
  if (bar == nullptr)
    return;
  lv_bar_set_value(bar, value, LV_ANIM_OFF);
}

void sys_ui_widget_set_arc_value(lv_obj_t *arc, int value)
{
  if (arc == nullptr)
    return;
  lv_arc_set_value(arc, value);
}

void sys_ui_widget_set_arc_color(lv_obj_t *arc, size_t color)
{
  if (arc == nullptr)
    return;
  lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
}

void sys_ui_widget_set_label_color(lv_obj_t *label, size_t color)
{
  if (label == nullptr)
    return;
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
}

/* End of file -------------------------------------------------------- */
