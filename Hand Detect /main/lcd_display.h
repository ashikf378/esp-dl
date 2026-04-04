#pragma once

#include "esp_lcd_panel_ops.h"
#include <stdint.h>

// LCD dimensions
#define LCD_H_RES 320
#define LCD_V_RES 240

// Public LCD functions
void lcd_init(void);
void draw_box(uint16_t *buffer, int w, int h, int x1, int y1, int x2, int y2);
void test_pattern_color_cycle(void);

// The panel handle is needed for drawing (extern so main.cpp can use it)
extern esp_lcd_panel_handle_t panel_handle;