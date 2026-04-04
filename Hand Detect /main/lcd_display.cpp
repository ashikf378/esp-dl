#include "lcd_display.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lcd_display";

// LCD Pinout
#define BSP_LCD_SPI_MOSI   GPIO_NUM_47
#define BSP_LCD_SPI_CLK    GPIO_NUM_21
#define BSP_LCD_CS         GPIO_NUM_42
#define BSP_LCD_DC         GPIO_NUM_41
#define BSP_LCD_RST        GPIO_NUM_NC
#define BSP_LCD_BACKLIGHT  GPIO_NUM_14

esp_lcd_panel_handle_t panel_handle = NULL;

// Backlight PWM – uses timer 1 / channel 1 to avoid conflict with camera (timer 0)
static void backlight_pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ledc_timer.freq_hz = 5000;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = BSP_LCD_BACKLIGHT;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel.duty = 0;          // active low: duty=0 -> fully on
    ledc_channel.hpoint = 0;

    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PWM backlight init failed, falling back to GPIO high");
        gpio_set_direction(BSP_LCD_BACKLIGHT, GPIO_MODE_OUTPUT);
        gpio_set_level(BSP_LCD_BACKLIGHT, 1);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ledc_channel.duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ESP_LOGI(TAG, "Backlight PWM enabled (timer 1, channel 1)");
    }
}

void lcd_init(void)
{
    // SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = BSP_LCD_SPI_CLK;
    buscfg.mosi_io_num = BSP_LCD_SPI_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * 2;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // LCD IO
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = BSP_LCD_DC;
    io_config.cs_gpio_num = BSP_LCD_CS;
    io_config.pclk_hz = 20 * 1000 * 1000;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle));

    // Hardware reset if needed
    if (BSP_LCD_RST != GPIO_NUM_NC) {
        gpio_set_direction(BSP_LCD_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(BSP_LCD_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(BSP_LCD_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // Panel config
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = BSP_LCD_RST;
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
    panel_config.bits_per_pixel = 16;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // Software reset
    uint8_t cmd_swreset = 0x01;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_swreset, NULL, 0));
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t cmd_sleep_out = 0x11;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_sleep_out, NULL, 0));
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t cmd_col_mode = 0x3A;
    uint8_t pixel_format = 0x55;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_col_mode, &pixel_format, 1));

    uint8_t cmd_normal_disp = 0x13;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_normal_disp, NULL, 0));

    uint8_t cmd_display_on = 0x29;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_display_on, NULL, 0));

    // Orientation
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    backlight_pwm_init();
    ESP_LOGI(TAG, "LCD initialized");
}

void draw_box(uint16_t *buffer, int w, int h, int x1, int y1, int x2, int y2)
{
    uint16_t color = 0xF800; // RED
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= w) x2 = w - 1;
    if (y2 >= h) y2 = h - 1;
    for (int x = x1; x <= x2; x++) {
        buffer[y1 * w + x] = color;
        buffer[y2 * w + x] = color;
    }
    for (int y = y1; y <= y2; y++) {
        buffer[y * w + x1] = color;
        buffer[y * w + x2] = color;
    }
}

void test_pattern_color_cycle(void)
{
    uint16_t *test_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!test_buffer) {
        test_buffer = (uint16_t *)malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    }
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        return;
    }
    const uint16_t colors[] = {0x0000, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const char *color_names[] = {"black","red","green","blue","yellow","magenta","cyan","white"};
    for (int c = 0; c < 8; c++) {
        for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) test_buffer[i] = colors[c];
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer);
        ESP_LOGI(TAG, "Test pattern: %s", color_names[c]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(test_buffer);
}