#include "dl_image.hpp"
#include "hand_detect.hpp"


#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"


#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"


static const char *TAG = "hand_detect_lcd";


// =================== LCD PINOUT ===================
#define BSP_LCD_SPI_MOSI GPIO_NUM_47
#define BSP_LCD_SPI_CLK GPIO_NUM_21
#define BSP_LCD_CS GPIO_NUM_42
#define BSP_LCD_DC GPIO_NUM_41
#define BSP_LCD_RST GPIO_NUM_NC // No hardware reset pin connected
#define BSP_LCD_BACKLIGHT GPIO_NUM_14


#define LCD_H_RES 320
#define LCD_V_RES 240


esp_lcd_panel_handle_t panel_handle = NULL;


// =================== BACKLIGHT PWM INIT ===================
static void backlight_pwm_init()
{
   bool active_low = true; // Change to false if needed


   ledc_timer_config_t ledc_timer = {};
   ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
   ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
   ledc_timer.timer_num = LEDC_TIMER_0;
   ledc_timer.freq_hz = 5000;
   ledc_timer.clk_cfg = LEDC_AUTO_CLK;
   ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));


   ledc_channel_config_t ledc_channel = {};
   ledc_channel.gpio_num = BSP_LCD_BACKLIGHT;
   ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
   ledc_channel.channel = LEDC_CHANNEL_0;
   ledc_channel.intr_type = LEDC_INTR_DISABLE;
   ledc_channel.timer_sel = LEDC_TIMER_0;
   ledc_channel.duty = active_low ? 0 : 255;
   ledc_channel.hpoint = 0;


   esp_err_t ret = ledc_channel_config(&ledc_channel);
   if (ret != ESP_OK) {
       ESP_LOGW(TAG, "PWM backlight init failed, falling back to GPIO high");
       gpio_set_direction(BSP_LCD_BACKLIGHT, GPIO_MODE_OUTPUT);
       gpio_set_level(BSP_LCD_BACKLIGHT, 1);
   } else {
       ESP_LOGI(TAG, "Backlight PWM enabled (duty=%u)", (unsigned int)ledc_channel.duty);
   }
}


// =================== LCD INIT ===================
void lcd_init()
{
   // 1. SPI bus
   spi_bus_config_t buscfg = {};
   buscfg.sclk_io_num = BSP_LCD_SPI_CLK;
   buscfg.mosi_io_num = BSP_LCD_SPI_MOSI;
   buscfg.miso_io_num = -1;
   buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * 2;
   ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));


   // 2. SPI interface config
   esp_lcd_panel_io_spi_config_t io_config = {};
   io_config.dc_gpio_num = BSP_LCD_DC;
   io_config.cs_gpio_num = BSP_LCD_CS;
   io_config.pclk_hz = 20 * 1000 * 1000; // 20 MHz
   io_config.lcd_cmd_bits = 8;
   io_config.lcd_param_bits = 8;
   io_config.spi_mode = 0;
   io_config.trans_queue_depth = 10;


   esp_lcd_panel_io_handle_t io_handle = nullptr;
   ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle));


   // 3. Hardware reset (if connected)
   if (BSP_LCD_RST != GPIO_NUM_NC) {
       gpio_set_direction(BSP_LCD_RST, GPIO_MODE_OUTPUT);
       gpio_set_level(BSP_LCD_RST, 0);
       vTaskDelay(pdMS_TO_TICKS(10));
       gpio_set_level(BSP_LCD_RST, 1);
       vTaskDelay(pdMS_TO_TICKS(120));
   }


   // 4. Panel driver
   esp_lcd_panel_dev_config_t panel_config = {};
   panel_config.reset_gpio_num = BSP_LCD_RST;
   panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
   panel_config.bits_per_pixel = 16;
   panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB; // Change to BGR if colors are wrong


   ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
   ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));


   // ========== ST7789 BASIC INIT ==========
   // Sleep out
   uint8_t cmd_sleep_out = 0x11;
   ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_sleep_out, NULL, 0));
   vTaskDelay(pdMS_TO_TICKS(120));


   // Set color mode to 16-bit (0x55)
   uint8_t cmd_col_mode = 0x3A;
   uint8_t pixel_format = 0x55;
   ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_col_mode, &pixel_format, 1));


   // Normal display mode (0x13) – optional but often needed
   uint8_t cmd_normal_disp = 0x13;
   ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_normal_disp, NULL, 0));


   // Display on
   uint8_t cmd_display_on = 0x29;
   ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd_display_on, NULL, 0));
   // =====================================


   // Orientation settings
   ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
   ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));


   // Optionally invert colors (uncomment if needed)
   // ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));


   // Turn display on
   ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


   // Backlight
   backlight_pwm_init();


   ESP_LOGI(TAG, "LCD initialized (simplified)");
}


// =================== DRAW BOX ===================
void draw_box(uint16_t *buffer, int w, int h, int x1, int y1, int x2, int y2)
{
   uint16_t color = 0xF800; // RED
   if (x1 < 0)
       x1 = 0;
   if (y1 < 0)
       y1 = 0;
   if (x2 >= w)
       x2 = w - 1;
   if (y2 >= h)
       y2 = h - 1;
   for (int x = x1; x <= x2; x++) {
       buffer[y1 * w + x] = color;
       buffer[y2 * w + x] = color;
   }
   for (int y = y1; y <= y2; y++) {
       buffer[y * w + x1] = color;
       buffer[y * w + x2] = color;
   }
}


// =================== TEST PATTERNS ===================
void test_pattern_color_cycle()
{
   uint16_t *test_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
   if (!test_buffer)
       return;


   const uint16_t colors[] = {0x0000, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
   const char *color_names[] = {"black", "red", "green", "blue", "yellow", "magenta", "cyan", "white"};


   for (int c = 0; c < 8; c++) {
       for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) test_buffer[i] = colors[c];
       ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer));
       ESP_LOGI(TAG, "Test pattern: %s", color_names[c]);
       vTaskDelay(pdMS_TO_TICKS(1000));
   }
   free(test_buffer);
}


// =================== HAND MOTION DETECTION ===================
// Simple structure to store previous hand center
struct HandMotion {
   int prev_cx = -1;
   int prev_cy = -1;
   bool valid = false;
   int motion_threshold = 20; // pixels movement to register motion
};


// Function to classify motion direction based on delta
std::string classify_motion(int dx, int dy)
{
   if (abs(dx) < 10 && abs(dy) < 10)
       return "STATIONARY";
   std::string result;
   if (abs(dx) > abs(dy)) {
       result = (dx > 0) ? "RIGHT" : "LEFT";
   } else {
       result = (dy > 0) ? "DOWN" : "UP";
   }
   return result;
}


extern "C" void app_main(void)
{
   ESP_LOGI(TAG, "Starting...");
   lcd_init();


   // Quick red screen test
   uint16_t *red = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
   if (red) {
       for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) red[i] = 0xF800;
       esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, red);
       free(red);
       vTaskDelay(pdMS_TO_TICKS(2000));
   }


   test_pattern_color_cycle();


   // Camera init
   ESP_LOGI(TAG, "Initializing camera...");
   camera_config_t camera_config = {};
   camera_config.pin_pwdn = -1;
   camera_config.pin_reset = -1;
   camera_config.pin_xclk = 15;
   camera_config.pin_sccb_sda = 4;
   camera_config.pin_sccb_scl = 5;
   camera_config.pin_d7 = 16;
   camera_config.pin_d6 = 17;
   camera_config.pin_d5 = 18;
   camera_config.pin_d4 = 12;
   camera_config.pin_d3 = 10;
   camera_config.pin_d2 = 8;
   camera_config.pin_d1 = 9;
   camera_config.pin_d0 = 11;
   camera_config.pin_vsync = 6;
   camera_config.pin_href = 7;
   camera_config.pin_pclk = 13;
   camera_config.xclk_freq_hz = 20000000;
   camera_config.ledc_timer = LEDC_TIMER_0;
   camera_config.ledc_channel = LEDC_CHANNEL_0;
   camera_config.pixel_format = PIXFORMAT_RGB565;
   camera_config.frame_size = FRAMESIZE_QVGA;
   camera_config.fb_count = 1;
   camera_config.fb_location = CAMERA_FB_IN_PSRAM;


   if (esp_camera_init(&camera_config) != ESP_OK) {
       ESP_LOGE(TAG, "Camera init failed");
       return;
   }


   // --- Enable vertical flip ---
   sensor_t *s = esp_camera_sensor_get();
   if (s) {
       s->set_vflip(s, 1); // 1 = enable vertical flip
       ESP_LOGI(TAG, "Vertical flip enabled");
   } else {
       ESP_LOGW(TAG, "Could not get sensor handle");
   }
   // ---------------------------


   ESP_LOGI(TAG, "Camera initialized");


   HandDetect hand_detect;
   HandMotion motion_tracker;


   uint16_t *lcd_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
   if (!lcd_buffer) {
       ESP_LOGE(TAG, "Failed to allocate LCD buffer");
       return;
   }


   while (true) {
       camera_fb_t *fb = esp_camera_fb_get();
       if (!fb) {
           ESP_LOGE(TAG, "Capture failed");
           continue;
       }


       dl::image::img_t hand_img;


       if (fb->format != PIXFORMAT_RGB565) {
           dl::image::jpeg_img_t jpeg_img = {.data = fb->buf, .data_len = fb->len};
           dl::image::img_t decoded = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
           uint8_t *img_bytes = (uint8_t *)decoded.data;
           for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
               uint8_t r = img_bytes[i * 3];
               uint8_t g = img_bytes[i * 3 + 1];
               uint8_t b = img_bytes[i * 3 + 2];
               lcd_buffer[i] = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b >> 3);
           }
           hand_img.width = decoded.width;
           hand_img.height = decoded.height;
           hand_img.data = decoded.data;
           hand_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
       } else {
           memcpy(lcd_buffer, fb->buf, LCD_H_RES * LCD_V_RES * 2);
           hand_img.width = fb->width;
           hand_img.height = fb->height;
           hand_img.data = fb->buf;
           hand_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;
       }


       auto results = hand_detect.run(hand_img);


       // Variables to store the most confident hand center
       int best_cx = -1, best_cy = -1;
       float best_score = 0.0;


       if (results.empty()) {
           ESP_LOGI(TAG, "No hand detected");
           // Optionally reset motion tracker after some time? For now, we keep last known position.
       } else {
           for (const auto &res : results) {
               if (res.score < 0.15)
                   continue;


               int x1 = res.box[0], y1 = res.box[1], x2 = res.box[2], y2 = res.box[3];
               draw_box(lcd_buffer, LCD_H_RES, LCD_V_RES, x1, y1, x2, y2);


               // Center in image coordinates
               int cx_img = (x1 + x2) / 2;
               int cy_img = (y1 + y2) / 2;


               // Transform to screen coordinates (same as original)
               int cx_screen = (LCD_H_RES - 1) - cy_img; // screen X = width - 1 - image Y
               int cy_screen = cx_img;                   // screen Y = image X


               // Use screen coordinates for classification
               std::string horizontal = (cx_screen < LCD_H_RES / 3) ? "CENTER"
                   : (cx_screen > 2 * LCD_H_RES / 3)                ? "RIGHT"
                                                                    : "LEFT";
               std::string vertical = (cy_screen < LCD_V_RES / 3) ? ""
                   : (cy_screen > 2 * LCD_V_RES / 3)              ? ""
                                                                  : "";


               int box_area = (x2 - x1) * (y2 - y1);
               std::string distance = (box_area > (LCD_H_RES * LCD_V_RES) / 3) ? "TOO CLOSE"
                   : (box_area < (LCD_H_RES * LCD_V_RES) / 10)                 ? "TOO FAR"
                                                                               : "OK";


               ESP_LOGI(TAG,
                        "Hand: %.2f, Pos: (%s,%s), Size: %s",
                        res.score,
                        horizontal.c_str(),
                        vertical.c_str(),
                        distance.c_str());


               // Track best detection (highest score)
               if (res.score > best_score) {
                   best_score = res.score;
                   best_cx = cx_screen;
                   best_cy = cy_screen;
               }
           }
       }


       // --- Hand Motion Detection ---
       if (best_cx != -1 && best_cy != -1) {
           // Hand detected in this frame
           if (motion_tracker.valid) {
               int dx = best_cx - motion_tracker.prev_cx;
               int dy = best_cy - motion_tracker.prev_cy;
               if (abs(dx) > motion_tracker.motion_threshold || abs(dy) > motion_tracker.motion_threshold) {
                   std::string direction = classify_motion(dx, dy);
                   ESP_LOGI(TAG, "Hand motion: %s (dx=%d, dy=%d)", direction.c_str(), dx, dy);
               }
           }
           // Update tracker
           motion_tracker.prev_cx = best_cx;
           motion_tracker.prev_cy = best_cy;
           motion_tracker.valid = true;
       } else {
           // No hand detected this frame; optionally invalidate after some frames?
           // For simplicity, we keep the last valid position and continue tracking.
           // If too many frames without detection, reset? Not needed for basic motion.
       }


       ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer));


       if (fb->format != PIXFORMAT_RGB565)
           heap_caps_free(hand_img.data);
       esp_camera_fb_return(fb);


       vTaskDelay(pdMS_TO_TICKS(30));
   }
}



