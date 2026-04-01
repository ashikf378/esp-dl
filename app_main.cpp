#include "dl_image.hpp"
#include "hand_detect.hpp"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
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
#define BSP_LCD_RST GPIO_NUM_NC
#define BSP_LCD_BACKLIGHT GPIO_NUM_14

#define LCD_H_RES 320
#define LCD_V_RES 240

// =================== DISTANCE THRESHOLDS (area) ===================
#define AREA_TOO_CLOSE (0.30f * LCD_H_RES * LCD_V_RES) // > 23040 px
#define AREA_TOO_FAR (0.05f * LCD_H_RES * LCD_V_RES)   // < 3840 px

esp_lcd_panel_handle_t panel_handle = NULL;

// =================== BACKLIGHT PWM INIT ===================
static void backlight_pwm_init()
{
    bool active_low = true;

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
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = BSP_LCD_SPI_CLK;
    buscfg.mosi_io_num = BSP_LCD_SPI_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * 2;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

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

    if (BSP_LCD_RST != GPIO_NUM_NC) {
        gpio_set_direction(BSP_LCD_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(BSP_LCD_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(BSP_LCD_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = BSP_LCD_RST;
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
    panel_config.bits_per_pixel = 16;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // ST7789 initialization commands
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
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        return;
    }

    const uint16_t colors[] = {0x0000, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const char *color_names[] = {"black", "red", "green", "blue", "yellow", "magenta", "cyan", "white"};

    for (int c = 0; c < 8; c++) {
        for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) test_buffer[i] = colors[c];
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "Draw test pattern failed: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "Test pattern: %s", color_names[c]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(test_buffer);
}

// =================== POSITION CLASSIFICATION ===================
const char *classify_horizontal(int x_center)
{
    if (x_center >= 30 && x_center <= 85)
        return "Left";
    if (x_center >= 200 && x_center <= 275)
        return "Right";
    if (x_center >= 160 && x_center <= 180)
        return "Center";
    return "Center";
}

const char *classify_vertical(int y_center)
{
    if (y_center >= 40 && y_center <= 80)
        return "Down";
    if (y_center >= 90 && y_center <= 115)
        return "Center";
    if (y_center >= 170 && y_center <= 200)
        return "Up";
    return "Center";
}

const char *classify_distance(int area)
{
    if (area > AREA_TOO_CLOSE)
        return "Too Close";
    if (area < AREA_TOO_FAR)
        return "Too Far";
    return "Normal";
}

// =================== TEMPORAL SMOOTHING STRUCTURE ===================
#define SMOOTHING_WINDOW 5     // number of recent detections to keep
#define MIN_VALID_DETECTIONS 3 // require at least 3 detections before output

struct SmoothedDetection {
    int cx = -1, cy = -1, area = -1;
    bool valid = false;
};

static SmoothedDetection detection_history[SMOOTHING_WINDOW];
static int history_index = 0;

void add_detection(int cx, int cy, int area)
{
    detection_history[history_index].cx = cx;
    detection_history[history_index].cy = cy;
    detection_history[history_index].area = area;
    detection_history[history_index].valid = true;
    history_index = (history_index + 1) % SMOOTHING_WINDOW;
}

void clear_history()
{
    for (int i = 0; i < SMOOTHING_WINDOW; i++) {
        detection_history[i].valid = false;
    }
    history_index = 0;
}

bool get_smoothed_detection(int &cx_out, int &cy_out, int &area_out)
{
    int sum_cx = 0, sum_cy = 0, sum_area = 0;
    int count = 0;
    for (int i = 0; i < SMOOTHING_WINDOW; i++) {
        if (detection_history[i].valid) {
            sum_cx += detection_history[i].cx;
            sum_cy += detection_history[i].cy;
            sum_area += detection_history[i].area;
            count++;
        }
    }
    if (count >= MIN_VALID_DETECTIONS) {
        cx_out = sum_cx / count;
        cy_out = sum_cy / count;
        area_out = sum_area / count;
        return true;
    }
    return false;
}

// =================== MAIN ===================
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
    } else {
        ESP_LOGE(TAG, "Failed to allocate red test buffer, skipping");
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

    // Vertical flip
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1);
        ESP_LOGI(TAG, "Vertical flip enabled");
    } else {
        ESP_LOGW(TAG, "Could not get sensor handle");
    }

    ESP_LOGI(TAG, "Camera initialized");

    // Increase confidence threshold for more stable detections
    HandDetect hand_detect(HandDetect::model_type_t::ESPDET_PICO_224_224_HAND);
    // Override thresholds (defaults are 0.25 score, 0.45 nms)
    hand_detect.set_score_thr(0.5f);
    hand_detect.set_nms_thr(0.45f); // unchanged but can adjust

    uint16_t *lcd_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!lcd_buffer) {
        ESP_LOGE(TAG, "Failed to allocate LCD buffer");
        return;
    }

    // Configure watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);

    while (true) {
        esp_task_wdt_reset();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Capture failed");
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        size_t expected_size = LCD_H_RES * LCD_V_RES * 2;
        if (fb->len != expected_size) {
            ESP_LOGW(TAG, "Frame size mismatch: got %zu, expected %zu", fb->len, expected_size);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        dl::image::img_t hand_img;
        if (fb->format != PIXFORMAT_RGB565) {
            ESP_LOGW(TAG, "Unsupported format, skipping");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        } else {
            memcpy(lcd_buffer, fb->buf, expected_size);
            hand_img.width = fb->width;
            hand_img.height = fb->height;
            hand_img.data = fb->buf;
            hand_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;
        }

        auto results = hand_detect.run(hand_img);

        int best_cx_img = -1, best_cy_img = -1;
        float best_score = 0.0;
        int best_x1 = 0, best_y1 = 0, best_x2 = 0, best_y2 = 0;

        if (results.empty()) {
            ESP_LOGI(TAG, "No hand detected");
            // Clear history on consecutive no-detections? We'll keep old history but not add new.
            // Optionally clear after a timeout, but here we just don't add new detection.
        } else {
            for (const auto &res : results) {
                // Use the already increased score threshold in model, but double-check
                if (res.score < 0.5) // same as set_score_thr, but safe
                    continue;

                int x1 = res.box[0], y1 = res.box[1], x2 = res.box[2], y2 = res.box[3];
                draw_box(lcd_buffer, LCD_H_RES, LCD_V_RES, x1, y1, x2, y2);

                int cx_img = (x1 + x2) / 2;
                int cy_img = (y1 + y2) / 2;

                if (res.score > best_score) {
                    best_score = res.score;
                    best_cx_img = cx_img;
                    best_cy_img = cy_img;
                    best_x1 = x1;
                    best_y1 = y1;
                    best_x2 = x2;
                    best_y2 = y2;
                }
            }

            if (best_cx_img != -1 && best_cy_img != -1) {
                int area = (best_x2 - best_x1) * (best_y2 - best_y1);
                add_detection(best_cx_img, best_cy_img, area);
            } else {
                // No valid detection (score too low) – optionally clear or ignore
                // We'll keep history but not add anything. This may cause stale data.
                // To prevent stale, we could clear if too many frames without detection.
                // For simplicity, we do nothing and rely on MIN_VALID_DETECTIONS.
            }
        }

        // Get smoothed detection
        int smoothed_cx, smoothed_cy, smoothed_area;
        if (get_smoothed_detection(smoothed_cx, smoothed_cy, smoothed_area)) {
            const char *horiz = classify_horizontal(smoothed_cx);
            const char *vert = classify_vertical(smoothed_cy);
            const char *dist = classify_distance(smoothed_area);
            ESP_LOGI(TAG,
                     "Hand Position (smoothed): %s-%s, Distance: %s (center: %d,%d, area: %d)",
                     horiz,
                     vert,
                     dist,
                     smoothed_cx,
                     smoothed_cy,
                     smoothed_area);
        } else {
            // Not enough detections – maybe show nothing or last known
            ESP_LOGI(TAG, "Insufficient valid detections for stable output");
        }

        // Draw to LCD
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LCD draw failed: %s", esp_err_to_name(ret));
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
