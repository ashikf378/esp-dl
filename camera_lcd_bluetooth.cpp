// Combined BLE Scanner + Hand Detection with LCD
// For ESP32-S3, uses NimBLE, camera, ST7789 LCD

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

// Hand detection & LCD includes
#include "dl_image.hpp"
#include "esp_camera.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_task_wdt.h"
#include "hand_detect.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

// =================== BLE Definitions ===================
#define BLE_TAG "BLE-Connect"
#define TARGET_DEVICE_NAME "TD630 SZ-FE9974(BLE)"

static uint8_t ble_addr_type;
static bool scanning = false;
static bool connecting = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Forward declarations for BLE
static void ble_start_scan(void);
static void ble_connect_to_target(const ble_addr_t *addr);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_on_sync(void);
static void host_task(void *param);

// =================== LCD & Hand Detection Definitions ===================
static const char *TAG = "hand_detect_lcd";

// LCD Pinout
#define BSP_LCD_SPI_MOSI GPIO_NUM_47
#define BSP_LCD_SPI_CLK GPIO_NUM_21
#define BSP_LCD_CS GPIO_NUM_42
#define BSP_LCD_DC GPIO_NUM_41
#define BSP_LCD_RST GPIO_NUM_NC
#define BSP_LCD_BACKLIGHT GPIO_NUM_14

#define LCD_H_RES 320
#define LCD_V_RES 240

// Distance thresholds (area in pixels)
#define AREA_TOO_CLOSE (0.30f * LCD_H_RES * LCD_V_RES) // > 23040
#define AREA_TOO_FAR (0.05f * LCD_H_RES * LCD_V_RES)   // < 3840

static esp_lcd_panel_handle_t panel_handle = NULL;

// Temporal smoothing for hand detection
#define SMOOTHING_WINDOW 5
#define MIN_VALID_DETECTIONS 3

struct SmoothedDetection {
    int cx = -1, cy = -1, area = -1;
    bool valid = false;
};

static SmoothedDetection detection_history[SMOOTHING_WINDOW];
static int history_index = 0;

// Forward declarations for LCD & hand detection
static void backlight_pwm_init(void);
static void lcd_init(void);
static void draw_box(uint16_t *buffer, int w, int h, int x1, int y1, int x2, int y2);
static void test_pattern_color_cycle(void);
static const char *classify_horizontal(int x_center);
static const char *classify_vertical(int y_center);
static const char *classify_distance(int area);
static void add_detection(int cx, int cy, int area);
static bool get_smoothed_detection(int &cx_out, int &cy_out, int &area_out);

// ==================== BLE Implementation ====================
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) {
            ESP_LOGE(BLE_TAG, "Failed to parse advertisement data");
            return 0;
        }
        if (fields.name != NULL) {
            char device_name[32];
            int name_len = fields.name_len;
            if (name_len > sizeof(device_name) - 1)
                name_len = sizeof(device_name) - 1;
            memcpy(device_name, fields.name, name_len);
            device_name[name_len] = '\0';
            ESP_LOGI(BLE_TAG, "Found device: %s", device_name);
            if (scanning && !connecting && strcmp(device_name, TARGET_DEVICE_NAME) == 0) {
                ESP_LOGI(BLE_TAG, "Target device found! Stopping scan and connecting...");
                scanning = false;
                ble_gap_disc_cancel();
                ble_connect_to_target(&event->disc.addr);
            }
        }
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(BLE_TAG, "Scan completed");
        if (scanning) {
            ESP_LOGI(BLE_TAG, "Target not found, restarting scan...");
            ble_start_scan();
        }
        break;
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(BLE_TAG, "Connection event: status = %d", event->connect.status);
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            connecting = false;
            ESP_LOGI(BLE_TAG, "Connected to %s successfully!", TARGET_DEVICE_NAME);
        } else {
            connecting = false;
            ESP_LOGE(BLE_TAG, "Connection failed; restarting scan...");
            ble_start_scan();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(BLE_TAG, "Disconnected; reason = %d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_start_scan();
        break;
    default:
        break;
    }
    return 0;
}

static void ble_connect_to_target(const ble_addr_t *addr)
{
    struct ble_gap_conn_params conn_params = {0};
    conn_params.scan_itvl = 0x30;
    conn_params.scan_window = 0x30;
    conn_params.itvl_min = 0x30;
    conn_params.itvl_max = 0x30;
    conn_params.latency = 0;
    conn_params.supervision_timeout = 0x200;
    conn_params.min_ce_len = 0;
    conn_params.max_ce_len = 0;

    connecting = true;
    int rc = ble_gap_connect(ble_addr_type, addr, BLE_HS_FOREVER, &conn_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Error initiating connection: %d", rc);
        connecting = false;
        ble_start_scan();
    }
}

static void ble_start_scan(void)
{
    if (scanning || connecting) {
        ESP_LOGI(BLE_TAG, "Already scanning or connecting");
        return;
    }
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 0;
    disc_params.itvl = 0x30;
    disc_params.window = 0x30;
    disc_params.filter_duplicates = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;
    disc_params.disable_observer_mode = 0;

    int rc = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Failed to start scan: %d", rc);
    } else {
        scanning = true;
        ESP_LOGI(BLE_TAG, "Scanning for %s", TARGET_DEVICE_NAME);
    }
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_start_scan();
}

void host_task(void *param)
{
    nimble_port_run();
}

// ==================== LCD & Hand Detection Implementation ====================
static void backlight_pwm_init(void)
{
    // Use LEDC_TIMER_1 and LEDC_CHANNEL_1 to avoid conflict with camera (which uses timer 0)
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
    ledc_channel.duty = 0; // active low: 0 = fully on
    ledc_channel.hpoint = 0;

    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PWM backlight init failed, fallback to GPIO high");
        gpio_set_direction(BSP_LCD_BACKLIGHT, GPIO_MODE_OUTPUT);
        gpio_set_level(BSP_LCD_BACKLIGHT, 1);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ledc_channel.duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ESP_LOGI(TAG, "Backlight PWM enabled (timer 1, channel 1)");
    }
}

static void lcd_init(void)
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
    io_config.pclk_hz = 20 * 1000 * 1000;
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

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    backlight_pwm_init();
    ESP_LOGI(TAG, "LCD initialized");
}

static void draw_box(uint16_t *buffer, int w, int h, int x1, int y1, int x2, int y2)
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

static void test_pattern_color_cycle(void)
{
    // Try PSRAM first, then internal SRAM
    uint16_t *test_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!test_buffer) {
        test_buffer = (uint16_t *)malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    }
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        return;
    }
    const uint16_t colors[] = {0x0000, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const char *color_names[] = {"black", "red", "green", "blue", "yellow", "magenta", "cyan", "white"};
    for (int c = 0; c < 8; c++) {
        for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) test_buffer[i] = colors[c];
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer);
        ESP_LOGI(TAG, "Test pattern: %s", color_names[c]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(test_buffer);
}
static const char *classify_horizontal(int x_center)
{
    if (x_center >= 30 && x_center <= 85)
        return "Left";
    if (x_center >= 200 && x_center <= 275)
        return "Right";
    if (x_center >= 160 && x_center <= 180)
        return "Center";
    return "Center";
}

static const char *classify_vertical(int y_center)
{
    if (y_center >= 40 && y_center <= 80)
        return "Down";
    if (y_center >= 90 && y_center <= 115)
        return "Center";
    if (y_center >= 170 && y_center <= 200)
        return "Up";
    return "Center";
}

static const char *classify_distance(int area)
{
    if (area > AREA_TOO_CLOSE)
        return "Too Close";
    if (area < AREA_TOO_FAR)
        return "Too Far";
    return "Normal";
}

static void add_detection(int cx, int cy, int area)
{
    detection_history[history_index].cx = cx;
    detection_history[history_index].cy = cy;
    detection_history[history_index].area = area;
    detection_history[history_index].valid = true;
    history_index = (history_index + 1) % SMOOTHING_WINDOW;
}

static bool get_smoothed_detection(int &cx_out, int &cy_out, int &area_out)
{
    int sum_cx = 0, sum_cy = 0, sum_area = 0, count = 0;
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

// ==================== Combined app_main ====================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== APP START ===");

    // 1. Initialize NVS (needed for both camera and BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // 2. Initialize LCD first (it uses SPI, not internal DMA)
    lcd_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "LCD init done");

    // Quick red screen test – allocate from PSRAM, then free
    uint16_t *red = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (red) {
        for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) red[i] = 0xF800;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, red);
        free(red);
        ESP_LOGI(TAG, "Red screen test done");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // 3. Initialize Camera (BEFORE BLE to reserve DMA memory)
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
    camera_config.fb_location = CAMERA_FB_IN_PSRAM; // Frame buffer in PSRAM

    esp_err_t cam_err = esp_camera_init(&camera_config);
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(cam_err));
        // If camera fails, try to reboot or just halt
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1);
        ESP_LOGI(TAG, "Camera flipped");
    } else {
        ESP_LOGW(TAG, "Could not get sensor handle");
    }
    ESP_LOGI(TAG, "Camera initialized OK");

    // 4. Initialize NimBLE (now camera DMA is already allocated)
    nimble_port_init();
    ble_svc_gap_device_name_set("ESP32-Connector");
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ESP_LOGI(TAG, "NimBLE initialized");

    // 5. Start BLE host task
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE host task started");

    // 6. Prepare hand detection model
    ESP_LOGI(TAG, "Loading hand detection model...");
    HandDetect hand_detect(HandDetect::model_type_t::ESPDET_PICO_224_224_HAND);
    hand_detect.set_score_thr(0.5f);
    hand_detect.set_nms_thr(0.45f);
    ESP_LOGI(TAG, "Hand detection model loaded");

    // Watchdog config
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Watchdog configured");

    // 7. Main loop – use camera frame buffer directly
    int frame_count = 0;
    while (true) {
        esp_task_wdt_reset();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Capture failed");
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        if (++frame_count % 100 == 0) {
            ESP_LOGI(TAG, "Frame %d captured, size=%zu", frame_count, fb->len);
        }

        if (fb->len != LCD_H_RES * LCD_V_RES * 2) {
            ESP_LOGW(TAG, "Frame size mismatch: got %zu, expected %d", fb->len, LCD_H_RES * LCD_V_RES * 2);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        // Use fb->buf as both image source and display buffer
        dl::image::img_t hand_img;
        hand_img.width = fb->width;
        hand_img.height = fb->height;
        hand_img.data = fb->buf;
        hand_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

        auto results = hand_detect.run(hand_img);

        int best_cx_img = -1, best_cy_img = -1;
        float best_score = 0.0;
        int best_x1 = 0, best_y1 = 0, best_x2 = 0, best_y2 = 0;

        if (!results.empty()) {
            for (const auto &res : results) {
                if (res.score < 0.35)
                    continue;
                int x1 = res.box[0], y1 = res.box[1], x2 = res.box[2], y2 = res.box[3];
                draw_box((uint16_t *)fb->buf, LCD_H_RES, LCD_V_RES, x1, y1, x2, y2);
                int cx = (x1 + x2) / 2, cy = (y1 + y2) / 2;
                if (res.score > best_score) {
                    best_score = res.score;
                    best_cx_img = cx;
                    best_cy_img = cy;
                    best_x1 = x1;
                    best_y1 = y1;
                    best_x2 = x2;
                    best_y2 = y2;
                }
            }
            if (best_cx_img != -1) {
                int area = (best_x2 - best_x1) * (best_y2 - best_y1);
                add_detection(best_cx_img, best_cy_img, area);
            }
        }

        int smoothed_cx, smoothed_cy, smoothed_area;
        if (get_smoothed_detection(smoothed_cx, smoothed_cy, smoothed_area)) {
            const char *horiz = classify_horizontal(smoothed_cx);
            const char *vert = classify_vertical(smoothed_cy);
            const char *dist = classify_distance(smoothed_area);
            ESP_LOGI(TAG,
                     "Hand: %s-%s, %s (center %d,%d area %d)",
                     horiz,
                     vert,
                     dist,
                     smoothed_cx,
                     smoothed_cy,
                     smoothed_area);
        } else if (frame_count % 100 == 0) {
            ESP_LOGI(TAG, "No hand detected (frame %d)", frame_count);
        }

        // Draw the buffer (already contains boxes) to LCD
        esp_err_t draw_err = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, fb->buf);
        if (draw_err != ESP_OK) {
            ESP_LOGE(TAG, "LCD draw failed: %s", esp_err_to_name(draw_err));
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
