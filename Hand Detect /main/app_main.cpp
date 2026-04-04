#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

// Hand detection & camera
#include "dl_image.hpp"
#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "hand_detect.hpp"
#include "driver/ledc.h"

// LCD functions
#include "lcd_display.h"

// =================== BLE Definitions ===================
#define BLE_TAG "BLE-Connect"
#define TARGET_DEVICE_NAME "TD630 SZ-FE9974(BLE)"

static uint8_t ble_addr_type;
static bool scanning = false;
static bool connecting = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// GATT discovery & command handle
static bool gatt_ready = false;
static uint16_t cmd_char_handle = 0;

// Command byte sequences
static const uint8_t CMD_LEFT[] = {0x1B, 0x80, 0x4C, 0x03, 0xFF};
static const uint8_t CMD_RIGHT[] = {0x1B, 0x80, 0x52, 0x00, 0xFF};
static const uint8_t CMD_UP[] = {0x1B, 0x80, 0x55, 0x03, 0xA0};
static const uint8_t CMD_DOWN[] = {0x1B, 0x80, 0x44, 0x00, 0xA0};

// Gesture tracking
static const char *last_horiz = "Center";
static const char *last_vert = "Center";
static uint32_t last_cmd_time = 0;
static const uint32_t CMD_DELAY_MS = 100;

// Forward declarations
static void ble_start_scan(void);
static void ble_connect_to_target(const ble_addr_t *addr);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_on_sync(void);
static void host_task(void *param);
static int gatt_disc_svc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service,
                            void *arg);
static int gatt_disc_chr_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr,
                            void *arg);
static void ble_send_command(const uint8_t *data, size_t len);

// =================== Hand Detection Helpers ===================
static const char *TAG = "hand_detect_main";

#define AREA_TOO_CLOSE (0.30f * LCD_H_RES * LCD_V_RES)
#define AREA_TOO_FAR (0.05f * LCD_H_RES * LCD_V_RES)
#define SMOOTHING_WINDOW 5
#define MIN_VALID_DETECTIONS 3
#define MAX_NO_DETECT_BEFORE_RESET 10

struct SmoothedDetection {
    int cx = -1, cy = -1, area = -1;
    bool valid = false;
};
static SmoothedDetection detection_history[SMOOTHING_WINDOW];
static int history_index = 0;
static int no_detection_counter = 0;

static void add_detection(int cx, int cy, int area)
{
    detection_history[history_index].cx = cx;
    detection_history[history_index].cy = cy;
    detection_history[history_index].area = area;
    detection_history[history_index].valid = true;
    history_index = (history_index + 1) % SMOOTHING_WINDOW;
    no_detection_counter = 0;
}

static void clear_history()
{
    for (int i = 0; i < SMOOTHING_WINDOW; i++) detection_history[i].valid = false;
    history_index = 0;
    no_detection_counter = 0;
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

// ==================== Gesture Detection & Command Trigger ====================
static void handle_gesture(const char *current_horiz, const char *current_vert)
{
    // Horizontal transitions – SWAPPED left/right commands
    if ((strcmp(last_horiz, "Right") == 0 && strcmp(current_horiz, "Left") == 0) ||
        (strcmp(last_horiz, "Center") == 0 && strcmp(current_horiz, "Left") == 0)) {
        // Moving to Left now sends RIGHT command
        ble_send_command(CMD_RIGHT, sizeof(CMD_RIGHT));
    } else if ((strcmp(last_horiz, "Left") == 0 && strcmp(current_horiz, "Right") == 0) ||
               (strcmp(last_horiz, "Center") == 0 && strcmp(current_horiz, "Right") == 0)) {
        // Moving to Right now sends LEFT command
        ble_send_command(CMD_LEFT, sizeof(CMD_LEFT));
    }
    // Vertical transitions unchanged
    else if ((strcmp(last_vert, "Down") == 0 && strcmp(current_vert, "Up") == 0) ||
             (strcmp(last_vert, "Center") == 0 && strcmp(current_vert, "Up") == 0)) {
        ble_send_command(CMD_UP, sizeof(CMD_UP));
    } else if ((strcmp(last_vert, "Up") == 0 && strcmp(current_vert, "Down") == 0) ||
               (strcmp(last_vert, "Center") == 0 && strcmp(current_vert, "Down") == 0)) {
        ble_send_command(CMD_DOWN, sizeof(CMD_DOWN));
    }

    last_horiz = current_horiz;
    last_vert = current_vert;
}

// ==================== BLE GATT Client Implementation ====================
static int gatt_disc_svc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service,
                            void *arg)
{
    if (error->status == 0 && service && !gatt_ready) {
        ESP_LOGI(BLE_TAG, "Discovering chars for service handle %04x", service->start_handle);
        int rc =
            ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, gatt_disc_chr_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(BLE_TAG, "Error starting char discovery: %d", rc);
        }
    } else if (error->status != 0) {
        ESP_LOGE(BLE_TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

static int gatt_disc_chr_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr,
                            void *arg)
{
    if (error->status == 0 && chr && !gatt_ready) {
        ESP_LOGI(BLE_TAG, "Char: handle=%04x, properties=0x%02x", chr->val_handle, chr->properties);
        // Check for WRITE or WRITE_NO_RSP (as in working code)
        if ((chr->properties & BLE_GATT_CHR_PROP_WRITE) || (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP)) {
            cmd_char_handle = chr->val_handle;
            gatt_ready = true;
            ESP_LOGI(BLE_TAG, "Found writable characteristic, handle=0x%04x", cmd_char_handle);
            return 1; // stop further discovery
        }
    } else if (error->status != 0) {
        ESP_LOGE(BLE_TAG, "Characteristic discovery error: %d", error->status);
    }
    return 0;
}

static void ble_send_command(const uint8_t *data, size_t len)
{
    if (!gatt_ready || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(BLE_TAG, "Cannot send command: not ready");
        return;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - last_cmd_time) < CMD_DELAY_MS) {
        ESP_LOGI(BLE_TAG, "Command cooldown, skipping");
        return;
    }

    int rc = ble_gattc_write_no_rsp_flat(conn_handle, cmd_char_handle, data, len);
    if (rc == 0) {
        last_cmd_time = now;
        ESP_LOGI(BLE_TAG, "Command sent");
    } else {
        ESP_LOGE(BLE_TAG, "Write failed, error: %d", rc);
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0)
            break;
        if (fields.name != NULL) {
            char device_name[32];
            int name_len = fields.name_len;
            if (name_len > sizeof(device_name) - 1)
                name_len = sizeof(device_name) - 1;
            memcpy(device_name, fields.name, name_len);
            device_name[name_len] = '\0';
            if (scanning && !connecting && strcmp(device_name, TARGET_DEVICE_NAME) == 0) {
                ESP_LOGI(BLE_TAG, "Target found! Stopping scan and connecting...");
                scanning = false;
                ble_gap_disc_cancel();
                ble_connect_to_target(&event->disc.addr);
            }
        }
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (scanning)
            ble_start_scan();
        break;
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            connecting = false;
            ESP_LOGI(BLE_TAG, "Connected to %s", TARGET_DEVICE_NAME);
            // Wait 500ms before starting GATT discovery (as in working code)
            vTaskDelay(pdMS_TO_TICKS(500));
            gatt_ready = false;
            cmd_char_handle = 0;
            int rc = ble_gattc_disc_all_svcs(conn_handle, gatt_disc_svc_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(BLE_TAG, "Failed to start service discovery: %d", rc);
                ble_gap_terminate(conn_handle, 0);
            }
        } else {
            connecting = false;
            ble_start_scan();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        gatt_ready = false;
        cmd_char_handle = 0;
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
        ESP_LOGE(BLE_TAG, "Connect error: %d", rc);
        connecting = false;
        ble_start_scan();
    }
}

static void ble_start_scan(void)
{
    if (scanning || connecting)
        return;

    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 0;
    disc_params.itvl = 0x30;
    disc_params.window = 0x30;
    disc_params.filter_duplicates = 0;

    int rc = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc == 0) {
        scanning = true;
        ESP_LOGI(BLE_TAG, "Scanning for %s", TARGET_DEVICE_NAME);
    } else {
        ESP_LOGE(BLE_TAG, "Scan start failed: %d", rc);
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

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== APP START ===");

    // 1. NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. LCD init
    lcd_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    uint16_t *red = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (red) {
        for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) red[i] = 0xF800;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, red);
        free(red);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    test_pattern_color_cycle();

    // 3. Camera init with improved settings
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
    vTaskDelay(pdMS_TO_TICKS(200));

    // ---- TUNE CAMERA SENSOR FOR BETTER HAND DETECTION ----
    // ---- TUNE CAMERA SENSOR FOR LOW LIGHT HAND DETECTION ----
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Flip vertically (adjust if needed)
        s->set_vflip(s, 1);

        // ----- Aggressive brightness & contrast for low light -----
        s->set_brightness(s, 2); // range -2 to 2, max brightness
        s->set_contrast(s, 2);   // max contrast to separate hand from background
        s->set_sharpness(s, 2);  // max sharpness for clearer edges

        // ----- Exposure & Gain tuning -----
        // Enable auto exposure, but increase its target level
        s->set_exposure_ctrl(s, 1); // auto exposure ON
        s->set_aec2(s, 1);          // advanced AEC (if supported)
        s->set_ae_level(s, 2);      // exposure compensation: +2 (makes image brighter)

        // Manual gain control – disable auto gain and set high fixed gain
        s->set_gain_ctrl(s, 0); // disable auto gain
        s->set_agc_gain(s, 30); // manual gain (typical max 30, adjust 0-30)

        // Alternative: keep auto gain but set higher ceiling – uncomment next lines
        // s->set_gain_ctrl(s, 1);        // auto gain ON
        // s->set_agc_gain(s, 30);        // maximum allowed gain (0-30)

        // White balance (auto)
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);

        // Special effect off
        s->set_special_effect(s, 0);

        ESP_LOGI(TAG, "Low‑light camera tuning: brightness=2, contrast=2, sharpness=2, ae_level=2, manual_gain=30");
    } else {
        ESP_LOGW(TAG, "Could not get camera sensor handle");
    }
    // 4. BLE init (unchanged)
    nimble_port_init();
    ble_svc_gap_device_name_set("ESP32-Connector");
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE host started");

    // 5. Hand detection model with LOWER SCORE THRESHOLD for low light
    HandDetect hand_detect(HandDetect::model_type_t::ESPDET_PICO_224_224_HAND);
    hand_detect.set_score_thr(0.2f); // lowered from 0.3 to 0.2
    hand_detect.set_nms_thr(0.45f);

    // 6. LCD buffer (unchanged)
    uint16_t *lcd_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!lcd_buffer) {
        ESP_LOGW(TAG, "DMA allocation failed, trying SPIRAM");
        lcd_buffer = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    }
    if (!lcd_buffer) {
        ESP_LOGE(TAG, "Failed to allocate LCD buffer");
        return;
    }

    // 7. Watchdog (unchanged)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);

    // 8. Main loop (with added minimum area filter)
    const int MIN_HAND_AREA = 500; // reject detections smaller than 500 pixels (noise)

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

        memcpy(lcd_buffer, fb->buf, expected_size);
        static bool gamma_init = false;
        static uint16_t gamma_lut[65536];
        if (!gamma_init) {
            for (int i = 0; i < 65536; i++) {
                float r = ((i >> 11) & 0x1F) / 31.0f;
                float g = ((i >> 5) & 0x3F) / 63.0f;
                float b = (i & 0x1F) / 31.0f;
                float gamma = 0.45f; // typical low‑light gamma
                r = powf(r, gamma);
                g = powf(g, gamma);
                b = powf(b, gamma);
                uint16_t r5 = (uint16_t)(r * 31);
                uint16_t g6 = (uint16_t)(g * 63);
                uint16_t b5 = (uint16_t)(b * 31);
                gamma_lut[i] = (r5 << 11) | (g6 << 5) | b5;
            }
            gamma_init = true;
        }
        for (size_t i = 0; i < expected_size / 2; i++) {
            ((uint16_t *)lcd_buffer)[i] = gamma_lut[((uint16_t *)lcd_buffer)[i]];
        }

        dl::image::img_t hand_img;
        hand_img.width = fb->width;
        hand_img.height = fb->height;
        hand_img.data = fb->buf;
        hand_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

        auto results = hand_detect.run(hand_img);

        int best_cx_img = -1, best_cy_img = -1;
        float best_score = 0.0;
        int best_x1 = 0, best_y1 = 0, best_x2 = 0, best_y2 = 0;

        if (results.empty()) {
            no_detection_counter++;
            if (no_detection_counter >= MAX_NO_DETECT_BEFORE_RESET) {
                clear_history();
                ESP_LOGI(TAG, "History reset due to prolonged no-detection");
            } else {
                detection_history[history_index].valid = false;
                history_index = (history_index + 1) % SMOOTHING_WINDOW;
            }
        } else {
            for (const auto &res : results) {
                // Use the new lower threshold (already set in model, but double-check)
                if (res.score < 0.3f)
                    continue;
                int x1 = res.box[0], y1 = res.box[1], x2 = res.box[2], y2 = res.box[3];
                int area = (x2 - x1) * (y2 - y1);
                // Reject very small boxes (likely noise)
                if (area < MIN_HAND_AREA)
                    continue;
                draw_box(lcd_buffer, LCD_H_RES, LCD_V_RES, x1, y1, x2, y2);
                int cx = (x1 + x2) / 2;
                int cy = (y1 + y2) / 2;
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
            } else {
                // No valid detection after area filter → treat as no detection
                no_detection_counter++;
                if (no_detection_counter >= MAX_NO_DETECT_BEFORE_RESET) {
                    clear_history();
                } else {
                    detection_history[history_index].valid = false;
                    history_index = (history_index + 1) % SMOOTHING_WINDOW;
                }
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
            handle_gesture(horiz, vert);
        }

        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer);
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}