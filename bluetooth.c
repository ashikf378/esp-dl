#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "sdkconfig.h"

#define TAG "BLE-Connect"
#define TARGET_DEVICE_NAME "TD630 SZ-FE9974(BLE)"

uint8_t ble_addr_type;
static bool scanning = false;
static bool connecting = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Forward declarations
static void ble_start_scan(void);
static void ble_connect_to_target(const ble_addr_t *addr);

// Helper to convert BLE address to string (for logging)
static void ble_addr_to_str_custom(char *dst, size_t len, const ble_addr_t *addr)
{
    snprintf(dst, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
    {
        struct ble_hs_adv_fields fields;
        int rc;

        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to parse advertisement data");
            return 0;
        }

        // Check if the device advertises a name
        if (fields.name != NULL) {
            char device_name[32];
            int name_len = fields.name_len;
            if (name_len > sizeof(device_name) - 1)
                name_len = sizeof(device_name) - 1;
            memcpy(device_name, fields.name, name_len);
            device_name[name_len] = '\0';

            ESP_LOGI(TAG, "Found device: %s", device_name);

            // If we are still scanning and this is our target, stop scan and connect
            if (scanning && !connecting && strcmp(device_name, TARGET_DEVICE_NAME) == 0) {
                ESP_LOGI(TAG, "Target device found! Stopping scan and connecting...");
                scanning = false;
                ble_gap_disc_cancel();          // Stop scanning
                ble_connect_to_target(&event->disc.addr);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan completed");
        // If we never found the target, restart scan
        if (scanning) {
            ESP_LOGI(TAG, "Target not found, restarting scan...");
            ble_start_scan();
        }
        break;

    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection event: status = %d", event->connect.status);
        if (event->connect.status == 0) {
            // Connection successful
            conn_handle = event->connect.conn_handle;
            connecting = false;
            ESP_LOGI(TAG, "Connected to %s successfully!", TARGET_DEVICE_NAME);
            // Optionally, you can now perform GATT discovery, etc.
        } else {
            // Connection failed
            connecting = false;
            ESP_LOGE(TAG, "Connection failed; restarting scan...");
            ble_start_scan();   // Retry scanning
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected from device; reason = %d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        // Re-scan to try to reconnect
        ESP_LOGI(TAG, "Restarting scan...");
        ble_start_scan();
        break;

    default:
        break;
    }
    return 0;
}

static void ble_connect_to_target(const ble_addr_t *addr)
{
    // Prepare connection parameters
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x30,
        .scan_window = 0x30,
        .itvl_min = 0x30,
        .itvl_max = 0x30,
        .latency = 0,
        .supervision_timeout = 0x200,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    connecting = true;
    int rc = ble_gap_connect(ble_addr_type, addr, BLE_HS_FOREVER, &conn_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating connection: %d", rc);
        connecting = false;
        ble_start_scan();   // Retry scanning
    }
}

static void ble_start_scan(void)
{
    if (scanning || connecting) {
        ESP_LOGI(TAG, "Already scanning or connecting, ignoring start request");
        return;
    }

    struct ble_gap_disc_params disc_params = {
        .passive = 0,               // Active scan to get device name
        .itvl = 0x30,
        .window = 0x30,
        .filter_duplicates = 0,
    };
    int rc = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scan, error code: %d", rc);
    } else {
        scanning = true;
        ESP_LOGI(TAG, "Scanning for target device: %s", TARGET_DEVICE_NAME);
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

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    ble_svc_gap_device_name_set("ESP32-Connector");
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
}
