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
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "os/os_mbuf.h"
#include "sdkconfig.h"

#define TAG "BLE-Connect"
#define TARGET_DEVICE_NAME "TD630 SZ-FE9974(BLE)"

uint8_t ble_addr_type;
static bool scanning = false;
static bool connecting = false;
static bool discovery_done = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t char_handle = 0;

// Command sending task handle
static TaskHandle_t cmd_task_handle = NULL;

typedef struct
{
    uint8_t *data;
    uint8_t len;
} ble_command_t;

static const uint8_t cmd1[] = {0x1B, 0x80, 0x4C, 0x03, 0xFF};
static const uint8_t cmd2[] = {0x1B, 0x80, 0x52, 0x00, 0xFF};
static const uint8_t cmd3[] = {0x1B, 0x80, 0x55, 0x03, 0xA0};
static const uint8_t cmd4[] = {0x1B, 0x80, 0x44, 0x00, 0xA0};

static ble_command_t commands[] = {
    {(uint8_t *)cmd1, sizeof(cmd1)},
    {(uint8_t *)cmd2, sizeof(cmd2)},
    {(uint8_t *)cmd3, sizeof(cmd3)},
    {(uint8_t *)cmd4, sizeof(cmd4)},
};
#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

static void ble_start_scan(void);
static void ble_connect_to_target(const ble_addr_t *addr);
static void start_gatt_discovery(void);
static void cmd_send_task(void *pvParameters);

// Write without response
static void write_no_rsp(uint16_t conn_handle, uint16_t attr_handle, uint8_t *data, uint16_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mbuf");
        return;
    }
    int rc = ble_gattc_write_no_rsp(conn_handle, attr_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Write_no_rsp failed: %d", rc);
        os_mbuf_free_chain(om);
    }
    else
    {
        ESP_LOGI(TAG, "Write_no_rsp queued successfully");
    }
}

// Task that sends commands one by one with 5-second delays
static void cmd_send_task(void *pvParameters)
{
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        ESP_LOGI(TAG, "Sending command %d/%d:", i + 1, NUM_COMMANDS);
        ESP_LOG_BUFFER_HEX("CMD", commands[i].data, commands[i].len);
        write_no_rsp(conn_handle, char_handle, commands[i].data, commands[i].len);

        // Wait 5 seconds before next command (except after the last)
        if (i < NUM_COMMANDS - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    ESP_LOGI(TAG, "All commands sent. Disconnecting in 1 second...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ble_gap_terminate(conn_handle, 0);

    cmd_task_handle = NULL;
    vTaskDelete(NULL);
}

// Characteristic discovery callback
static int char_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr != NULL && !discovery_done)
    {
        ESP_LOGI(TAG, "Discovered characteristic: def_handle=%04x, val_handle=%04x, properties=0x%02x",
                 chr->def_handle, chr->val_handle, chr->properties);
        if ((chr->properties & BLE_GATT_CHR_PROP_WRITE) ||
            (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP))
        {
            char_handle = chr->val_handle;
            ESP_LOGI(TAG, "Found writable characteristic, handle=0x%04x", char_handle);
            discovery_done = true;

            // Create the command sending task
            if (cmd_task_handle == NULL)
            {
                xTaskCreate(cmd_send_task, "cmd_send", 4096, NULL, 5, &cmd_task_handle);
                if (cmd_task_handle == NULL)
                {
                    ESP_LOGE(TAG, "Failed to create command task");
                    ble_gap_terminate(conn_handle, 0);
                }
            }
            return 1; // stop further characteristic discovery
        }
    }
    else if (error->status != 0)
    {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
    }
    return 0;
}

// Service discovery callback
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (discovery_done)
        return 1;

    if (error->status == 0 && svc != NULL)
    {
        ESP_LOGI(TAG, "Discovered service: start=%04x, end=%04x", svc->start_handle, svc->end_handle);
        int rc = ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle,
                                         char_disc_cb, NULL);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Error discovering characteristics: %d", rc);
        }
    }
    else if (error->status != 0)
    {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

static void start_gatt_discovery(void)
{
    ESP_LOGI(TAG, "Starting GATT service discovery");
    char_handle = 0;
    discovery_done = false;
    int rc = ble_gattc_disc_all_svcs(conn_handle, svc_disc_cb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start service discovery: %d", rc);
        ble_gap_terminate(conn_handle, 0);
    }
}

// BLE GAP event handler
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
    {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Failed to parse advertisement data");
            return 0;
        }
        if (fields.name != NULL)
        {
            char device_name[32];
            int name_len = fields.name_len;
            if (name_len > sizeof(device_name) - 1)
                name_len = sizeof(device_name) - 1;
            memcpy(device_name, fields.name, name_len);
            device_name[name_len] = '\0';
            ESP_LOGI(TAG, "Found device: %s", device_name);
            if (scanning && !connecting && strcmp(device_name, TARGET_DEVICE_NAME) == 0)
            {
                ESP_LOGI(TAG, "Target device found! Stopping scan and connecting...");
                scanning = false;
                ble_gap_disc_cancel();
                ble_connect_to_target(&event->disc.addr);
            }
        }
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan completed");
        if (scanning)
        {
            ESP_LOGI(TAG, "Target not found, restarting scan...");
            ble_start_scan();
        }
        break;
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection event: status = %d", event->connect.status);
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;
            connecting = false;
            ESP_LOGI(TAG, "Connected to %s successfully!", TARGET_DEVICE_NAME);
            vTaskDelay(pdMS_TO_TICKS(500));
            start_gatt_discovery();
        }
        else
        {
            connecting = false;
            ESP_LOGE(TAG, "Connection failed; restarting scan...");
            ble_start_scan();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected from device; reason = %d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        discovery_done = false;
        // Delete the command task if it exists
        if (cmd_task_handle != NULL)
        {
            vTaskDelete(cmd_task_handle);
            cmd_task_handle = NULL;
        }
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
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error initiating connection: %d", rc);
        connecting = false;
        ble_start_scan();
    }
}

static void ble_start_scan(void)
{
    if (scanning || connecting)
    {
        ESP_LOGI(TAG, "Already scanning or connecting, ignoring start request");
        return;
    }
    struct ble_gap_disc_params disc_params = {
        .passive = 0,
        .itvl = 0x30,
        .window = 0x30,
        .filter_duplicates = 0,
    };
    int rc = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start scan, error code: %d", rc);
    }
    else
    {
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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
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
