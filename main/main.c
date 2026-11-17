#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ble_transport.h"
#include "meter_mqtt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9488.h"
#include "dlms_obis.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "dlms_hdlc.h"
#include "dlms_cosem.h"
#include "dlms_link.h"
#include "tft_digit_font.h"
#include <inttypes.h>  


static const char *TAG = "MAIN";
static const char WIFI_PROVISIONING_AP_SSID[] = "smart meter";

#define REGISTER_COUNT 12U
#define METER_READ_INTERVAL_SECONDS 300U
#define WIFI_PROVISIONING_AP_TIMEOUT_MS (5U * 60U * 1000U)


extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t wifi_setup_html_start[] asm("_binary_wifi_setup_html_start");
extern const uint8_t wifi_setup_html_end[] asm("_binary_wifi_setup_html_end");

static double s_meter_values[REGISTER_COUNT];
static bool s_meter_value_valid[REGISTER_COUNT];
static TickType_t s_meter_last_update_tick;
static char s_wifi_ip[16];
static char s_wifi_ssid[33];
static char s_wifi_password[65];
static char s_wifi_candidate_ssid[33];
static char s_wifi_candidate_password[65];
static uint8_t s_ble_selected_address[6];
static uint8_t s_ble_selected_address_type;
static char s_ble_device_name[BLE_TRANSPORT_DEVICE_NAME_LENGTH] =
    "Factory default";
static bool s_ble_selection_saved;
static bool s_ble_selection_cleared;
static bool s_wifi_candidate_pending;
static bool s_wifi_candidate_failed;
static TickType_t s_wifi_candidate_started_tick;
static bool s_provisioning_active;
static bool s_wifi_setup_completed;
static bool s_wifi_provisioning_timed_out;
static bool s_tft_auto_rotate = true;
static uint8_t s_wifi_ap_station_count;
static uint32_t s_wifi_provisioning_generation;
static bool s_wifi_scan_running;
static bool s_wifi_scan_ready;
static bool s_wifi_scan_failed;
static uint16_t s_wifi_scan_count;
static wifi_ap_record_t s_wifi_scan_records[20];
static wifi_ap_record_t s_wifi_scan_work_records[20];
static char s_wifi_scan_response[2048];
static esp_netif_t *s_wifi_ap_netif;
static TaskHandle_t s_dns_task_handle;
static portMUX_TYPE s_dashboard_mux = portMUX_INITIALIZER_UNLOCKED;
static httpd_handle_t s_dashboard_server;

static esp_err_t stop_wifi_provisioning_ap(void);
static void wifi_provisioning_timeout_task(void *arg);

static const char *const s_meter_units[REGISTER_COUNT] = {
    "kWh", "kW", "V", "A", "Hz", "", "kWh", "kvar", "kVA",
    "kW", "kvarh", "kvarh", // "kW"
};

static const char *const s_meter_names[REGISTER_COUNT] = {
    "Import energy",
    "Active power",
    "Voltage L1",
    "Current L1",
    "Frequency", 
    "Power factor",
    "Export energy",
      "Reactive power",
    "Apparent power",
     "Export active power",
      "Import reactive energy",
    "Export reactive energy"
    //  "Maximum demand"
};

static void store_meter_value(size_t index, double value)
{
    if (index >= REGISTER_COUNT) {
        return;
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_meter_values[index] = value;
    s_meter_value_valid[index] = true;
    s_meter_last_update_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_dashboard_mux);
}

static void invalidate_meter_value(size_t index)
{
    if (index >= REGISTER_COUNT) {
        return;
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_meter_value_valid[index] = false;
    portEXIT_CRITICAL(&s_dashboard_mux);
}

static bool append_json(char *buffer, size_t capacity, size_t *length,
                        const char *format, ...)
{
    if (*length >= capacity) {
        return false;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(buffer + *length, capacity - *length, format, args);
    va_end(args);

    if (written < 0 || (size_t)written >= capacity - *length) {
        return false;
    }

    *length += (size_t)written;
    return true;
}

static bool append_json_string(char *buffer, size_t capacity, size_t *length,
                               const char *value)
{
    if (!append_json(buffer, capacity, length, "\"")) {
        return false;
    }
    for (const unsigned char *cursor = (const unsigned char *)value;
         *cursor != '\0'; ++cursor) {
        if (*cursor == '"' || *cursor == '\\') {
            if (!append_json(buffer, capacity, length, "\\%c", *cursor)) {
                return false;
            }
        } else if (*cursor < 0x20U) {
            if (!append_json(buffer, capacity, length, "\\u%04x",
                             (unsigned)*cursor)) {
                return false;
            }
        } else if (!append_json(buffer, capacity, length, "%c", *cursor)) {
            return false;
        }
    }
    return append_json(buffer, capacity, length, "\"");
}

static bool wifi_provisioning_is_active(void)
{
    bool active;
    portENTER_CRITICAL(&s_dashboard_mux);
    active = s_provisioning_active;
    portEXIT_CRITICAL(&s_dashboard_mux);
    return active;
}

static esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open Wi-Fi credential storage: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(nvs, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "password", password);
    }
    if (err == ESP_OK) {
        err = nvs_erase_key(nvs, "cleared");
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Wi-Fi credentials: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t load_wifi_credentials(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open Wi-Fi credential storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    uint8_t credentials_cleared = 0U;
    err = nvs_get_u8(nvs, "cleared", &credentials_cleared);
    if (err == ESP_OK && credentials_cleared != 0U) {
        s_wifi_ssid[0] = '\0';
        s_wifi_password[0] = '\0';
        nvs_close(nvs);
        return ESP_OK;
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to read Wi-Fi reset state: %s",
                 esp_err_to_name(err));
        nvs_close(nvs);
        return err;
    }

    size_t ssid_length = sizeof(s_wifi_ssid);
    size_t password_length = sizeof(s_wifi_password);
    err = nvs_get_str(nvs, "ssid", s_wifi_ssid, &ssid_length);
    if (err == ESP_OK) {
        err = nvs_get_str(nvs, "password", s_wifi_password,
                          &password_length);
    }
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_wifi_ssid[0] = '\0';
        s_wifi_password[0] = '\0';
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load Wi-Fi credentials: %s",
                 esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

static esp_err_t clear_wifi_credentials(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open Wi-Fi credential storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_key(nvs, "ssid");
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_erase_key(nvs, "password");
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear Wi-Fi credentials: %s",
                 esp_err_to_name(err));
        return err;
    }

    memset(s_wifi_ssid, 0, sizeof(s_wifi_ssid));
    memset(s_wifi_password, 0, sizeof(s_wifi_password));
    memset(s_wifi_candidate_ssid, 0, sizeof(s_wifi_candidate_ssid));
    memset(s_wifi_candidate_password, 0, sizeof(s_wifi_candidate_password));
    portENTER_CRITICAL(&s_dashboard_mux);
    s_wifi_candidate_pending = false;
    s_wifi_candidate_failed = false;
    s_wifi_ip[0] = '\0';
    portEXIT_CRITICAL(&s_dashboard_mux);
    ESP_LOGI(TAG, "Saved Wi-Fi SSID and password cleared");
    return ESP_OK;
}

static esp_err_t load_ble_selection(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ble", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_ble_selection_saved = false;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open BLE selection storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    uint8_t selection_cleared = 0U;
    err = nvs_get_u8(nvs, "cleared", &selection_cleared);
    if (err == ESP_OK && selection_cleared != 0U) {
        nvs_close(nvs);
        s_ble_selection_saved = false;
        s_ble_selection_cleared = true;
        (void)snprintf(s_ble_device_name, sizeof(s_ble_device_name),
                       "Not selected");
        return ESP_OK;
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        ESP_LOGE(TAG, "Failed to read BLE selection reset state: %s",
                 esp_err_to_name(err));
        return err;
    }

    size_t address_length = sizeof(s_ble_selected_address);
    err = nvs_get_blob(nvs, "address", s_ble_selected_address,
                       &address_length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        s_ble_selection_saved = false;
        s_ble_selection_cleared = false;
        return ESP_OK;
    }
    if (err == ESP_OK && address_length != sizeof(s_ble_selected_address)) {
        err = ESP_ERR_INVALID_SIZE;
    }
    if (err == ESP_OK) {
        err = nvs_get_u8(nvs, "addr_type", &s_ble_selected_address_type);
    }
    if (err == ESP_OK) {
        size_t name_length = sizeof(s_ble_device_name);
        err = nvs_get_str(nvs, "name", s_ble_device_name, &name_length);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load BLE selection: %s",
                 esp_err_to_name(err));
        return err;
    }
    if (s_ble_selected_address_type > 3U) {
        ESP_LOGE(TAG, "Stored BLE address type is invalid");
        return ESP_ERR_INVALID_STATE;
    }

    s_ble_selection_saved = true;
    s_ble_selection_cleared = false;
    return ESP_OK;
}

static esp_err_t save_ble_selection(const uint8_t address[6],
                                    uint8_t address_type, const char *name)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ble", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open BLE selection storage: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(nvs, "address", address, 6U);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "addr_type", address_type);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "name", name);
    }
    if (err == ESP_OK) {
        err = nvs_erase_key(nvs, "cleared");
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save BLE selection: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t delete_ble_selection(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ble", NVS_READWRITE, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "BLE selection storage is unavailable for deletion");
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open BLE selection storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    const char *const keys[] = {"address", "addr_type", "name"};
    for (size_t i = 0U; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        err = nvs_erase_key(nvs, keys[i]);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
        if (err != ESP_OK) {
            break;
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "cleared", 1U);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete BLE selection: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static void wifi_dns_server_task(void *arg)
{
    (void)arg;
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create captive DNS socket");
        s_dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct timeval timeout = {.tv_sec = 0, .tv_usec = 500000};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(53U),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        ESP_LOGE(TAG, "Failed to bind captive DNS port 53");
        closesocket(socket_fd);
        s_dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    uint8_t query[512];
    uint8_t response[512];
    while (wifi_provisioning_is_active()) {
        struct sockaddr_in source;
        socklen_t source_length = sizeof(source);
        int received = recvfrom(socket_fd, query, sizeof(query), 0,
                                (struct sockaddr *)&source, &source_length);
        if (received < 17 || (query[2] & 0x80U) != 0U) {
            continue;
        }

        size_t question_end = 12U;
        while (question_end < (size_t)received &&
               query[question_end] != 0U) {
            const uint8_t label_length = query[question_end];
            if ((label_length & 0xC0U) != 0U ||
                question_end + (size_t)label_length + 1U >=
                    (size_t)received) {
                question_end = 0U;
                break;
            }
            question_end += (size_t)label_length + 1U;
        }
        if (question_end == 0U || question_end + 5U > (size_t)received) {
            continue;
        }
        question_end += 5U;

        memcpy(response, query, question_end);
        response[2] = 0x81U;
        response[3] = 0x80U;
        response[6] = 0U;
        response[7] = 1U;
        response[8] = 0U;
        response[9] = 0U;
        response[10] = 0U;
        response[11] = 0U;
        static const uint8_t answer[] = {
            0xC0U, 0x0CU, 0x00U, 0x01U, 0x00U, 0x01U,
            0x00U, 0x00U, 0x00U, 0x3CU, 0x00U, 0x04U,
            192U, 168U, 4U, 1U
        };
        memcpy(response + question_end, answer, sizeof(answer));
        (void)sendto(socket_fd, response, question_end + sizeof(answer), 0,
                     (struct sockaddr *)&source, source_length);
    }

    closesocket(socket_fd);
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t start_wifi_provisioning_ap(void)
{
    if (wifi_provisioning_is_active()) {
        return ESP_OK;
    }
    if (s_wifi_ap_netif == NULL) {
        s_wifi_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_wifi_ap_netif == NULL) {
            ESP_LOGE(TAG, "Failed to create Wi-Fi provisioning AP interface");
            return ESP_FAIL;
        }
    }

    wifi_config_t ap_config = {0};
    memcpy(ap_config.ap.ssid, WIFI_PROVISIONING_AP_SSID,
           sizeof(WIFI_PROVISIONING_AP_SSID));
    ap_config.ap.ssid_len = sizeof(WIFI_PROVISIONING_AP_SSID) - 1U;
    ap_config.ap.channel = 1U;
    ap_config.ap.ssid_hidden = 0U;
    ap_config.ap.max_connection = 4U;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure provisioning AP: %s",
                 esp_err_to_name(err));
        return err;
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_provisioning_active = true;
    s_wifi_setup_completed = false;
    s_wifi_provisioning_timed_out = false;
    s_wifi_ap_station_count = 0U;
    const uint32_t generation = ++s_wifi_provisioning_generation;
    portEXIT_CRITICAL(&s_dashboard_mux);
    if (xTaskCreate(wifi_provisioning_timeout_task, "wifi_ap_timeout",
                    2048U, (void *)(uintptr_t)generation, 4U, NULL) !=
        pdPASS) {
        ESP_LOGE(TAG, "Failed to schedule provisioning AP timeout");
        portENTER_CRITICAL(&s_dashboard_mux);
        s_wifi_provisioning_timed_out = true;
        portEXIT_CRITICAL(&s_dashboard_mux);
        return stop_wifi_provisioning_ap();
    }
    if (s_dns_task_handle == NULL &&
        xTaskCreate(wifi_dns_server_task, "wifi_captive_dns", 3072U, NULL,
                    4U, &s_dns_task_handle) != pdPASS) {
        s_dns_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to start captive DNS task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t stop_wifi_provisioning_ap(void)
{
    if (!wifi_provisioning_is_active()) {
        return ESP_OK;
    }
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop provisioning AP: %s",
                 esp_err_to_name(err));
    } else {
        portENTER_CRITICAL(&s_dashboard_mux);
        s_provisioning_active = false;
        s_wifi_ap_station_count = 0U;
        ++s_wifi_provisioning_generation;
        portEXIT_CRITICAL(&s_dashboard_mux);
        ESP_LOGI(TAG, "Provisioning AP '%s' stopped",
                 WIFI_PROVISIONING_AP_SSID);
    }
    return err;
}

static void wifi_provisioning_timeout_task(void *arg)
{
    const uint32_t generation = (uint32_t)(uintptr_t)arg;
    const TickType_t timeout = pdMS_TO_TICKS(
        WIFI_PROVISIONING_AP_TIMEOUT_MS);
    const TickType_t started = xTaskGetTickCount();

    for (;;) {
        portENTER_CRITICAL(&s_dashboard_mux);
        const bool current = s_provisioning_active &&
            s_wifi_provisioning_generation == generation;
        portEXIT_CRITICAL(&s_dashboard_mux);
        if (!current) {
            break;
        }

        const TickType_t elapsed = xTaskGetTickCount() - started;
        if (elapsed >= timeout) {
            portENTER_CRITICAL(&s_dashboard_mux);
            const bool should_stop = s_provisioning_active &&
                s_wifi_provisioning_generation == generation;
            if (should_stop) {
                s_wifi_provisioning_timed_out = true;
            }
            portEXIT_CRITICAL(&s_dashboard_mux);

            if (should_stop) {
                ESP_LOGW(TAG,
                         "Provisioning AP timed out after 5 minutes; "
                         "it will remain off until Wi-Fi connects or device restarts");
                while (stop_wifi_provisioning_ap() != ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(2000U));
                    portENTER_CRITICAL(&s_dashboard_mux);
                    const bool retry = s_provisioning_active &&
                        s_wifi_provisioning_generation == generation;
                    portEXIT_CRITICAL(&s_dashboard_mux);
                    if (!retry) {
                        break;
                    }
                }
            }
            break;
        }

        TickType_t remaining = timeout - elapsed;
        const TickType_t check_interval = pdMS_TO_TICKS(1000U);
        if (remaining > check_interval) {
            remaining = check_interval;
        }
        vTaskDelay(remaining);
    }
    vTaskDelete(NULL);
}

static esp_err_t dashboard_index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

static esp_err_t wifi_setup_page_handler(httpd_req_t *request);

static esp_err_t wifi_root_handler(httpd_req_t *request)
{
    if (wifi_provisioning_is_active()) {
        return wifi_setup_page_handler(request);
    }
    return dashboard_index_handler(request);
}

static esp_err_t wifi_setup_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, (const char *)wifi_setup_html_start,
                           wifi_setup_html_end - wifi_setup_html_start);
}

static void wifi_scan_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting Wi-Fi network scan");
    wifi_scan_config_t scan_config = {0};
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    uint16_t count = 0U;
    memset(s_wifi_scan_work_records, 0, sizeof(s_wifi_scan_work_records));
    if (err == ESP_OK) {
        err = esp_wifi_scan_get_ap_num(&count);
    }
    if (err == ESP_OK) {
        if (count > sizeof(s_wifi_scan_work_records) /
                        sizeof(s_wifi_scan_work_records[0])) {
            count = sizeof(s_wifi_scan_work_records) /
                    sizeof(s_wifi_scan_work_records[0]);
        }
        if (count > 0U) {
            err = esp_wifi_scan_get_ap_records(&count,
                                               s_wifi_scan_work_records);
        }
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_wifi_scan_failed = err != ESP_OK;
    s_wifi_scan_count = err == ESP_OK ? count : 0U;
    s_wifi_scan_ready = true;
    if (err == ESP_OK) {
        memcpy(s_wifi_scan_records, s_wifi_scan_work_records,
               (size_t)count * sizeof(s_wifi_scan_records[0]));
    }
    s_wifi_scan_running = false;
    portEXIT_CRITICAL(&s_dashboard_mux);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Wi-Fi scan completed; %u networks found",
                 (unsigned int)count);
    }
    vTaskDelete(NULL);
}

static esp_err_t wifi_scan_handler(httpd_req_t *request)
{
    if (!wifi_provisioning_is_active()) {
        return httpd_resp_send_err(request, HTTPD_403_FORBIDDEN,
                                   "Wi-Fi setup is not active");
    }

    const bool refresh = strstr(request->uri, "refresh=1") != NULL;
    portENTER_CRITICAL(&s_dashboard_mux);
    bool start_scan = false;
    if (refresh && !s_wifi_scan_running) {
        s_wifi_scan_ready = false;
        s_wifi_scan_failed = false;
        s_wifi_scan_count = 0U;
    }
    if (!s_wifi_scan_running && !s_wifi_scan_ready) {
        s_wifi_scan_running = true;
        s_wifi_scan_failed = false;
        s_wifi_scan_count = 0U;
        start_scan = true;
    }
    portEXIT_CRITICAL(&s_dashboard_mux);

    if (start_scan &&
        xTaskCreate(wifi_scan_task, "wifi_scan", 8192U, NULL, 4U, NULL) !=
            pdPASS) {
        portENTER_CRITICAL(&s_dashboard_mux);
        s_wifi_scan_running = false;
        portEXIT_CRITICAL(&s_dashboard_mux);
        ESP_LOGE(TAG, "Failed to create Wi-Fi scan task");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to start Wi-Fi scan");
    }

    uint16_t count;
    bool scanning;
    bool failed;
    portENTER_CRITICAL(&s_dashboard_mux);
    scanning = s_wifi_scan_running;
    failed = s_wifi_scan_failed;
    count = s_wifi_scan_count;
    portEXIT_CRITICAL(&s_dashboard_mux);

    char *response = s_wifi_scan_response;
    size_t length = 0U;
    if (scanning) {
        static const char scanning_response[] = "{\"scanning\":true}";
        httpd_resp_set_type(request, "application/json");
        httpd_resp_set_hdr(request, "Cache-Control", "no-store");
        return httpd_resp_send(request, scanning_response,
                               sizeof(scanning_response) - 1U);
    }
    if (failed) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Wi-Fi scan failed; try again");
    }

    bool ok = append_json(response, sizeof(s_wifi_scan_response), &length,
                          "{\"scanning\":false,\"networks\":[");
    bool first_network = true;
    for (uint16_t i = 0U; ok && i < count; ++i) {
        if (s_wifi_scan_records[i].ssid[0] == '\0') {
            continue;
        }
        ok = append_json(response, sizeof(s_wifi_scan_response), &length,
                         "%s{\"ssid\":", first_network ? "" : ",");
        first_network = false;
        if (ok) {
            char ssid[sizeof(s_wifi_scan_records[i].ssid) + 1U];
            memcpy(ssid, s_wifi_scan_records[i].ssid,
                   sizeof(s_wifi_scan_records[i].ssid));
            ssid[sizeof(s_wifi_scan_records[i].ssid)] = '\0';
            ok = append_json_string(response, sizeof(s_wifi_scan_response), &length,
                                    ssid);
        }
        if (ok) {
            ok = append_json(response, sizeof(s_wifi_scan_response), &length,
                             ",\"rssi\":%d}",
                             (int)s_wifi_scan_records[i].rssi);
        }
    }
    ok = ok && append_json(response, sizeof(s_wifi_scan_response), &length,
                           "]}");
    if (!ok) {
        ESP_LOGE(TAG, "Wi-Fi scan response buffer is too small");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Scan response is too large");
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, (ssize_t)length);
}

static int wifi_hex_value(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    return -1;
}

static bool wifi_form_decode_value(const char *encoded, size_t encoded_length,
                                   char *decoded, size_t decoded_capacity)
{
    size_t output_length = 0U;
    for (size_t i = 0U; i < encoded_length; ++i) {
        char value = encoded[i];
        if (value == '+') {
            value = ' ';
        } else if (value == '%') {
            if (i + 2U >= encoded_length) {
                return false;
            }
            const int high = wifi_hex_value(encoded[i + 1U]);
            const int low = wifi_hex_value(encoded[i + 2U]);
            if (high < 0 || low < 0) {
                return false;
            }
            value = (char)((high << 4) | low);
            i += 2U;
        }
        if (value == '\0' || output_length + 1U >= decoded_capacity) {
            return false;
        }
        decoded[output_length++] = value;
    }
    decoded[output_length] = '\0';
    return true;
}

static bool wifi_parse_form_value(const char *body, const char *key,
                                  char *value, size_t value_capacity)
{
    const size_t key_length = strlen(key);
    const char *cursor = body;
    while (*cursor != '\0') {
        const char *end = strchr(cursor, '&');
        const size_t pair_length = end == NULL
            ? strlen(cursor)
            : (size_t)(end - cursor);
        const char *separator = memchr(cursor, '=', pair_length);
        if (separator != NULL &&
            (size_t)(separator - cursor) == key_length &&
            memcmp(cursor, key, key_length) == 0) {
            const char *encoded_value = separator + 1;
            const size_t encoded_length =
                pair_length - (size_t)(encoded_value - cursor);
            return wifi_form_decode_value(encoded_value, encoded_length,
                                          value, value_capacity);
        }
        if (end == NULL) {
            break;
        }
        cursor = end + 1;
    }
    return false;
}

static esp_err_t wifi_provision_status_handler(httpd_req_t *request)
{
    char ip[sizeof(s_wifi_ip)] = {0};
    bool failed;
    bool pending;
    portENTER_CRITICAL(&s_dashboard_mux);
    memcpy(ip, s_wifi_ip, sizeof(ip));
    failed = s_wifi_candidate_failed;
    pending = s_wifi_candidate_pending;
    if (s_wifi_candidate_pending && ip[0] == '\0' &&
        (TickType_t)(xTaskGetTickCount() - s_wifi_candidate_started_tick) >=
            pdMS_TO_TICKS(20000U)) {
        s_wifi_candidate_failed = true;
        failed = true;
    }
    portEXIT_CRITICAL(&s_dashboard_mux);

    char response[192];
    int written = snprintf(response, sizeof(response),
                           "{\"connected\":%s,\"ip\":\"%s\",\"error\":%s}",
                           ip[0] != '\0' && !pending && !failed
                               ? "true" : "false",
                           ip,
                           failed ? "\"Wi-Fi connection or credential save failed. Check the password and try again.\""
                                  : "null");
    if (written < 0 || (size_t)written >= sizeof(response)) {
        ESP_LOGE(TAG, "Failed to build Wi-Fi provision status response");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to build status response");
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, written);
}

static esp_err_t wifi_provision_config_handler(httpd_req_t *request)
{
    if (!wifi_provisioning_is_active()) {
        return httpd_resp_send_err(request, HTTPD_403_FORBIDDEN,
                                   "Wi-Fi setup is not active");
    }
    if (request->content_len <= 0 || request->content_len > 256) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid form size");
    }

    char body[257];
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        int count = httpd_req_recv(request, body + received,
                                   (size_t)request->content_len - received);
        if (count <= 0) {
            ESP_LOGE(TAG, "Failed to receive Wi-Fi setup form");
            return httpd_resp_send_err(request,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "Failed to receive form");
        }
        received += (size_t)count;
    }
    body[received] = '\0';

    char ssid[sizeof(s_wifi_candidate_ssid)] = {0};
    char password[sizeof(s_wifi_candidate_password)] = {0};
    if (!wifi_parse_form_value(body, "ssid", ssid, sizeof(ssid)) ||
        !wifi_parse_form_value(body, "password", password,
                               sizeof(password)) ||
        ssid[0] == '\0' || strlen(ssid) > 32U || strlen(password) > 63U) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid SSID or password length");
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    memcpy(s_wifi_candidate_ssid, ssid, sizeof(s_wifi_candidate_ssid));
    memcpy(s_wifi_candidate_password, password,
           sizeof(s_wifi_candidate_password));
    s_wifi_candidate_pending = true;
    s_wifi_candidate_failed = false;
    s_wifi_candidate_started_tick = xTaskGetTickCount();
    s_wifi_ip[0] = '\0';
    portEXIT_CRITICAL(&s_dashboard_mux);

    wifi_config_t station_config = {0};
    memcpy(station_config.sta.ssid, ssid, strlen(ssid));
    memcpy(station_config.sta.password, password, strlen(password));
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &station_config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_disconnect();
        if (err == ESP_ERR_WIFI_NOT_CONNECT) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to selected Wi-Fi network: %s",
                 esp_err_to_name(err));
        portENTER_CRITICAL(&s_dashboard_mux);
        s_wifi_candidate_pending = false;
        s_wifi_candidate_failed = true;
        portEXIT_CRITICAL(&s_dashboard_mux);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Failed to start Wi-Fi connection");
    }

    static const char response[] = "{\"accepted\":true}";
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, sizeof(response) - 1U);
}

static bool wifi_parse_mac_address(const char *text, uint8_t address[6])
{
    if (text == NULL || strlen(text) != 17U) {
        return false;
    }

    for (size_t i = 0U; i < 6U; ++i) {
        const size_t offset = i * 3U;
        const int high = wifi_hex_value(text[offset]);
        const int low = wifi_hex_value(text[offset + 1U]);
        if (high < 0 || low < 0 ||
            (i < 5U && text[offset + 2U] != ':')) {
            return false;
        }
        address[5U - i] = (uint8_t)((high << 4) | low);
    }
    return true;
}

static void restart_after_http_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800U));
    esp_restart();
}

static esp_err_t wifi_settings_handler(httpd_req_t *request)
{
    wifi_ap_record_t ap_info = {0};
    const bool connected = esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
    char ssid[sizeof(s_wifi_ssid)];
    char active_ssid[sizeof(ap_info.ssid) + 1U] = {0};
    portENTER_CRITICAL(&s_dashboard_mux);
    memcpy(ssid, s_wifi_ssid, sizeof(ssid));
    portEXIT_CRITICAL(&s_dashboard_mux);
    if (connected) {
        memcpy(active_ssid, ap_info.ssid, sizeof(ap_info.ssid));
    }

    char response[256];
    size_t length = 0U;
    bool ok = append_json(response, sizeof(response), &length,
                          "{\"ssid\":");
    ok = ok && append_json_string(response, sizeof(response), &length,
                                  connected ? active_ssid : ssid);
    ok = ok && append_json(response, sizeof(response), &length,
                           ",\"connected\":%s,\"rssi_dbm\":",
                           connected ? "true" : "false");
    ok = ok && append_json(response, sizeof(response), &length,
                           connected ? "%d}" : "null}",
                           connected ? (int)ap_info.rssi : 0);
    if (!ok) {
        ESP_LOGE(TAG, "Failed to build Wi-Fi settings response");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to build Wi-Fi status");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, (ssize_t)length);
}

static esp_err_t wifi_reset_handler(httpd_req_t *request)
{
    esp_err_t err = clear_wifi_credentials();
    if (err != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not clear Wi-Fi credentials");
    }

    static const char response[] = "{\"cleared\":true,\"restarting\":true}";
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    err = httpd_resp_send(request, response, sizeof(response) - 1U);
    if (err == ESP_OK &&
        xTaskCreate(restart_after_http_task, "wifi_reset_restart", 2048U,
                    NULL, 5U, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not schedule restart after Wi-Fi reset response");
        esp_restart();
    }
    return err;
}

static esp_err_t ble_settings_handler(httpd_req_t *request)
{
    uint8_t address[6];
    uint8_t address_type = 0U;
    esp_err_t err = ble_transport_get_target_addr(address, &address_type);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read selected BLE address: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to read BLE selection");
    }

    int8_t rssi = 0;
    const bool rssi_valid = ble_transport_get_rssi(&rssi) == ESP_OK;
    char device_name[BLE_TRANSPORT_DEVICE_NAME_LENGTH];
    if (ble_transport_get_device_name(device_name, sizeof(device_name)) !=
        ESP_OK) {
        (void)snprintf(device_name, sizeof(device_name), "%s",
                       s_ble_device_name);
    }
    char response[384];
    size_t length = 0U;
    bool ok = append_json(response, sizeof(response), &length,
                          "{\"name\":");
    ok = ok && append_json_string(response, sizeof(response), &length,
                                  device_name);
    ok = ok && append_json(response, sizeof(response), &length,
                           ",\"address\":");
    if (ok && s_ble_selection_cleared) {
        ok = append_json(response, sizeof(response), &length, "null");
    } else if (ok) {
        ok = append_json(response, sizeof(response), &length,
                         "\"%02X:%02X:%02X:%02X:%02X:%02X\"",
                         address[5], address[4], address[3], address[2],
                         address[1], address[0]);
    }
    ok = ok && append_json(response, sizeof(response), &length,
                           ",\"address_type\":%u,\"saved\":%s,"
                           "\"cleared\":%s,\"connected\":%s,\"rssi_dbm\":",
                           (unsigned)address_type,
                           s_ble_selection_saved ? "true" : "false",
                           s_ble_selection_cleared ? "true" : "false",
                           ble_transport_is_ready() ? "true" : "false");
    ok = ok && append_json(response, sizeof(response), &length,
                           rssi_valid ? "%d}" : "null}",
                           (int)rssi);
    if (!ok) {
        ESP_LOGE(TAG, "Failed to build BLE settings response");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to build BLE status");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, (ssize_t)length);
}

static esp_err_t ble_scan_handler(httpd_req_t *request)
{
    if (strstr(request->uri, "refresh=1") != NULL) {
        esp_err_t err = ble_transport_scan_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start BLE scan: %s",
                     esp_err_to_name(err));
            return httpd_resp_send_err(request,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "Unable to start BLE scan");
        }
    }

    ble_transport_scan_result_t devices[BLE_TRANSPORT_SCAN_RESULT_COUNT];
    size_t count = 0U;
    bool scanning = false;
    esp_err_t err = ble_transport_scan_get_results(
        devices, BLE_TRANSPORT_SCAN_RESULT_COUNT, &count, &scanning);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BLE scan results: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to read BLE scan results");
    }

    static char response[6144];
    size_t length = 0U;
    bool ok = append_json(response, sizeof(response), &length,
                          "{\"scanning\":%s,\"devices\":[",
                          scanning ? "true" : "false");
    for (size_t i = 0U; ok && i < count; ++i) {
        char address[18];
        (void)snprintf(address, sizeof(address),
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       devices[i].address[5], devices[i].address[4],
                       devices[i].address[3], devices[i].address[2],
                       devices[i].address[1], devices[i].address[0]);
        const char *name = devices[i].name[0] != '\0'
            ? devices[i].name : "Unnamed device";
        ok = append_json(response, sizeof(response), &length,
                         "%s{\"name\":", i == 0U ? "" : ",");
        ok = ok && append_json_string(response, sizeof(response), &length,
                                      name);
        ok = ok && append_json(response, sizeof(response), &length,
                               ",\"address\":\"%s\",\"address_type\":%u,"
                               "\"rssi_dbm\":%d}",
                               address, (unsigned)devices[i].address_type,
                               (int)devices[i].rssi);
    }
    ok = ok && append_json(response, sizeof(response), &length, "]}");
    if (!ok) {
        ESP_LOGE(TAG, "BLE scan response buffer is too small");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "BLE scan response is too large");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, (ssize_t)length);
}

static esp_err_t ble_select_handler(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len > 256) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid BLE selection form");
    }

    char body[257];
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        int count = httpd_req_recv(request, body + received,
                                   (size_t)request->content_len - received);
        if (count <= 0) {
            ESP_LOGE(TAG, "Failed to receive BLE selection form");
            return httpd_resp_send_err(request,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "Failed to receive BLE selection");
        }
        received += (size_t)count;
    }
    body[received] = '\0';

    char address_text[18] = {0};
    char type_text[4] = {0};
    char name[BLE_TRANSPORT_DEVICE_NAME_LENGTH] = {0};
    uint8_t address[6];
    if (!wifi_parse_form_value(body, "address", address_text,
                               sizeof(address_text)) ||
        !wifi_parse_form_value(body, "address_type", type_text,
                               sizeof(type_text)) ||
        !wifi_parse_form_value(body, "name", name, sizeof(name)) ||
        !wifi_parse_mac_address(address_text, address) ||
        (strcmp(type_text, "0") != 0 && strcmp(type_text, "1") != 0 &&
         strcmp(type_text, "2") != 0 && strcmp(type_text, "3") != 0)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid BLE address or address type");
    }

    const uint8_t address_type = (uint8_t)(type_text[0] - '0');
    if (name[0] == '\0') {
        (void)snprintf(name, sizeof(name), "%s", address_text);
    }

    uint8_t previous_address[6] = {0};
    uint8_t previous_address_type = 0U;
    esp_err_t err = ble_transport_get_target_addr(previous_address,
                                                  &previous_address_type);
    const bool previous_target_valid = err == ESP_OK;
    const bool previous_selection_cleared = s_ble_selection_cleared;
    if (err == ESP_OK) {
        err = ble_transport_set_target_addr(address, address_type);
    }
    if (err == ESP_OK) {
        err = save_ble_selection(address, address_type, name);
    }
    if (err != ESP_OK) {
        if (previous_selection_cleared) {
            (void)ble_transport_reset_target_addr();
        } else if (previous_target_valid) {
            (void)ble_transport_set_target_addr(previous_address,
                                                previous_address_type);
        }
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not save BLE selection");
    }

    memcpy(s_ble_selected_address, address, sizeof(s_ble_selected_address));
    s_ble_selected_address_type = address_type;
    (void)snprintf(s_ble_device_name, sizeof(s_ble_device_name), "%s", name);
    s_ble_selection_saved = true;
    portENTER_CRITICAL(&s_dashboard_mux);
    s_ble_selection_cleared = false;
    memset(s_meter_value_valid, 0, sizeof(s_meter_value_valid));
    s_meter_last_update_tick = 0U;
    portEXIT_CRITICAL(&s_dashboard_mux);
    static const char response[] = "{\"saved\":true,\"connecting\":true}";
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, sizeof(response) - 1U);
}

static esp_err_t ble_delete_handler(httpd_req_t *request)
{
    esp_err_t err = delete_ble_selection();
    if (err != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not remove saved BLE selection");
    }

    memset(s_ble_selected_address, 0, sizeof(s_ble_selected_address));
    s_ble_selected_address_type = 0U;
    (void)snprintf(s_ble_device_name, sizeof(s_ble_device_name),
                   "Not selected");
    s_ble_selection_saved = false;
    portENTER_CRITICAL(&s_dashboard_mux);
    s_ble_selection_cleared = true;
    memset(s_meter_value_valid, 0, sizeof(s_meter_value_valid));
    s_meter_last_update_tick = 0U;
    portEXIT_CRITICAL(&s_dashboard_mux);
    err = ble_transport_reset_target_addr();
    if (err != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Selection deleted, but BLE target reset failed");
    }

    static const char response[] =
        "{\"deleted\":true,\"name\":\"Not selected\",\"address\":null}";
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, sizeof(response) - 1U);
}

static esp_err_t load_tft_rotation_setting(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("tft", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open TFT settings: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t enabled = 0U;
    err = nvs_get_u8(nvs, "auto_rotate", &enabled);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read TFT rotation setting: %s",
                 esp_err_to_name(err));
        return err;
    }
    if (enabled > 1U) {
        ESP_LOGE(TAG, "Stored TFT rotation setting is invalid: %u",
                 (unsigned)enabled);
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_tft_auto_rotate = enabled != 0U;
    portEXIT_CRITICAL(&s_dashboard_mux);
    return ESP_OK;
}

static esp_err_t save_tft_rotation_setting(bool enabled)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("tft", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open TFT settings for writing: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(nvs, "auto_rotate", enabled ? 1U : 0U);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save TFT rotation setting: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t tft_rotation_handler(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len > 32) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid rotation setting");
    }

    char body[33];
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        int count = httpd_req_recv(request, body + received,
                                   (size_t)request->content_len - received);
        if (count <= 0) {
            ESP_LOGE(TAG, "Failed to receive TFT rotation setting");
            return httpd_resp_send_err(request,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "Failed to receive setting");
        }
        received += (size_t)count;
    }
    body[received] = '\0';

    char value[2];
    if (!wifi_parse_form_value(body, "enabled", value, sizeof(value)) ||
        (strcmp(value, "0") != 0 && strcmp(value, "1") != 0)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Rotation setting must be 0 or 1");
    }

    const bool enabled = value[0] == '1';
    esp_err_t err = save_tft_rotation_setting(enabled);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not save TFT rotation setting");
    }

    portENTER_CRITICAL(&s_dashboard_mux);
    s_tft_auto_rotate = enabled;
    portEXIT_CRITICAL(&s_dashboard_mux);

    char response[40];
    int written = snprintf(response, sizeof(response),
                           "{\"tft_auto_rotate\":%s}",
                           enabled ? "true" : "false");
    if (written < 0 || (size_t)written >= sizeof(response)) {
        ESP_LOGE(TAG, "Failed to build TFT rotation response");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to build response");
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, written);
}

static esp_err_t dashboard_status_handler(httpd_req_t *request)
{
    double values[REGISTER_COUNT];
    bool valid[REGISTER_COUNT];
    char ip[sizeof(s_wifi_ip)];
    TickType_t updated_tick;
    TickType_t now_tick;
    bool ble_selection_cleared;
    bool tft_auto_rotate;
    wifi_ap_record_t ap_info = {0};
    int8_t ble_rssi = 0;
    bool wifi_rssi_valid = esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
    bool ble_rssi_valid = ble_transport_get_rssi(&ble_rssi) == ESP_OK;

    portENTER_CRITICAL(&s_dashboard_mux);
    memcpy(values, s_meter_values, sizeof(values));
    memcpy(valid, s_meter_value_valid, sizeof(valid));
    memcpy(ip, s_wifi_ip, sizeof(ip));
    updated_tick = s_meter_last_update_tick;
    now_tick = xTaskGetTickCount();
    ble_selection_cleared = s_ble_selection_cleared;
    tft_auto_rotate = s_tft_auto_rotate;
    portEXIT_CRITICAL(&s_dashboard_mux);
    if (ble_selection_cleared) {
        memset(valid, 0, sizeof(valid));
        updated_tick = 0U;
    }

    char response[1536];
    size_t length = 0U;
    bool ok = append_json(response, sizeof(response), &length,
                          "{\"ip\":\"%s\",\"tft_auto_rotate\":%s,"
                          "\"wifi_rssi_dbm\":",
                          ip, tft_auto_rotate ? "true" : "false");
    if (wifi_rssi_valid) {
        ok = ok && append_json(response, sizeof(response), &length, "%d",
                               ap_info.rssi);
    } else {
        ok = ok && append_json(response, sizeof(response), &length, "null");
    }
    ok = ok && append_json(response, sizeof(response), &length,
                           ",\"ble_connected\":%s,\"ble_rssi_dbm\":",
                           ble_rssi_valid ? "true" : "false");
    if (ble_rssi_valid) {
        ok = ok && append_json(response, sizeof(response), &length, "%d",
                               ble_rssi);
    } else {
        ok = ok && append_json(response, sizeof(response), &length, "null");
    }
    ok = ok && append_json(
        response, sizeof(response), &length,
        ",\"meter_age_ms\":%" PRIu64
        ",\"read_interval_seconds\":%u,\"meter\":[",
        updated_tick == 0U
            ? 0U
            : (uint64_t)(now_tick - updated_tick) * portTICK_PERIOD_MS,
        (unsigned)METER_READ_INTERVAL_SECONDS);

    for (size_t i = 0U; ok && i < REGISTER_COUNT; ++i) {
        ok = append_json(response, sizeof(response), &length,
                         "%s{\"name\":\"%s\",\"value\":",
                         i == 0U ? "" : ",", s_meter_names[i]);
        if (ok && valid[i]) {
            ok = append_json(response, sizeof(response), &length, "%.6f",
                             values[i]);
        } else if (ok) {
            ok = append_json(response, sizeof(response), &length, "null");
        }
        if (ok) {
            ok = append_json(response, sizeof(response), &length,
                             ",\"unit\":\"%s\"}", s_meter_units[i]);
        }
    }
    ok = ok && append_json(response, sizeof(response), &length, "]}");

    if (!ok) {
        ESP_LOGE(TAG, "Dashboard status JSON buffer is too small");
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to build status response");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, (ssize_t)length);
}

static esp_err_t start_dashboard_server(void)
{
    if (s_dashboard_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    esp_err_t err = httpd_start(&s_dashboard_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start dashboard HTTP server: %s",
                 esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = wifi_root_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t dashboard_uri = {
        .uri = "/dashboard",
        .method = HTTP_GET,
        .handler = dashboard_index_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t scan_uri = {
        .uri = "/api/wifi/scan",
        .method = HTTP_GET,
        .handler = wifi_scan_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t config_uri = {
        .uri = "/api/wifi/config",
        .method = HTTP_POST,
        .handler = wifi_provision_config_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t provision_status_uri = {
        .uri = "/api/wifi/status",
        .method = HTTP_GET,
        .handler = wifi_provision_status_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = dashboard_status_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t tft_rotation_uri = {
        .uri = "/api/settings/tft/rotation",
        .method = HTTP_POST,
        .handler = tft_rotation_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t wifi_settings_uri = {
        .uri = "/api/settings/wifi",
        .method = HTTP_GET,
        .handler = wifi_settings_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t wifi_reset_uri = {
        .uri = "/api/settings/wifi/reset",
        .method = HTTP_POST,
        .handler = wifi_reset_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t ble_settings_uri = {
        .uri = "/api/settings/ble",
        .method = HTTP_GET,
        .handler = ble_settings_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t ble_scan_uri = {
        .uri = "/api/settings/ble/scan",
        .method = HTTP_GET,
        .handler = ble_scan_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t ble_select_uri = {
        .uri = "/api/settings/ble/select",
        .method = HTTP_POST,
        .handler = ble_select_handler,
        .user_ctx = NULL
    };
    const httpd_uri_t ble_delete_uri = {
        .uri = "/api/settings/ble/delete",
        .method = HTTP_POST,
        .handler = ble_delete_handler,
        .user_ctx = NULL
    };

    err = httpd_register_uri_handler(s_dashboard_server, &root_uri);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &dashboard_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &status_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server,
                                         &tft_rotation_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &scan_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &config_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server,
                                         &provision_status_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server,
                                         &wifi_settings_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &wifi_reset_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &ble_settings_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &ble_scan_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &ble_select_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_dashboard_server, &ble_delete_uri);
    }
    static const char *const captive_paths[] = {
        "/generate_204", "/gen_204", "/hotspot-detect.html",
        "/connecttest.txt", "/ncsi.txt", "/fwlink"
    };
    for (size_t i = 0U; err == ESP_OK &&
         i < sizeof(captive_paths) / sizeof(captive_paths[0]); ++i) {
        const httpd_uri_t captive_uri = {
            .uri = captive_paths[i],
            .method = HTTP_GET,
            .handler = wifi_root_handler,
            .user_ctx = NULL
        };
        err = httpd_register_uri_handler(s_dashboard_server, &captive_uri);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register dashboard routes: %s",
                 esp_err_to_name(err));
        httpd_stop(s_dashboard_server);
        s_dashboard_server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}

static void stop_provisioning_after_success_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(15000U));

    bool connected;
    portENTER_CRITICAL(&s_dashboard_mux);
    connected = s_wifi_ip[0] != '\0' && !s_wifi_candidate_pending;
    portEXIT_CRITICAL(&s_dashboard_mux);

    if (connected) {
        esp_err_t err = stop_wifi_provisioning_ap();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi connected but setup AP remains active");
        }
    }
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_AP_STACONNECTED) {
        portENTER_CRITICAL(&s_dashboard_mux);
        if (s_wifi_ap_station_count < UINT8_MAX) {
            ++s_wifi_ap_station_count;
        }
        portEXIT_CRITICAL(&s_dashboard_mux);
        ESP_LOGI(TAG, "Client connected to provisioning AP");
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        portENTER_CRITICAL(&s_dashboard_mux);
        if (s_wifi_ap_station_count > 0U) {
            --s_wifi_ap_station_count;
        }
        portEXIT_CRITICAL(&s_dashboard_mux);
        ESP_LOGI(TAG, "Client disconnected from provisioning AP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_wifi_ssid[0] != '\0') {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start Wi-Fi connection: %s",
                         esp_err_to_name(err));
            }
        }
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        portENTER_CRITICAL(&s_dashboard_mux);
        s_wifi_ip[0] = '\0';
        s_wifi_setup_completed = false;
        const bool provisioning_timed_out =
            s_wifi_provisioning_timed_out;
        portEXIT_CRITICAL(&s_dashboard_mux);
        esp_err_t mqtt_err = meter_mqtt_stop();
        if (mqtt_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop MQTT after Wi-Fi disconnect: %s",
                     esp_err_to_name(mqtt_err));
        }
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        if (s_wifi_ssid[0] != '\0' || s_wifi_candidate_pending) {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Wi-Fi reconnect failed: %s",
                         esp_err_to_name(err));
            }
        }
        if (!wifi_provisioning_is_active() && !provisioning_timed_out) {
            esp_err_t ap_err = start_wifi_provisioning_ap();
            if (ap_err != ESP_OK) {
                ESP_LOGE(TAG, "Wi-Fi setup AP unavailable: %s",
                         esp_err_to_name(ap_err));
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        char ip[sizeof(s_wifi_ip)];
        int ip_length = snprintf(ip, sizeof(ip), IPSTR,
                                 IP2STR(&event->ip_info.ip));
        if (ip_length < 0 || (size_t)ip_length >= sizeof(ip)) {
            ESP_LOGE(TAG, "Failed to format assigned Wi-Fi IP address");
            return;
        }
        char ssid_to_save[sizeof(s_wifi_ssid)];
        char password_to_save[sizeof(s_wifi_password)];
        portENTER_CRITICAL(&s_dashboard_mux);
        memcpy(s_wifi_ip, ip, sizeof(s_wifi_ip));
        s_wifi_provisioning_timed_out = false;
        const bool candidate_pending = s_wifi_candidate_pending;
        memcpy(ssid_to_save, candidate_pending
                   ? s_wifi_candidate_ssid : s_wifi_ssid,
               sizeof(ssid_to_save));
        memcpy(password_to_save, candidate_pending
                   ? s_wifi_candidate_password : s_wifi_password,
               sizeof(password_to_save));
        portEXIT_CRITICAL(&s_dashboard_mux);
        ESP_LOGI(TAG, "Wi-Fi connected, IP address: %s", ip);
        esp_err_t mqtt_err = meter_mqtt_start();
        if (mqtt_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start MQTT client: %s",
                     esp_err_to_name(mqtt_err));
        }
        esp_err_t save_err = save_wifi_credentials(ssid_to_save,
                                                   password_to_save);
        if (save_err == ESP_OK) {
            portENTER_CRITICAL(&s_dashboard_mux);
            memcpy(s_wifi_ssid, ssid_to_save, sizeof(s_wifi_ssid));
            memcpy(s_wifi_password, password_to_save,
                   sizeof(s_wifi_password));
            s_wifi_candidate_pending = false;
            s_wifi_candidate_failed = false;
            s_wifi_setup_completed = true;
            portEXIT_CRITICAL(&s_dashboard_mux);
            if (wifi_provisioning_is_active()) {
                if (xTaskCreate(stop_provisioning_after_success_task,
                                "wifi_ap_stop", 2048U, NULL, 4U, NULL) !=
                    pdPASS) {
                    ESP_LOGE(TAG, "Could not schedule provisioning AP shutdown");
                    esp_err_t stop_err = stop_wifi_provisioning_ap();
                    if (stop_err != ESP_OK) {
                        ESP_LOGE(TAG, "Wi-Fi connected but setup AP remains active");
                    }
                }
            }
        } else {
            portENTER_CRITICAL(&s_dashboard_mux);
            s_wifi_candidate_failed = true;
            portEXIT_CRITICAL(&s_dashboard_mux);
            ESP_LOGE(TAG,
                     "Wi-Fi connected but credentials were not saved; setup AP stays active");
        }
        esp_err_t err = start_dashboard_server();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Dashboard unavailable: %s", esp_err_to_name(err));
        }
    }
}

static esp_err_t start_wifi(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Reinitializing NVS partition for Wi-Fi credential storage");
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = load_tft_rotation_setting();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Using default enabled TFT auto-rotation");
    }

    err = load_wifi_credentials();
    if (err == ESP_ERR_NOT_FOUND) {
        size_t ssid_length = strlen(CONFIG_SMART_METER_WIFI_SSID);
        size_t password_length = strlen(CONFIG_SMART_METER_WIFI_PASSWORD);
        if (ssid_length > 32U || password_length > 63U) {
            ESP_LOGE(TAG, "Configured Wi-Fi SSID or password is too long");
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(s_wifi_ssid, CONFIG_SMART_METER_WIFI_SSID, ssid_length + 1U);
        memcpy(s_wifi_password, CONFIG_SMART_METER_WIFI_PASSWORD,
               password_length + 1U);
    } else if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize network interface: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s",
                 esp_err_to_name(err));
        return err;
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi station interface");
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wi-Fi: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     &wifi_event_handler, NULL);
    if (err == ESP_OK) {
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         &wifi_event_handler, NULL);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Wi-Fi event handler: %s",
                 esp_err_to_name(err));
        return err;
    }

    wifi_config_t wifi_config = {0};
    if (s_wifi_ssid[0] != '\0') {
        memcpy(wifi_config.sta.ssid, s_wifi_ssid, strlen(s_wifi_ssid));
        memcpy(wifi_config.sta.password, s_wifi_password,
               strlen(s_wifi_password));
    }
    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    if (err == ESP_OK) {
        err = start_wifi_provisioning_ap();
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi station: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "Open setup AP '%s' is broadcasting; setup page: http://192.168.4.1/",
             WIFI_PROVISIONING_AP_SSID);
    err = start_dashboard_server();
    if (err != ESP_OK) {
        return err;
    }
    if (s_wifi_ssid[0] == '\0') {
        ESP_LOGI(TAG, "No saved Wi-Fi network; provisioning AP is ready");
    } else {
        ESP_LOGI(TAG, "Testing saved Wi-Fi network '%s'", s_wifi_ssid);
    }
    return ESP_OK;
}

/* TFT display ILI9488, 480x320 landscape, SPI interface */
#define TFT_WIDTH 480U
#define TFT_HEIGHT 320U
#define TFT_STATUS_HEIGHT 48U
#define TFT_GAUGE_WIDTH (TFT_WIDTH / 2U)
#define TFT_GAUGE_HEIGHT ((TFT_HEIGHT - TFT_STATUS_HEIGHT) / 2U)
#define TFT_PAGE_FOOTER_HEIGHT 32U
#define TFT_NAVIGATION_TOUCH_TOP (TFT_HEIGHT - TFT_PAGE_FOOTER_HEIGHT - 24U)
#define TFT_PAGE_CONTENT_HEIGHT ((TFT_HEIGHT - TFT_STATUS_HEIGHT - TFT_PAGE_FOOTER_HEIGHT) / 2U)
#define TFT_TRANSFER_CHUNK_ROWS 16U
#define TFT_CANVAS_BYTES (TFT_GAUGE_WIDTH * TFT_GAUGE_HEIGHT * sizeof(uint16_t))
#define TFT_PAGE_COUNT 5U
#define TFT_TOUCH_CAL_POINT_COUNT 4U
#define TFT_BACKLIGHT_IDLE_TIMEOUT_MS (5U * 60U * 1000U)
#define TFT_BACKLIGHT_FULL_DUTY 255U
#define TFT_BACKLIGHT_IDLE_DUTY 32U

#define TFT_COLOR_BACKGROUND 0xB7BFU
#define TFT_COLOR_AUX_BACKGROUND 0x0841U
#define TFT_COLOR_PANEL      0x10A2U
#define TFT_COLOR_TEXT       0xFFFFU
#define TFT_COLOR_MUTED      0x7BEFU
#define TFT_COLOR_DIGIT      0x0000U
#define TFT_COLOR_GREEN      0x07E0U
#define TFT_COLOR_YELLOW     0xFFE0U
#define TFT_COLOR_RED        0xF800U
#define TFT_COLOR_BLUE       0x05FFU

static esp_lcd_panel_io_handle_t s_tft_io;
static esp_lcd_panel_handle_t s_tft_panel;
static spi_device_handle_t s_touch_spi;
static SemaphoreHandle_t s_tft_transfer_done;
static uint16_t *s_tft_canvas;
typedef struct {
    double x_coefficients[3];
    double y_coefficients[3];
} tft_touch_calibration_t;
static tft_touch_calibration_t s_touch_calibration;
static const char s_tft_font_chars[] =
    " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.-:/";
static const uint8_t s_tft_font[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E},
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x00, 0x60, 0x60, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02}
};

static void tft_canvas_clear(uint16_t color, uint16_t width, uint16_t height)
{
    for (size_t i = 0U; i < (size_t)width * height; ++i) {
        s_tft_canvas[i] = color;
    }
}

static void tft_canvas_pixel(int x, int y, uint16_t color)
{
    if (x >= 0 && x < (int)TFT_GAUGE_WIDTH &&
        y >= 0 && y < (int)TFT_GAUGE_HEIGHT) {
        s_tft_canvas[(size_t)y * TFT_GAUGE_WIDTH + (size_t)x] = color;
    }
}

static void tft_canvas_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    for (;;) {
        tft_canvas_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static void tft_canvas_text(int x, int y, const char *text, uint8_t scale,
                            uint16_t color)
{
    for (const char *cursor = text; *cursor != '\0'; ++cursor) {
        const char *glyph_char = strchr(s_tft_font_chars, *cursor);
        size_t glyph_index = glyph_char == NULL
            ? 0U
            : (size_t)(glyph_char - s_tft_font_chars);

        for (int column = 0; column < 5; ++column) {
            for (int row = 0; row < 7; ++row) {
                if ((s_tft_font[glyph_index][column] & (1U << row)) == 0U) {
                    continue;
                }
                for (uint8_t sx = 0U; sx < scale; ++sx) {
                    for (uint8_t sy = 0U; sy < scale; ++sy) {
                        tft_canvas_pixel(x + column * scale + sx,
                                         y + row * scale + sy, color);
                    }
                }
            }
        }
        x += 6 * scale;
    }
}

static void tft_canvas_centered_text(const char *text, int y, uint8_t scale,
                                    uint16_t color)
{
    const int text_width = (int)strlen(text) * 6 * scale - scale;
    tft_canvas_text(((int)TFT_GAUGE_WIDTH - text_width) / 2, y, text,
                    scale, color);
}

static const tft_dseg7_glyph_t *tft_dseg7_find_glyph(char character)
{
    for (size_t i = 0U;
         i < sizeof(s_tft_dseg7_glyphs) / sizeof(s_tft_dseg7_glyphs[0]);
         ++i) {
        if (s_tft_dseg7_glyphs[i].character == character) {
            return &s_tft_dseg7_glyphs[i];
        }
    }
    return NULL;
}

static void tft_canvas_centered_digital_value(const char *text, int y,
                                              uint16_t color)
{
    int text_width = 0;
    for (const char *cursor = text; *cursor != '\0'; ++cursor) {
        const tft_dseg7_glyph_t *glyph = tft_dseg7_find_glyph(*cursor);
        if (glyph == NULL) {
            ESP_LOGW(TAG, "Unsupported character in TFT digital value");
            return;
        }
        text_width += glyph->advance;
    }

    int x = ((int)TFT_GAUGE_WIDTH - text_width) / 2;
    for (const char *cursor = text; *cursor != '\0'; ++cursor) {
        const tft_dseg7_glyph_t *glyph = tft_dseg7_find_glyph(*cursor);
        const size_t row_stride = (glyph->width + 7U) / 8U;
        for (uint8_t row = 0U; row < glyph->height; ++row) {
            for (uint8_t column = 0U; column < glyph->width; ++column) {
                const size_t byte_index = glyph->bitmap_offset +
                    (size_t)row * row_stride + column / 8U;
                const uint8_t mask = (uint8_t)(0x80U >> (column % 8U));
                if ((s_tft_dseg7_bitmap[byte_index] & mask) != 0U) {
                    tft_canvas_pixel(x + glyph->bearing_x + column,
                                     y + glyph->bearing_y + row, color);
                }
            }
        }
        x += glyph->advance;
    }
}

static esp_err_t tft_flush_tile(uint16_t x, uint16_t y,
                                uint16_t width, uint16_t height)
{
    static bool first_transfer_logged;
    for (uint16_t row = 0U; row < height;) {
        const uint16_t rows = (height - row) < TFT_TRANSFER_CHUNK_ROWS
            ? (uint16_t)(height - row)
            : TFT_TRANSFER_CHUNK_ROWS;
        const uint16_t *pixels =
            &s_tft_canvas[(size_t)row * width];
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            s_tft_panel, x, y + row, x + width, y + row + rows, pixels);
        if (err != ESP_OK) {
            return err;
        }
        xSemaphoreTake(s_tft_transfer_done, portMAX_DELAY);
        row = (uint16_t)(row + rows);
    }

    if (!first_transfer_logged) {
        ESP_LOGI(TAG,
                 "ILI9488 SPI color transfer completed: tile=(%u,%u) %ux%u",
                 (unsigned)x, (unsigned)y, (unsigned)width, (unsigned)height);
        first_transfer_logged = true;
    }
    return ESP_OK;
}

static esp_err_t tft_touch_read_command(uint8_t command, uint16_t *value)
{
    uint8_t tx_data[3] = {command, 0U, 0U};
    uint8_t rx_data[3] = {0U, 0U, 0U};
    spi_transaction_t transaction = {
        .length = sizeof(tx_data) * 8U,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };
    esp_err_t err = spi_device_polling_transmit(s_touch_spi, &transaction);
    if (err == ESP_OK) {
        *value = (uint16_t)((((uint16_t)rx_data[1] << 8) | rx_data[2]) >> 3)
            & 0x0FFFU;
    }
    return err;
}

static esp_err_t tft_touch_read(uint16_t *raw_x, uint16_t *raw_y,
                                bool *pressed)
{
    uint16_t z1 = 0U;
    uint16_t z2 = 0U;
    esp_err_t err = tft_touch_read_command(0xB0U, &z1);
    if (err == ESP_OK) {
        err = tft_touch_read_command(0xC0U, &z2);
    }
    if (err == ESP_OK) {
        *pressed = z1 > 100U && z2 < 4000U;
    }
    if (err == ESP_OK && *pressed) {
        err = tft_touch_read_command(0xD0U, raw_x);
    }
    if (err == ESP_OK && *pressed) {
        err = tft_touch_read_command(0x90U, raw_y);
    }
    return err;
}

static esp_err_t tft_touch_read_filtered(uint16_t *raw_x, uint16_t *raw_y,
                                         bool *pressed)
{
    uint16_t sample_x = 0U;
    uint16_t sample_y = 0U;
    esp_err_t err = tft_touch_read(&sample_x, &sample_y, pressed);
    if (err != ESP_OK || !*pressed) {
        return err;
    }

    uint32_t x_total = sample_x;
    uint32_t y_total = sample_y;
    size_t sample_count = 1U;
    for (size_t sample = 1U; sample < 5U; ++sample) {
        vTaskDelay(pdMS_TO_TICKS(5U));
        bool sample_pressed = false;
        err = tft_touch_read(&sample_x, &sample_y, &sample_pressed);
        if (err != ESP_OK) {
            return err;
        }
        if (!sample_pressed) {
            break;
        }
        x_total += sample_x;
        y_total += sample_y;
        ++sample_count;
    }

    *raw_x = (uint16_t)(x_total / sample_count);
    *raw_y = (uint16_t)(y_total / sample_count);
    *pressed = true;
    return ESP_OK;
}

static esp_err_t tft_wait_for_touch(uint16_t *raw_x, uint16_t *raw_y)
{
    bool pressed = false;
    esp_err_t err = ESP_OK;
    while (!pressed) {
        err = tft_touch_read(raw_x, raw_y, &pressed);
        if (err != ESP_OK) {
            return err;
        }
        if (!pressed) {
            vTaskDelay(pdMS_TO_TICKS(20U));
        }
    }

    uint32_t x_total = *raw_x;
    uint32_t y_total = *raw_y;
    size_t sample_count = 1U;
    for (size_t sample = 1U; sample < 8U; ++sample) {
        uint16_t sample_x = 0U;
        uint16_t sample_y = 0U;
        err = tft_touch_read(&sample_x, &sample_y, &pressed);
        if (err != ESP_OK) {
            return err;
        }
        if (!pressed) {
            break;
        }
        x_total += sample_x;
        y_total += sample_y;
        ++sample_count;
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    if (sample_count == 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    *raw_x = (uint16_t)(x_total / sample_count);
    *raw_y = (uint16_t)(y_total / sample_count);

    while (pressed) {
        vTaskDelay(pdMS_TO_TICKS(20U));
        uint16_t released_x = 0U;
        uint16_t released_y = 0U;
        err = tft_touch_read(&released_x, &released_y, &pressed);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static bool tft_color_transfer_done(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *event_data,
                                    void *user_context)
{
    (void)panel_io;
    (void)event_data;
    SemaphoreHandle_t transfer_done = (SemaphoreHandle_t)user_context;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(transfer_done, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static esp_err_t tft_render_touch_calibration_prompt(size_t active_point)
{
    static const char *const point_names[TFT_TOUCH_CAL_POINT_COUNT] = {
        "TOUCH TOP LEFT", "TOUCH TOP RIGHT",
        "TOUCH BOTTOM LEFT", "TOUCH BOTTOM RIGHT"
    };
    for (size_t point = 0U; point < TFT_TOUCH_CAL_POINT_COUNT; ++point) {
        const uint16_t x = (uint16_t)(point % 2U) * TFT_GAUGE_WIDTH;
        const uint16_t y = TFT_STATUS_HEIGHT +
            (uint16_t)(point / 2U) * TFT_PAGE_CONTENT_HEIGHT;
        const int target_x = (point % 2U == 0U) ? 20 : 219;
        const int target_y = (point / 2U == 0U) ? 20 : 99;

        tft_canvas_clear(TFT_COLOR_AUX_BACKGROUND, TFT_GAUGE_WIDTH,
                         TFT_PAGE_CONTENT_HEIGHT);
        tft_canvas_centered_text(point_names[active_point], 8, 1U,
                                 TFT_COLOR_TEXT);
        const uint16_t target_color = point == active_point
            ? TFT_COLOR_RED
            : TFT_COLOR_MUTED;
        tft_canvas_line(target_x - 12, target_y, target_x + 12, target_y,
                        target_color);
        tft_canvas_line(target_x, target_y - 12, target_x, target_y + 12,
                        target_color);
        esp_err_t err = tft_flush_tile(x, y, TFT_GAUGE_WIDTH,
                                       TFT_PAGE_CONTENT_HEIGHT);
        if (err != ESP_OK) {
            return err;
        }
    }

    for (uint16_t x = 0U; x < TFT_WIDTH; x += TFT_GAUGE_WIDTH) {
        tft_canvas_clear(TFT_COLOR_PANEL, TFT_GAUGE_WIDTH,
                         TFT_PAGE_FOOTER_HEIGHT);
        tft_canvas_centered_text("TOUCH RED TARGET", 8, 1U,
                                 TFT_COLOR_YELLOW);
        esp_err_t err = tft_flush_tile(x, TFT_HEIGHT - TFT_PAGE_FOOTER_HEIGHT,
                                       TFT_GAUGE_WIDTH,
                                       TFT_PAGE_FOOTER_HEIGHT);
        if (err != ESP_OK) {
            return err;
        }
    }

    for (uint16_t x = 0U; x < TFT_WIDTH; x += TFT_GAUGE_WIDTH) {
        tft_canvas_clear(TFT_COLOR_PANEL, TFT_GAUGE_WIDTH, TFT_STATUS_HEIGHT);
        tft_canvas_centered_text("TOUCH CALIBRATION", 16, 1U,
                                 TFT_COLOR_BLUE);
        esp_err_t err = tft_flush_tile(x, 0U, TFT_GAUGE_WIDTH,
                                       TFT_STATUS_HEIGHT);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t tft_load_touch_calibration(bool *completed)
{
    if (completed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *completed = false;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("tft", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open touch calibration storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    uint8_t state = 0U;
    err = nvs_get_u8(nvs, "touch_state", &state);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        ESP_LOGE(TAG, "Failed to read touch setup state: %s",
                 esp_err_to_name(err));
        return err;
    }

    if (state == 2U) {
        nvs_close(nvs);
        return ESP_OK;
    }
    if (state != 1U) {
        nvs_close(nvs);
        ESP_LOGE(TAG, "Touch setup state is invalid: %u", (unsigned)state);
        return ESP_ERR_INVALID_STATE;
    }

    size_t calibration_size = sizeof(s_touch_calibration);
    err = nvs_get_blob(nvs, "touch_cal", &s_touch_calibration,
                       &calibration_size);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load touch calibration: %s",
                 esp_err_to_name(err));
        return err;
    }
    if (calibration_size != sizeof(s_touch_calibration)) {
        ESP_LOGE(TAG, "Stored touch calibration has an invalid size");
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(s_touch_calibration.x_coefficients[i]) ||
            !isfinite(s_touch_calibration.y_coefficients[i])) {
            ESP_LOGE(TAG, "Stored touch calibration contains invalid values");
            return ESP_ERR_INVALID_STATE;
        }
    }
    *completed = true;
    return ESP_OK;
}

static esp_err_t tft_save_touch_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("tft", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open touch calibration storage: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs, "touch_cal", &s_touch_calibration,
                       sizeof(s_touch_calibration));
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "touch_state", 1U);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save touch setup state: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static bool tft_solve_touch_calibration(const uint16_t raw_x[],
                                        const uint16_t raw_y[],
                                        tft_touch_calibration_t *calibration)
{
    static const double screen_x[TFT_TOUCH_CAL_POINT_COUNT] = {
        20.0, 459.0, 20.0, 459.0
    };
    static const double screen_y[TFT_TOUCH_CAL_POINT_COUNT] = {
        68.0, 68.0, 267.0, 267.0
    };
    double matrix[3][5] = {{0.0}};

    for (size_t point = 0U; point < TFT_TOUCH_CAL_POINT_COUNT; ++point) {
        const double feature[3] = {
            raw_x[point] / 4095.0, raw_y[point] / 4095.0, 1.0
        };
        for (size_t row = 0U; row < 3U; ++row) {
            for (size_t column = 0U; column < 3U; ++column) {
                matrix[row][column] += feature[row] * feature[column];
            }
            matrix[row][3] += feature[row] * screen_x[point];
            matrix[row][4] += feature[row] * screen_y[point];
        }
    }

    for (size_t pivot = 0U; pivot < 3U; ++pivot) {
        size_t best_row = pivot;
        for (size_t row = pivot + 1U; row < 3U; ++row) {
            if (fabs(matrix[row][pivot]) > fabs(matrix[best_row][pivot])) {
                best_row = row;
            }
        }
        if (fabs(matrix[best_row][pivot]) < 1e-9) {
            return false;
        }
        if (best_row != pivot) {
            for (size_t column = pivot; column < 5U; ++column) {
                const double temporary = matrix[pivot][column];
                matrix[pivot][column] = matrix[best_row][column];
                matrix[best_row][column] = temporary;
            }
        }
        const double divisor = matrix[pivot][pivot];
        for (size_t column = pivot; column < 5U; ++column) {
            matrix[pivot][column] /= divisor;
        }
        for (size_t row = 0U; row < 3U; ++row) {
            if (row == pivot) {
                continue;
            }
            const double factor = matrix[row][pivot];
            for (size_t column = pivot; column < 5U; ++column) {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
        }
    }

    for (size_t i = 0U; i < 3U; ++i) {
        calibration->x_coefficients[i] = matrix[i][3];
        calibration->y_coefficients[i] = matrix[i][4];
    }
    return true;
}

static esp_err_t tft_calibrate_touch(void)
{
    uint16_t raw_x[TFT_TOUCH_CAL_POINT_COUNT];
    uint16_t raw_y[TFT_TOUCH_CAL_POINT_COUNT];
    for (size_t point = 0U; point < TFT_TOUCH_CAL_POINT_COUNT; ++point) {
        esp_err_t err = tft_render_touch_calibration_prompt(point);
        if (err != ESP_OK) {
            return err;
        }
        ESP_LOGI(TAG, "Touch the highlighted calibration target %u/4",
                 (unsigned)(point + 1U));
        err = tft_wait_for_touch(&raw_x[point], &raw_y[point]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Touch calibration read failed: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    if (!tft_solve_touch_calibration(raw_x, raw_y, &s_touch_calibration)) {
        ESP_LOGE(TAG, "Touch calibration points are invalid");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "XPT2046 touch calibration complete");
    return ESP_OK;
}

static bool tft_touch_get_screen_position(uint16_t raw_x, uint16_t raw_y,
                                          int *screen_x, int *screen_y)
{
    const double x = s_touch_calibration.x_coefficients[0] *
            (raw_x / 4095.0) +
        s_touch_calibration.x_coefficients[1] * (raw_y / 4095.0) +
        s_touch_calibration.x_coefficients[2];
    const double y = s_touch_calibration.y_coefficients[0] *
            (raw_x / 4095.0) +
        s_touch_calibration.y_coefficients[1] * (raw_y / 4095.0) +
        s_touch_calibration.y_coefficients[2];
    if (x < -20.0 || x > TFT_WIDTH + 20.0 ||
        y < -20.0 || y > TFT_HEIGHT + 20.0) {
        return false;
    }

    *screen_x = (int)lround(x);
    *screen_y = (int)lround(y);
    return true;
}

static void tft_render_status_card(bool wifi_connected, const char *wifi_ip,
                                   bool wifi_rssi_valid, int8_t wifi_rssi_dbm,
                                   bool ble_connected, uint16_t x)
{
    char line[24];
    const bool wifi_card = (x == 0U);
    tft_canvas_clear(TFT_COLOR_PANEL, TFT_GAUGE_WIDTH, TFT_STATUS_HEIGHT);
    if (wifi_card) {
        tft_canvas_text(8, 4, "WIFI", 1U, TFT_COLOR_BLUE);
        if (wifi_connected && wifi_rssi_valid) {
            (void)snprintf(line, sizeof(line), "SIGNAL %d DBM",
                           (int)wifi_rssi_dbm);
        } else {
            (void)snprintf(line, sizeof(line), "SIGNAL N/A");
        }
        tft_canvas_text(38, 4, line, 1U, TFT_COLOR_TEXT);
        if (wifi_connected) {
            (void)snprintf(line, sizeof(line), "%s", wifi_ip);
            tft_canvas_text(8, 20, line, 2U, TFT_COLOR_GREEN);
        } else {
            tft_canvas_text(8, 20, "DISCONNECTED", 2U, TFT_COLOR_RED);
        }
    } else {
        tft_canvas_text(8, 4, "BLUETOOTH", 1U, TFT_COLOR_BLUE);
        tft_canvas_text(8, 20, ble_connected ? "CONNECTED" : "SCANNING",
                        2U, ble_connected ? TFT_COLOR_GREEN : TFT_COLOR_YELLOW);
    }
    esp_err_t err = tft_flush_tile(x, 0U, TFT_GAUGE_WIDTH, TFT_STATUS_HEIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw TFT status card: %s",
                 esp_err_to_name(err));
    }
}

static esp_err_t tft_set_backlight_duty(uint32_t duty)
{
    if (CONFIG_SMART_METER_TFT_BL_GPIO < 0) {
        return ESP_OK;
    }

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TFT backlight brightness: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t tft_init(void)
{
    const int touch_cs_gpio = CONFIG_SMART_METER_TFT_TOUCH_CS_GPIO;
    const int touch_miso_gpio = CONFIG_SMART_METER_TFT_TOUCH_MISO_GPIO;
    const int pins[] = {
        CONFIG_SMART_METER_TFT_SCLK_GPIO,
        CONFIG_SMART_METER_TFT_MOSI_GPIO,
        CONFIG_SMART_METER_TFT_CS_GPIO,
        CONFIG_SMART_METER_TFT_DC_GPIO,
        CONFIG_SMART_METER_TFT_RST_GPIO
    };
    for (size_t i = 0U; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (pins[i] < 0 || pins[i] > GPIO_NUM_MAX) {
            ESP_LOGE(TAG, "TFT GPIO is outside the supported range");
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = i + 1U; j < sizeof(pins) / sizeof(pins[0]); ++j) {
            if (pins[i] == pins[j]) {
                ESP_LOGE(TAG, "TFT GPIO assignments must be unique");
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    if (touch_cs_gpio < 0 || touch_cs_gpio > GPIO_NUM_MAX ||
        touch_miso_gpio < 0 || touch_miso_gpio > GPIO_NUM_MAX ||
        touch_cs_gpio == touch_miso_gpio) {
        ESP_LOGE(TAG, "XPT2046 GPIO assignments are invalid");
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0U; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (touch_cs_gpio == pins[i] || touch_miso_gpio == pins[i]) {
            ESP_LOGE(TAG, "XPT2046 GPIO assignments must not overlap TFT GPIOs");
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (CONFIG_SMART_METER_TFT_BL_GPIO >= 0) {
        for (size_t i = 0U; i < sizeof(pins) / sizeof(pins[0]); ++i) {
            if (CONFIG_SMART_METER_TFT_BL_GPIO == pins[i]) {
                ESP_LOGE(TAG, "TFT backlight GPIO must be unique");
                return ESP_ERR_INVALID_ARG;
            }
        }
        if (CONFIG_SMART_METER_TFT_BL_GPIO == touch_cs_gpio ||
            CONFIG_SMART_METER_TFT_BL_GPIO == touch_miso_gpio) {
            ESP_LOGE(TAG, "TFT backlight GPIO must be unique");
            return ESP_ERR_INVALID_ARG;
        }
    }

    s_tft_canvas = heap_caps_malloc(TFT_CANVAS_BYTES,
                                   MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_tft_canvas == NULL) {
        ESP_LOGE(TAG, "Unable to allocate TFT drawing buffer");
        return ESP_ERR_NO_MEM;
    }
    s_tft_transfer_done = xSemaphoreCreateBinary();
    if (s_tft_transfer_done == NULL) {
        ESP_LOGE(TAG, "Unable to create TFT transfer semaphore");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_config = {
        .sclk_io_num = CONFIG_SMART_METER_TFT_SCLK_GPIO,
        .mosi_io_num = CONFIG_SMART_METER_TFT_MOSI_GPIO,
        .miso_io_num = touch_miso_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_CANVAS_BYTES
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TFT SPI bus: %s",
                 esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t touch_config = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = touch_cs_gpio,
        .queue_size = 1
    };
    err = spi_bus_add_device(SPI2_HOST, &touch_config, &s_touch_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add XPT2046 SPI device: %s",
                 esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = CONFIG_SMART_METER_TFT_DC_GPIO,
        .cs_gpio_num = CONFIG_SMART_METER_TFT_CS_GPIO,
        .pclk_hz = 5 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = tft_color_transfer_done,
        .user_ctx = s_tft_transfer_done
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                   &io_config, &s_tft_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TFT panel IO: %s",
                 esp_err_to_name(err));
        return err;
    }

    const esp_lcd_panel_dev_config_t panel_config = {
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 18,
        .reset_gpio_num = CONFIG_SMART_METER_TFT_RST_GPIO,
        .vendor_config = NULL
    };
    err = esp_lcd_new_panel_ili9488(
        s_tft_io, &panel_config,
        TFT_GAUGE_WIDTH * TFT_TRANSFER_CHUNK_ROWS, &s_tft_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ILI9488 panel: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_reset(s_tft_panel);
    if (err == ESP_OK) {
        err = esp_lcd_panel_init(s_tft_panel);
    }
    if (err == ESP_OK) {
        err = esp_lcd_panel_swap_xy(s_tft_panel, true);
    }
    if (err == ESP_OK) {
        err = esp_lcd_panel_mirror(s_tft_panel, true, false);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ILI9488 panel: %s",
                 esp_err_to_name(err));
        return err;
    }

    if (CONFIG_SMART_METER_TFT_BL_GPIO >= 0) {
        const ledc_timer_config_t backlight_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK
        };
        err = ledc_timer_config(&backlight_timer);
        const ledc_channel_config_t backlight_channel = {
            .gpio_num = CONFIG_SMART_METER_TFT_BL_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = TFT_BACKLIGHT_FULL_DUTY,
            .hpoint = 0
        };
        if (err == ESP_OK) {
            err = ledc_channel_config(&backlight_channel);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable TFT backlight: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG,
             "ILI9488 initialized for SPI RGB666, 480x320 landscape; XPT2046 on CS GPIO%d, MISO GPIO%d",
             touch_cs_gpio, touch_miso_gpio);
    return ESP_OK;
}

static esp_err_t tft_render_startup_test_pattern(void)
{
    static const uint16_t colors[] = {
        TFT_COLOR_RED, TFT_COLOR_GREEN, 0x001FU,
        TFT_COLOR_YELLOW, TFT_COLOR_TEXT, 0x07FFU
    };
    const uint16_t band_height = TFT_HEIGHT / 3U;

    for (size_t i = 0U; i < sizeof(colors) / sizeof(colors[0]); ++i) {
        const uint16_t y = (uint16_t)(i / 2U) * band_height;
        const uint16_t height = (i / 2U == 2U)
            ? (uint16_t)(TFT_HEIGHT - y)
            : band_height;
        const uint16_t x = (uint16_t)(i % 2U) * TFT_GAUGE_WIDTH;
        tft_canvas_clear(colors[i], TFT_GAUGE_WIDTH, height);
        esp_err_t err = tft_flush_tile(x, y, TFT_GAUGE_WIDTH, height);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw ILI9488 startup test pattern: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "ILI9488 startup color test displayed for 1 second");
    vTaskDelay(pdMS_TO_TICKS(1000U));
    return ESP_OK;
}

static void tft_render_metric_card(int tile_x, uint16_t y,
                                   const char *title, double value,
                                   bool valid, double demo_value,
                                   const char *unit, uint8_t decimals)
{
    char value_text[24];
    const double display_value = valid ? value : demo_value;
    if (decimals == 0U) {
        (void)snprintf(value_text, sizeof(value_text), "%.0f", display_value);
    } else if (decimals == 1U) {
        (void)snprintf(value_text, sizeof(value_text), "%.1f", display_value);
    } else {
        (void)snprintf(value_text, sizeof(value_text), "%.2f", display_value);
    }

    tft_canvas_clear(TFT_COLOR_BACKGROUND, TFT_GAUGE_WIDTH,
                     TFT_PAGE_CONTENT_HEIGHT);
    tft_canvas_centered_text(title, 8, 1U, TFT_COLOR_GREEN);
    tft_canvas_centered_digital_value(value_text, 32, TFT_COLOR_DIGIT);
    tft_canvas_centered_text(unit, 78, 2U, TFT_COLOR_GREEN);
    if (!valid) {
        tft_canvas_centered_text("DEMO", 104, 1U, TFT_COLOR_YELLOW);
    }

    esp_err_t err = tft_flush_tile((uint16_t)tile_x, y, TFT_GAUGE_WIDTH,
                                   TFT_PAGE_CONTENT_HEIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw TFT metric card %s: %s", title,
                 esp_err_to_name(err));
    }
}

static void tft_render_text_card(int tile_x, uint16_t y, const char *title,
                                 const char *value, const char *unit,
                                 bool demo)
{
    tft_canvas_clear(TFT_COLOR_BACKGROUND, TFT_GAUGE_WIDTH,
                     TFT_PAGE_CONTENT_HEIGHT);
    tft_canvas_centered_text(title, 8, 1U, TFT_COLOR_GREEN);
    bool numeric_value = value[0] != '\0';
    size_t decimal_point_count = 0U;
    for (const char *cursor = value; *cursor != '\0'; ++cursor) {
        if (*cursor == '.') {
            ++decimal_point_count;
        } else if ((*cursor < '0' || *cursor > '9') && *cursor != '-') {
            numeric_value = false;
            break;
        }
    }
    if (decimal_point_count > 1U) {
        numeric_value = false;
    }
    if (numeric_value) {
        tft_canvas_centered_digital_value(value, 32, TFT_COLOR_DIGIT);
    } else {
        tft_canvas_centered_text(value, 38, 2U, TFT_COLOR_DIGIT);
    }
    tft_canvas_centered_text(unit, 78, 1U, TFT_COLOR_GREEN);
    if (demo) {
        tft_canvas_centered_text("DEMO", 104, 1U, TFT_COLOR_YELLOW);
    }

    esp_err_t err = tft_flush_tile((uint16_t)tile_x, y, TFT_GAUGE_WIDTH,
                                   TFT_PAGE_CONTENT_HEIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw TFT status card %s: %s", title,
                 esp_err_to_name(err));
    }
}

static void tft_render_wifi_setup_screen(const char *setup_url)
{
    for (uint16_t row = 0U; row < 2U; ++row) {
        for (uint16_t column = 0U; column < 2U; ++column) {
            const uint16_t tile_x = column * TFT_GAUGE_WIDTH;
            const uint16_t tile_y = row * TFT_GAUGE_HEIGHT;
            tft_canvas_clear(TFT_COLOR_AUX_BACKGROUND, TFT_GAUGE_WIDTH,
                             TFT_GAUGE_HEIGHT);

            if (row == 0U && column == 0U) {
                tft_canvas_centered_text("SMART METER", 38, 2U,
                                         TFT_COLOR_BLUE);
                tft_canvas_centered_text("WIFI SETUP", 72, 2U,
                                         TFT_COLOR_GREEN);
            } else if (row == 0U) {
                tft_canvas_centered_text("SETUP IN PROGRESS", 56, 1U,
                                         TFT_COLOR_YELLOW);
            } else if (column == 0U) {
                tft_canvas_centered_text("CONNECT TO AP", 42, 1U,
                                         TFT_COLOR_TEXT);
                tft_canvas_centered_text("SMART METER", 66, 1U,
                                         TFT_COLOR_BLUE);
            } else {
                tft_canvas_centered_text("OPEN THIS ADDRESS", 34, 1U,
                                         TFT_COLOR_TEXT);
                tft_canvas_centered_text(setup_url, 62, 2U,
                                         TFT_COLOR_GREEN);
            }

            esp_err_t err = tft_flush_tile(tile_x, tile_y, TFT_GAUGE_WIDTH,
                                           TFT_GAUGE_HEIGHT);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to draw Wi-Fi setup screen: %s",
                         esp_err_to_name(err));
                return;
            }
        }
    }
}

static void tft_render_wifi_reset_card(uint32_t hold_ms, bool failed,
                                       bool resetting)
{
    const uint16_t tile_y =
        TFT_STATUS_HEIGHT + TFT_PAGE_CONTENT_HEIGHT;
    const uint16_t tile_x = TFT_GAUGE_WIDTH;
    const uint32_t bounded_hold_ms = hold_ms > 5000U ? 5000U : hold_ms;
    char label[24];

    tft_canvas_clear(TFT_COLOR_PANEL, TFT_GAUGE_WIDTH,
                     TFT_PAGE_CONTENT_HEIGHT);
    tft_canvas_centered_text("RESET WIFI", 12, 1U, TFT_COLOR_BLUE);
    if (failed) {
        tft_canvas_centered_text("RESET FAILED", 42, 1U, TFT_COLOR_RED);
        tft_canvas_centered_text("CHECK LOG", 62, 1U, TFT_COLOR_MUTED);
    } else if (resetting) {
        tft_canvas_centered_text("WIFI CLEARED", 42, 1U, TFT_COLOR_GREEN);
        tft_canvas_centered_text("RESTARTING", 62, 1U, TFT_COLOR_MUTED);
    } else {
        if (bounded_hold_ms == 0U) {
            (void)snprintf(label, sizeof(label), "HOLD 5 SEC");
        } else {
            (void)snprintf(label, sizeof(label), "HOLD %u SEC",
                           (unsigned)((bounded_hold_ms + 999U) / 1000U));
        }
        tft_canvas_centered_text(label, 42, 1U, TFT_COLOR_TEXT);
    }

    const int bar_left = 20;
    const int bar_right = (int)TFT_GAUGE_WIDTH - 21;
    const int bar_top = 87;
    const int bar_bottom = 98;
    tft_canvas_line(bar_left, bar_top, bar_right, bar_top, TFT_COLOR_MUTED);
    tft_canvas_line(bar_left, bar_bottom, bar_right, bar_bottom, TFT_COLOR_MUTED);
    tft_canvas_line(bar_left, bar_top, bar_left, bar_bottom, TFT_COLOR_MUTED);
    tft_canvas_line(bar_right, bar_top, bar_right, bar_bottom, TFT_COLOR_MUTED);

    const int fill_width =
        (int)((uint32_t)(bar_right - bar_left - 2) * bounded_hold_ms / 5000U);
    for (int x = bar_left + 1; x <= bar_left + fill_width; ++x) {
        tft_canvas_line(x, bar_top + 1, x, bar_bottom - 1,
                        failed ? TFT_COLOR_RED : TFT_COLOR_GREEN);
    }

    esp_err_t err = tft_flush_tile(tile_x, tile_y, TFT_GAUGE_WIDTH,
                                   TFT_PAGE_CONTENT_HEIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw Wi-Fi reset button: %s",
                 esp_err_to_name(err));
    }
}

static void tft_render_page_footer(size_t page)
{
    char label[16];
    for (uint16_t x = 0U; x < TFT_WIDTH; x += TFT_GAUGE_WIDTH) {
        tft_canvas_clear(TFT_COLOR_PANEL, TFT_GAUGE_WIDTH,
                         TFT_PAGE_FOOTER_HEIGHT);
        if (x == 0U) {
            (void)snprintf(label, sizeof(label), "BACK %u/5",
                           (unsigned)(page + 1U));
        } else {
            (void)snprintf(label, sizeof(label), "NEXT %u/5",
                           (unsigned)(page + 1U));
        }
        tft_canvas_centered_text(label, 8, 1U, TFT_COLOR_YELLOW);
        esp_err_t err = tft_flush_tile(x, TFT_HEIGHT - TFT_PAGE_FOOTER_HEIGHT,
                                       TFT_GAUGE_WIDTH,
                                       TFT_PAGE_FOOTER_HEIGHT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw TFT page navigation: %s",
                     esp_err_to_name(err));
        }
    }
}

static void tft_render_meter_page(size_t page,
                                  const double values[REGISTER_COUNT],
                                  const bool valid[REGISTER_COUNT],
                                  bool wifi_connected, const char *wifi_ip,
                                  bool wifi_rssi_valid, int8_t wifi_rssi_dbm,
                                  bool ble_connected)
{
    static const char *const page_titles[TFT_PAGE_COUNT] = {
        "OVERVIEW", "ENERGY/DEMAND", "POWER", "ELECTRICAL", "NETWORK"
    };
    static const struct {
        size_t index;
        const char *title;
        const char *unit;
        double demo_value;
        uint8_t decimals;
    } page_metrics[4][4] = {
        {
            {0U, "IMPORT ENERGY", "KWH", 125.4, 1U},
            {1U, "ACTIVE POWER", "KW", 2.45, 2U},
            {2U, "VOLTAGE L1", "V", 231.0, 0U},
            {3U, "CURRENT L1", "A", 10.6, 1U}
        },
        {
            {6U, "EXPORT ENERGY", "KWH", 3.2, 1U},
            {10U, "IMPORT REACTIVE", "KVARH", 1.7, 1U},
            {11U, "EXPORT REACTIVE", "KVARH", 0.2, 1U},
            {5U, "POWER FACTOR", "", 0.98, 2U}
        },
        {
            {1U, "ACTIVE POWER", "KW", 2.45, 2U},
            {9U, "EXPORT ACTIVE", "KW", 0.25, 2U},
            {7U, "REACTIVE POWER", "KVAR", 0.42, 2U},
            {8U, "APPARENT POWER", "KVA", 2.51, 2U}
        },
        {
            {2U, "VOLTAGE L1", "V", 231.0, 0U},
            {3U, "CURRENT L1", "A", 10.6, 1U},
            {4U, "FREQUENCY", "HZ", 50.0, 1U},
            {5U, "POWER FACTOR", "", 0.98, 2U}
        }
    };
    char label[24];
    wifi_ap_record_t ap_info = {0};
    const bool current_wifi_rssi_valid =
        esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;

    tft_render_status_card(wifi_connected, wifi_ip, wifi_rssi_valid,
                           wifi_rssi_dbm, ble_connected, 0U);
    tft_render_status_card(wifi_connected, wifi_ip, wifi_rssi_valid,
                           wifi_rssi_dbm, ble_connected, TFT_GAUGE_WIDTH);

    if (page < 4U) {
        for (size_t card = 0U; card < 4U; ++card) {
            const size_t column = card % 2U;
            const size_t row = card / 2U;
            const uint16_t tile_x = (uint16_t)column * TFT_GAUGE_WIDTH;
            const uint16_t tile_y = TFT_STATUS_HEIGHT +
                (uint16_t)row * TFT_PAGE_CONTENT_HEIGHT;
            const size_t index = page_metrics[page][card].index;
            tft_render_metric_card(
                tile_x, tile_y, page_metrics[page][card].title,
                values[index], valid[index],
                page_metrics[page][card].demo_value,
                page_metrics[page][card].unit,
                page_metrics[page][card].decimals);
        }
    } else {
        if (wifi_connected && current_wifi_rssi_valid) {
            (void)snprintf(label, sizeof(label), "%d",
                           (int)ap_info.rssi);
        } else {
            (void)snprintf(label, sizeof(label), "-57");
        }
        tft_render_text_card(0U, TFT_STATUS_HEIGHT, "WIFI SIGNAL", label,
                             "DBM", !(wifi_connected &&
                                      current_wifi_rssi_valid));
        tft_render_text_card(TFT_GAUGE_WIDTH, TFT_STATUS_HEIGHT, "IP ADDRESS",
                             wifi_connected ? wifi_ip : "DISCONNECTED", "",
                             false);
        tft_render_text_card(0U,
                             TFT_STATUS_HEIGHT + TFT_PAGE_CONTENT_HEIGHT,
                             "BLE LINK",
                             ble_connected ? "CONNECTED" : "SCANNING", "",
                             false);
        tft_render_wifi_reset_card(0U, false, false);
    }

    tft_render_page_footer(page);
    ESP_LOGD(TAG, "Rendered TFT page %u: %s", (unsigned)(page + 1U),
             page_titles[page]);
}

static void tft_wait_for_release(void)
{
    bool pressed = true;
    while (pressed) {
        uint16_t raw_x = 0U;
        uint16_t raw_y = 0U;
        esp_err_t err = tft_touch_read(&raw_x, &raw_y, &pressed);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "XPT2046 read failed while waiting for release: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100U));
            return;
        }
        if (pressed) {
            vTaskDelay(pdMS_TO_TICKS(20U));
        }
    }
}

static void tft_display_task(void *arg)
{
    (void)arg;
    bool calibration_completed = false;
    esp_err_t calibration_err =
        tft_load_touch_calibration(&calibration_completed);
    if (calibration_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not load saved touch calibration: %s",
                 esp_err_to_name(calibration_err));
        calibration_completed = false;
    }

    calibration_err = tft_calibrate_touch();
    if (calibration_err == ESP_OK) {
        calibration_completed = true;
        esp_err_t save_err = tft_save_touch_calibration();
        if (save_err == ESP_OK) {
            ESP_LOGI(TAG, "Touch calibration saved");
        } else {
            ESP_LOGE(TAG,
                     "Touch calibration is active but was not saved: %s",
                     esp_err_to_name(save_err));
        }
    } else if (calibration_completed) {
        ESP_LOGW(TAG,
                 "Touch calibration failed; using the previous saved calibration");
    } else {
        ESP_LOGE(TAG, "Touch calibration did not complete: %s",
                 esp_err_to_name(calibration_err));
    }

    const bool touch_enabled =
        calibration_completed;
    if (!touch_enabled) {
        ESP_LOGW(TAG, "TFT touch navigation is disabled");
    }
    TickType_t last_touch_activity = xTaskGetTickCount();
    TickType_t last_page_change_tick = xTaskGetTickCount();
    bool backlight_dimmed = false;
    bool previous_auto_rotate = true;
    size_t page = 0U;
    for (;;) {
        double values[REGISTER_COUNT];
        bool valid[REGISTER_COUNT];
        char ip[sizeof(s_wifi_ip)];
        bool show_wifi_setup;

        portENTER_CRITICAL(&s_dashboard_mux);
        memcpy(values, s_meter_values, sizeof(values));
        memcpy(valid, s_meter_value_valid, sizeof(valid));
        memcpy(ip, s_wifi_ip, sizeof(ip));
        show_wifi_setup = s_provisioning_active &&
            s_wifi_ap_station_count > 0U && !s_wifi_setup_completed;
        portEXIT_CRITICAL(&s_dashboard_mux);

        if (show_wifi_setup) {
            last_page_change_tick = xTaskGetTickCount();
            esp_netif_ip_info_t ap_ip_info = {0};
            char setup_url[32] = "AP IP UNAVAILABLE";
            if (s_wifi_ap_netif != NULL &&
                esp_netif_get_ip_info(s_wifi_ap_netif, &ap_ip_info) == ESP_OK) {
                int url_length = snprintf(setup_url, sizeof(setup_url),
                                          "HTTP://" IPSTR,
                                          IP2STR(&ap_ip_info.ip));
                if (url_length < 0 ||
                    (size_t)url_length >= sizeof(setup_url)) {
                    (void)snprintf(setup_url, sizeof(setup_url),
                                   "AP IP UNAVAILABLE");
                }
            }
            if (backlight_dimmed) {
                esp_err_t err = tft_set_backlight_duty(TFT_BACKLIGHT_FULL_DUTY);
                if (err == ESP_OK) {
                    backlight_dimmed = false;
                }
            }
            tft_render_wifi_setup_screen(setup_url);
            vTaskDelay(pdMS_TO_TICKS(500U));
            continue;
        }

        bool auto_rotate;
        portENTER_CRITICAL(&s_dashboard_mux);
        auto_rotate = s_tft_auto_rotate;
        portEXIT_CRITICAL(&s_dashboard_mux);
        const TickType_t now_tick = xTaskGetTickCount();
        if (auto_rotate != previous_auto_rotate) {
            previous_auto_rotate = auto_rotate;
            last_page_change_tick = now_tick;
            ESP_LOGI(TAG, "TFT auto-rotation %s",
                     auto_rotate ? "enabled" : "disabled");
        } else if (auto_rotate &&
                   now_tick - last_page_change_tick >=
                       pdMS_TO_TICKS(10000U)) {
            page = (page + 1U) % TFT_PAGE_COUNT;
            last_page_change_tick = now_tick;
            ESP_LOGI(TAG, "TFT auto-advanced to page %u",
                     (unsigned)(page + 1U));
        }

        const bool wifi_connected = ip[0] != '\0';
        wifi_ap_record_t ap_info = {0};
        const bool wifi_rssi_valid =
            esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
        const bool ble_connected = ble_transport_is_ready();
        tft_render_meter_page(page, values, valid, wifi_connected, ip,
                              wifi_rssi_valid, ap_info.rssi, ble_connected);

        if (touch_enabled || CONFIG_SMART_METER_TFT_BL_GPIO >= 0) {
            uint16_t raw_x = 0U;
            uint16_t raw_y = 0U;
            bool pressed = false;
            esp_err_t err =
                tft_touch_read_filtered(&raw_x, &raw_y, &pressed);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "XPT2046 touch read failed: %s",
                         esp_err_to_name(err));
            } else if (pressed) {
                last_touch_activity = xTaskGetTickCount();
                if (backlight_dimmed) {
                    err = tft_set_backlight_duty(TFT_BACKLIGHT_FULL_DUTY);
                    if (err == ESP_OK) {
                        backlight_dimmed = false;
                        ESP_LOGI(TAG, "TFT backlight restored");
                    }
                }
                if (!touch_enabled) {
                    tft_wait_for_release();
                    vTaskDelay(pdMS_TO_TICKS(150U));
                    continue;
                }

                int screen_x = 0;
                int screen_y = 0;
                const bool position_valid =
                    tft_touch_get_screen_position(raw_x, raw_y, &screen_x,
                                                  &screen_y);
                ESP_LOGI(TAG,
                         "TFT touch raw=(%u,%u), screen=(%d,%d), valid=%s",
                         (unsigned)raw_x, (unsigned)raw_y, screen_x, screen_y,
                         position_valid ? "yes" : "no");
                const bool reset_button_pressed =
                    page == 4U && position_valid &&
                    screen_x >= (int)TFT_GAUGE_WIDTH &&
                    screen_x < (int)TFT_WIDTH &&
                    screen_y >=
                        (int)(TFT_STATUS_HEIGHT + TFT_PAGE_CONTENT_HEIGHT) &&
                    screen_y < (int)TFT_NAVIGATION_TOUCH_TOP;
                if (reset_button_pressed) {
                    const TickType_t hold_started = xTaskGetTickCount();
                    bool reset_completed = false;
                    bool reset_failed = false;
                    while (!reset_completed && !reset_failed) {
                        const TickType_t elapsed =
                            xTaskGetTickCount() - hold_started;
                        if (elapsed >= pdMS_TO_TICKS(5000U)) {
                            tft_render_wifi_reset_card(5000U, false, true);
                            err = clear_wifi_credentials();
                            if (err == ESP_OK) {
                                reset_completed = true;
                                vTaskDelay(pdMS_TO_TICKS(800U));
                                esp_restart();
                            } else {
                                tft_render_wifi_reset_card(5000U, true, false);
                                reset_failed = true;
                            }
                            break;
                        }

                        tft_render_wifi_reset_card(
                            (uint32_t)elapsed * portTICK_PERIOD_MS,
                            false, false);
                        vTaskDelay(pdMS_TO_TICKS(100U));
                        err = tft_touch_read_filtered(&raw_x, &raw_y,
                                                      &pressed);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG,
                                     "XPT2046 read failed during Wi-Fi reset: %s",
                                     esp_err_to_name(err));
                            break;
                        }
                        if (!pressed) {
                            break;
                        }
                    }
                    if (!reset_completed && !reset_failed) {
                        tft_render_wifi_reset_card(0U, false, false);
                    }
                    if (reset_failed) {
                        tft_wait_for_release();
                    }
                } else {
                    if (position_valid &&
                        screen_y >= (int)TFT_NAVIGATION_TOUCH_TOP) {
                        if (screen_x < (int)TFT_WIDTH / 2) {
                            page = (page + TFT_PAGE_COUNT - 1U) %
                                TFT_PAGE_COUNT;
                            ESP_LOGI(TAG, "TFT previous page: %u",
                                     (unsigned)(page + 1U));
                        } else {
                            page = (page + 1U) % TFT_PAGE_COUNT;
                            ESP_LOGI(TAG, "TFT next page: %u",
                                     (unsigned)(page + 1U));
                        }
                        last_page_change_tick = xTaskGetTickCount();
                    }
                    tft_wait_for_release();
                }
                vTaskDelay(pdMS_TO_TICKS(150U));
            } else {
                if (CONFIG_SMART_METER_TFT_BL_GPIO >= 0 &&
                    !backlight_dimmed &&
                    xTaskGetTickCount() - last_touch_activity >=
                        pdMS_TO_TICKS(TFT_BACKLIGHT_IDLE_TIMEOUT_MS)) {
                    err = tft_set_backlight_duty(TFT_BACKLIGHT_IDLE_DUTY);
                    if (err == ESP_OK) {
                        backlight_dimmed = true;
                        ESP_LOGI(TAG, "TFT backlight dimmed after 5 minutes");
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(40U));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(500U));
        }
    }
}

// HDLC SNRM Frame (Client Address: 0x21, Server Address: 0x01)
static const uint8_t dlms_snrm[]  = {
    0x7E,                   // Flag Start
    0xA0, 0x07,             // Frame Format & Length (7 bytes)
    0x03,                   // Server Address
    0x21,                   // Client Address
    0x93,                   // SNRM Request Command
    0x0F, 0x01,             // HCS Checksum
    0x7E                    // Flag End
};

static volatile uint8_t dlms_state;

typedef enum {
    METER_CYCLE_WAITING,
    METER_CYCLE_CONNECTING,
    METER_CYCLE_READING,
    METER_CYCLE_DISCONNECTING,
} meter_cycle_phase_t;

static volatile meter_cycle_phase_t s_meter_cycle_phase =
    METER_CYCLE_WAITING;
static TickType_t s_cycle_connect_deadline;
static TickType_t s_release_response_deadline;
static TickType_t s_disconnect_retry_tick;
static volatile bool s_release_response_waiting;
static bool s_ble_stop_requested;

#define METER_CYCLE_CONNECT_TIMEOUT_MS 30000U
#define DLMS_RELEASE_RESPONSE_TIMEOUT_MS 2000U
#define BLE_DISCONNECT_RETRY_MS 1000U

// typedef struct {
//     const char *name;
//     uint8_t obis[6];
//     uint8_t expected_unit;
// } meter_register_t;


/*
 * DLMS unit:
 * 0x1E = Wh, 0x1B = W, 0x23 = V, 0x21 = A
 * ตรวจ unit ที่มิเตอร์ตอบอีกครั้งก่อนแสดงผล
 */
// static const meter_register_t s_registers[] = {
//     { "Import energy",  {1, 0,  1, 8, 0, 255}, 0x1E }, /* Wh */
//     { "Active power",   {1, 0,  1, 7, 0, 255}, 0x1B }, /* W */
//     { "Voltage L1",    {1, 0, 32, 7, 0, 255}, 0x23 }, /* V */
//     { "Current L1",    {1, 0, 31, 7, 0, 255}, 0x21 }, /* A */
//     { "Frequency",     {1, 0, 14, 7, 0, 255}, 0x2C }, /* Hz */
//     { "Power factor",  {1, 0, 13, 7, 0, 255}, 0xFF }, /* ตรวจจากมิเตอร์แล้ว */

//     { "Export energy", {1, 0,  2, 8, 0, 255}, 0x1E }, /* Wh */
//     { "Reactive power",{1, 0,  3, 7, 0, 255}, 0x1D }, /* var */
//     { "Apparent power",{1, 0,  9, 7, 0, 255}, 0x1C }, /* VA */
// };

typedef struct {
    const char *name;
    dlms_object_id_t object_id;
    uint16_t class_id;
    uint8_t expected_unit;
    uint8_t scaler_attribute;
    bool read_enabled;
} meter_register_t;

static const meter_register_t s_registers[] = {
    { "Import energy", DLMS_OBJECT_IMPORT_ACTIVE_TOTAL, 3U, 0x1E, 3U, true },
    { "Active power", DLMS_OBJECT_ACTIVE_POWER_TOTAL, 3U, 0x1B, 3U, true },
    { "Voltage L1", DLMS_OBJECT_VOLTAGE_L1, 3U, 0x23, 3U, true },
    { "Current L1", DLMS_OBJECT_CURRENT_L1, 3U, 0x21, 3U, true },
    { "Frequency", DLMS_OBJECT_FREQUENCY, 3U, 0x2C, 3U, true },
    { "Power factor", DLMS_OBJECT_POWER_FACTOR_TOTAL, 3U, 0xFF, 3U, true },
    { "Export energy", DLMS_OBJECT_EXPORT_ACTIVE_TOTAL, 3U, 0x1E, 3U, true },
    { "Reactive power", DLMS_OBJECT_REACTIVE_POWER_TOTAL, 3U, 0x1D, 3U, true },
    { "Apparent power", DLMS_OBJECT_APPARENT_POWER_TOTAL, 3U, 0x1C, 3U, true },
    { "Export active power", DLMS_OBJECT_EXPORT_ACTIVE_POWER, 3U, 0x1B, 3U, true },
    { "Import reactive energy", DLMS_OBJECT_IMPORT_REACTIVE_TOTAL,
      3U, 0x20, 3U, true },
    { "Export reactive energy", DLMS_OBJECT_EXPORT_REACTIVE_TOTAL,
      3U, 0x20, 3U, true },
    // { "Maximum demand", DLMS_OBJECT_MAXIMUM_DEMAND, 5U, 0x1B, 4U, false },
};

_Static_assert((sizeof(s_registers) / sizeof(s_registers[0])) == REGISTER_COUNT,
               "Update REGISTER_COUNT when adding meter registers");

static size_t s_register_index = 0U;
static int64_t s_register_raw = 0;
static uint8_t s_expected_attribute = 2U;
static volatile TickType_t s_next_poll_tick = 0U;
static volatile bool s_poll_scheduled = false;
static volatile TickType_t s_get_response_deadline = 0U;
static volatile uint8_t s_get_retry_count = 0U;
static volatile bool s_get_waiting_response = false;
static uint8_t s_get_retry_frame[32];
static size_t s_get_retry_frame_length;
static bool s_dlms_session_start_pending = false;
static TickType_t s_dlms_session_start_retry_tick = 0U;
static bool s_dlms_session_waiting_response = false;
static TickType_t s_dlms_session_response_deadline = 0U;
static uint8_t s_dlms_session_timeout_count = 0U;
static bool s_resume_cycle_after_session_restart = false;
static size_t s_resume_register_index = 0U;

#define DLMS_GET_RESPONSE_TIMEOUT_MS 5000U
#define DLMS_GET_MAX_RETRIES 2U
#define DLMS_SESSION_START_RETRY_MS 1000U
#define DLMS_SESSION_RESPONSE_TIMEOUT_MS 5000U
#define DLMS_SESSION_MAX_TIMEOUTS 3U


// static esp_err_t dlms_send_aarq(void)
// {
//     static const uint8_t aarq_apdu[] = {            

          
//             0x7E,
//             0xA0, 0x2B,
//             0x03, 0x21,
//             0x10,
//             0xFB, 0xAF,

//             0xE6, 0xE6, 0x00,
//             0x60, 0x1D,
//             0xA1, 0x09, 0x06, 0x07, 0x60, 0x85, 0x74, 0x05, 0x08, 0x01, 0x01,
//             0xBE, 0x10, 0x04, 0x0E,
//             0x01, 0x00, 0x00, 0x00, 0x06,
//             0x5F, 0x1F, 0x04, 0x00,
//             0x40, 0x13, 0xFD,
//             0xFF, 0xFF,

//             0xC9, 0xA7,
//             0x7E
//     };
//     uint8_t frame[64];
//     size_t frame_length = 
//        dlms_build_hdlc(
//              0x10,
//              aarq_apdu,
//              sizeof(aarq_apdu),
//               frame);

//     return ble_transport_write(frame, frame_length);
// }

static esp_err_t dlms_send_aarq(void)
{
    static const uint8_t aarq_apdu[] = {
        0xE6, 0xE6, 0x00, /* LLC   */

        0x60, 0x1D,

        0xA1, 0x09,
        0x06, 0x07,
        0x60, 0x85, 0x74, 0x05, 0x08, 0x01, 0x01,

        0xBE, 0x10,
        0x04, 0x0E,

        0x01, 0x00, 0x00, 0x00, 0x06,

        0x5F, 0x1F,
        0x04, 0x00,
        0x40, 0x13, 0xFD,

        0xFF, 0xFF
    };

    uint8_t frame[64];

    size_t frame_length =
        dlms_build_hdlc(
            dlms_link_next_i_control(),  // /* เดิม 0x10 */
            aarq_apdu,
            sizeof(aarq_apdu),
            frame);

    ESP_LOGI(TAG,
             "Sending AARQ APDU (%u bytes), HDLC frame (%u bytes)",
             (unsigned)sizeof(aarq_apdu),
             (unsigned)frame_length);

    return ble_transport_write(frame, frame_length);
}

static esp_err_t dlms_send_release(void)
{
    static const uint8_t release_apdu[] = {
        0xE6, 0xE6, 0x00,
        0x62, 0x03, 0x80, 0x01, 0x00
    };
    uint8_t frame[32];
    const size_t frame_length = dlms_build_hdlc(
        dlms_link_next_i_control(),
        release_apdu,
        sizeof(release_apdu),
        frame);

    if (frame_length == 0U || frame_length > sizeof(frame)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Sending DLMS Release Association");
    return ble_transport_write(frame, frame_length);
}

// static esp_err_t dlms_send_get_energy(void)
// {
//     static const uint8_t get_apdu[] = {
//         0xE6, 0xE6, 0x00,
//         0xC0, 0x01, 0xC1, 0x00,
//         0x00, 0x03,
//         0x09, 0x06, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF,
//         0x0F, 0x02, 0x00
//     };
//     uint8_t frame[48];
//     size_t frame_length = dlms_build_hdlc(0x32, get_apdu,
//                                           sizeof(get_apdu), frame);
//     return ble_transport_write(frame, frame_length);
// }

// static esp_err_t dlms_send_get_energy(uint8_t attribute)
// {
//     /*
//      * LLC: E6 E6 00
//      * GET.request-normal: C0 01 C1
//      * Class ID: 00 03 (Register)
//      * OBIS: 01 00 01 08 00 FF = 1.0.1.8.0.255
//      * Attribute: 02 = value, 03 = scaler_unit
//      * Access selection: 00
//      */
//     const uint8_t request[] = {
//         0xE6, 0xE6, 0x00,
//         0xC0, 0x01, 0xC1,
//         0x00, 0x03,
//         0x01, 0x00, 0x01, 0x08, 0x00, 0xFF,
//         attribute,
//         0x00
//     };

//     uint8_t frame[32];
//     size_t frame_length = dlms_build_hdlc(
//         dlms_link_next_i_control(),
//         request,
//         sizeof(request),
//         frame);

//     if (frame_length == 0U || frame_length > sizeof(frame)) {
//         return ESP_ERR_INVALID_SIZE;
//     }

//     ESP_LOGI(TAG, "Request energy attribute %u", (unsigned)attribute);
//     return ble_transport_write(frame, frame_length);
// }



static esp_err_t dlms_send_get_register(size_t index, uint8_t attribute)
{
    if (index >= REGISTER_COUNT ||
        (attribute != 2U &&
         attribute != s_registers[index].scaler_attribute)) {
        return ESP_ERR_INVALID_ARG;
    }

    const meter_register_t *reg = &s_registers[index];

    const dlms_object_info_t *obj = dlms_obis_get(reg->object_id);


    if (obj == NULL) {

        ESP_LOGE(TAG, "OBIS not found for %s", reg->name);

        return ESP_ERR_NOT_FOUND;

    }


    if (obj->attribute != 2U || obj->access != DLMS_ACCESS_READ) {

        ESP_LOGE(TAG, "Unsupported object configuration: %s", reg->name);

        return ESP_ERR_INVALID_STATE;

    }




    uint8_t request[] = {
        0xE6, 0xE6, 0x00,       /* LLC */
        0xC0, 0x01, 0xC1,       /* GET.request-normal, invoke C1 */
        0x00, 0x00,             /* COSEM interface class */
        0, 0, 0, 0, 0, 0,       /* OBIS */
        attribute,
        0x00                  /* no selective access */
    };

    request[6] = (uint8_t)(reg->class_id >> 8U);
    request[7] = (uint8_t)reg->class_id;
    memcpy(&request[8], obj->obis, sizeof(obj->obis));

    uint8_t frame[32];
    size_t frame_length = dlms_build_hdlc(
        dlms_link_next_i_control(),
        request,
        sizeof(request),
        frame);

    /* request ขนาดคงที่นี้ควรได้ HDLC frame 27 bytes */
    if (frame_length != 27U || frame_length > sizeof(frame)) {
        ESP_LOGE(TAG, "Unexpected GET frame length: %u",
                 (unsigned)frame_length);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(s_get_retry_frame, frame, frame_length);
    s_get_retry_frame_length = frame_length;

    ESP_LOGI(TAG, "Request %s attribute %u",
             reg->name, (unsigned)attribute);

    return ble_transport_write(frame, frame_length);
}

static esp_err_t dlms_retry_get_register(void)
{
    if (s_get_retry_frame_length == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Retransmitting %s attribute %u with the same HDLC frame",
             s_registers[s_register_index].name,
             (unsigned)s_expected_attribute);
    return ble_transport_write(s_get_retry_frame,
                               s_get_retry_frame_length);
}

static esp_err_t dlms_send_session_start(void)
{
    s_dlms_session_start_pending = false;
    s_dlms_session_waiting_response = true;
    s_dlms_session_response_deadline = xTaskGetTickCount() +
        pdMS_TO_TICKS(DLMS_SESSION_RESPONSE_TIMEOUT_MS);

    esp_err_t err = ble_transport_write(dlms_snrm, sizeof(dlms_snrm));
    if (err != ESP_OK) {
        s_dlms_session_waiting_response = false;
        s_dlms_session_start_pending = true;
        s_dlms_session_start_retry_tick = xTaskGetTickCount() +
            pdMS_TO_TICKS(DLMS_SESSION_START_RETRY_MS);
    }
    return err;
}

static esp_err_t dlms_start_get_register(size_t index, uint8_t attribute)
{
    s_get_retry_count = 0U;
    s_get_waiting_response = true;
    s_get_response_deadline = xTaskGetTickCount() +
        pdMS_TO_TICKS(DLMS_GET_RESPONSE_TIMEOUT_MS);

    esp_err_t err = dlms_send_get_register(index, attribute);
    if (err != ESP_OK) {
        s_get_waiting_response = false;
    }
    return err;
}

static void dlms_restart_session(const char *reason,
                                bool resume_cycle,
                                size_t resume_register_index)
{
    ESP_LOGW(TAG, "Restarting DLMS session: %s", reason);
    s_get_waiting_response = false;
    s_get_retry_frame_length = 0U;
    s_expected_attribute = 2U;
    s_register_index = 0U;
    s_poll_scheduled = false;
    s_dlms_session_waiting_response = false;
    s_resume_cycle_after_session_restart = resume_cycle;
    s_resume_register_index = resume_register_index;
    dlms_link_reset();
    dlms_state = 0U;
    s_dlms_session_start_pending = true;
    s_dlms_session_start_retry_tick = xTaskGetTickCount();
}

static void schedule_next_meter_read(void)
{
    s_next_poll_tick = xTaskGetTickCount() +
        pdMS_TO_TICKS(METER_READ_INTERVAL_SECONDS * 1000U);
    s_poll_scheduled = true;
}

static void dlms_request_next_register(void)
{
    while (++s_register_index < REGISTER_COUNT) {
        if (!s_registers[s_register_index].read_enabled) {
            ESP_LOGI(TAG, "Skipping disabled meter register: %s",
                     s_registers[s_register_index].name);
            continue;
        }

        s_expected_attribute = 2U;
        esp_err_t err = dlms_start_get_register(s_register_index, 2U);
        if (err == ESP_OK) {
            return;
        }

        ESP_LOGE(TAG, "Failed to request %s: %s; skipping register",
                 s_registers[s_register_index].name, esp_err_to_name(err));
        invalidate_meter_value(s_register_index);
    }

    ESP_LOGI(TAG, "Finished reading all meter registers");
    s_get_waiting_response = false;
    dlms_state = 5U;
    s_release_response_waiting = true;
    s_release_response_deadline = xTaskGetTickCount() +
        pdMS_TO_TICKS(DLMS_RELEASE_RESPONSE_TIMEOUT_MS);
    esp_err_t release_err = dlms_send_release();
    if (release_err != ESP_OK) {
        ESP_LOGW(TAG, "Could not send DLMS Release Association: %s",
                 esp_err_to_name(release_err));
        s_release_response_waiting = false;
        dlms_state = 6U;
    }
}

static void dlms_skip_current_register(const char *reason)
{
    ESP_LOGW(TAG, "Skipping %s: %s",
             s_registers[s_register_index].name, reason);
    invalidate_meter_value(s_register_index);
    dlms_request_next_register();
}

/*
 * ตัดสินใจหลังจาก COSEM จัดการข้อมูลแล้ว
 *  - HDLC segmented → ส่ง RR เสมอ (หน้าที่ของ HDLC layer)
 *  - ไม่ segmented  → COSEM จัดการ GET-NEXT เอง หรือรับครบแล้ว
 */
// static void dlms_handle_get_result(esp_err_t err, bool segmented)
// {
//     if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
//         ESP_LOGE(TAG, "GET handling failed: %s", esp_err_to_name(err));
//         return;
//     }

//     if (segmented) {
//         esp_err_t rr_err = dlms_link_send_rr();
//         if (rr_err != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to send RR: %s", esp_err_to_name(rr_err));
//         }
//         return;
//     }

//     if (err == ESP_ERR_NOT_FINISHED) {
//         /* HDLC บอกว่าหมด segment แล้ว แต่ข้อมูลบล็อกยังไม่ครบ = ผิดปกติ */
//         ESP_LOGW(TAG, "Last HDLC segment received but DLMS block incomplete");
//         return;
//     }

//     if (dlms_cosem_is_association_view_complete()) {
//         ESP_LOGI(TAG, "Association View COMPLETE");
//         dlms_state = 3U;
//         /* ขั้นต่อไป: parse Association View หา OBIS */
//     }
//     /* ถ้ายังไม่ complete: dlms_cosem ส่ง GET-NEXT ไปแล้ว รอบล็อกถัดไป */
// }

static bool decode_register_value(
    const uint8_t *data,
    size_t length,
    int64_t *value)
{
    if (data == NULL || value == NULL || length == 0U) {
        return false;
    }

    /* รองรับชนิดที่พบบ่อย: double-long-unsigned */
    if (data[0] == 0x06U && length == 5U) {
        uint32_t raw =
            ((uint32_t)data[1] << 24U) |
            ((uint32_t)data[2] << 16U) |
            ((uint32_t)data[3] << 8U) |
            (uint32_t)data[4];
        *value = (int64_t)raw;
        return true;
    }

    /* double-long แบบ signed */
    if (data[0] == 0x05U && length == 5U) {
        uint32_t raw =
            ((uint32_t)data[1] << 24U) |
            ((uint32_t)data[2] << 16U) |
            ((uint32_t)data[3] << 8U) |
            (uint32_t)data[4];

        *value = (raw & 0x80000000U)
            ? (int64_t)raw - 4294967296LL
            : (int64_t)raw;
        return true;
    }

    /* long-unsigned */
    if (data[0] == 0x12U && length == 3U) {
        *value = (int64_t)(((uint16_t)data[1] << 8U) | data[2]);
        return true;
    }

    /* Some meters encode values below 256 as a single byte after this tag. */
    if (data[0] == 0x12U && length == 2U) {
        *value = data[1];
        return true;
    }

    /* long แบบ signed */
    if (data[0] == 0x10U && length == 3U) {
        uint16_t raw = (uint16_t)(((uint16_t)data[1] << 8U) | data[2]);
        *value = (raw & 0x8000U)
            ? (int64_t)raw - 65536LL
            : (int64_t)raw;
        return true;
    }

    /* unsigned 8-bit / signed 8-bit */
    if (data[0] == 0x11U && length == 2U) {
        *value = data[1];
        return true;
    }
    if (data[0] == 0x0FU && length == 2U) {
        *value = (int8_t)data[1];
        return true;
    }

    return false;
}

static double apply_scaler(int64_t raw, int8_t scaler)
{
    double result = (double)raw;

    while (scaler > 0) {
        result *= 10.0;
        --scaler;
    }
    while (scaler < 0) {
        result /= 10.0;
        ++scaler;
    }

    return result;
}




static void dlms_on_rx(const uint8_t *data, size_t length)
{
    if (data == NULL || length < 9U) {
        return;
    }

    dlms_hdlc_frame_t frame = {0};
    dlms_hdlc_parse_result_t result =
        dlms_hdlc_parse_frame(data, length, &frame);

    if (result != DLMS_HDLC_PARSE_OK) {
        ESP_LOGE(TAG, "HDLC parse failed: %s",
                 dlms_hdlc_parse_result_to_string(result));
        return;
    }

    /* อัปเดตเลขลำดับ HDLC ทุกเฟรม (ต้องทำก่อนส่งอะไรกลับ) */
    dlms_link_on_rx_control(frame.control);

    /* Format byte bit3 = S-bit (segmented) */
    const bool segmented = (data[1] & 0x08U) != 0U;

    ESP_LOGI(TAG, "DLMS RX: control=0x%02X info_len=%u segmented=%d",
             frame.control, (unsigned)frame.information_length, segmented);

    /* ---------------- State 0 : รอ UA ---------------- */
    if (dlms_state == 0U) {
        if (frame.control == 0x73U) {
            ESP_LOGI(TAG, "DLMS UA received; sending AARQ");
              s_dlms_session_waiting_response = true;
              s_dlms_session_response_deadline = xTaskGetTickCount() +
                  pdMS_TO_TICKS(DLMS_SESSION_RESPONSE_TIMEOUT_MS);
              if (dlms_send_aarq() == ESP_OK) {
                  dlms_state = 1U;
              }
              else {
                  s_dlms_session_waiting_response = false;
                  s_dlms_session_start_pending = true;
                  s_dlms_session_start_retry_tick = xTaskGetTickCount() +
                      pdMS_TO_TICKS(DLMS_SESSION_START_RETRY_MS);
              }
        }
        return;
    }

    /* เฟรมไม่มีข้อมูล (เช่น RR/RNR จากมิเตอร์) */
    if (!frame.has_information ||
        frame.information == NULL ||
        frame.information_length == 0U) {
        return;
    }

    /* -------- State 2 : continuation segment (ไม่มี LLC) -------- */
    // if (dlms_state == 2U && dlms_cosem_is_waiting_hdlc_segment()) {
    //     ESP_LOGI(TAG, "HDLC continuation segment: %u bytes",
    //              (unsigned)frame.information_length);

    //     esp_err_t err = dlms_cosem_append_get_data(
    //         frame.information, frame.information_length);

    //     dlms_handle_get_result(err, segmented);
    //     return;
    // }

    /* เฟรมแรกของ APDU ต้องขึ้นต้นด้วย LLC E6 E7 00 */
    if (frame.information_length < 4U ||
        frame.information[0] != 0xE6U ||
        frame.information[1] != 0xE7U ||
        frame.information[2] != 0x00U) {
        ESP_LOGW(TAG, "Unexpected LLC: %02X %02X %02X",
                 frame.information[0], frame.information[1],
                 frame.information[2]);
        return;
    }

    const uint8_t *apdu = &frame.information[3];
    size_t apdu_length = frame.information_length - 3U;

    ESP_LOGI(TAG, "DLMS APDU: tag=0x%02X length=%u",
             apdu[0], (unsigned)apdu_length);

    if (dlms_state == 5U) {
        if (apdu[0] == 0x63U) {
            ESP_LOGI(TAG, "DLMS Release Association response received");
            s_release_response_waiting = false;
            dlms_state = 6U;
        } else {
            ESP_LOGW(TAG,
                     "Unexpected DLMS response while waiting for release "
                     "(tag=0x%02X)",
                     apdu[0]);
        }
        return;
    }

    /* ---------------- State 1 : รอ AARE ---------------- */
    if (dlms_state == 1U) {
      if (apdu[0] == 0x61U) {
          ESP_LOGI(TAG, "DLMS AARE received; reading meter registers");
          s_dlms_session_waiting_response = false;
          s_dlms_session_timeout_count = 0U;
          s_expected_attribute = 2U;
          dlms_state = 2U;

          if (s_resume_cycle_after_session_restart) {
              const size_t resume_index = s_resume_register_index;
              s_resume_cycle_after_session_restart = false;
              if (resume_index == 0U) {
                  s_register_index = 0U;
                  if (dlms_start_get_register(0U, 2U) != ESP_OK) {
                      ESP_LOGE(TAG, "Failed to request first register");
                      dlms_restart_session(
                          "initial GET request failed", false, 0U);
                  }
              } else {
                  s_register_index = resume_index - 1U;
                  dlms_request_next_register();
              }
          } else {
              s_register_index = 0U;
              if (dlms_start_get_register(s_register_index, 2U) != ESP_OK) {
                  ESP_LOGE(TAG, "Failed to request first register");
                  dlms_restart_session(
                      "initial GET request failed", false, 0U);
              }
          }
      }
      return;
    }

        /* State 2: อ่าน attribute 2 และ 3 ของแต่ละ Register ตามลำดับ */
    if (dlms_state == 2U) {
        if (segmented) {
            ESP_LOGE(TAG,
                     "Segmented register response; reassembly not implemented");
            dlms_skip_current_register("segmented response is unsupported");
            return;
        }

        if (apdu_length < 5U ||
            apdu[0] != 0xC4U ||
            apdu[1] != 0x01U ||
            apdu[2] != 0xC1U) {
            ESP_LOGE(TAG, "Unexpected GET response");
            dlms_skip_current_register("unexpected GET response");
            return;
        }

        const meter_register_t *reg = &s_registers[s_register_index];

        if (apdu[3] == 0x01U) {
            ESP_LOGE(TAG, "%s attribute %u: DataAccessError=0x%02X",
                     reg->name,
                     (unsigned)s_expected_attribute,
                     apdu[4]);
            dlms_skip_current_register("meter returned DataAccessError");
            return;
        }

        if (apdu[3] != 0x00U) {
            ESP_LOGE(TAG, "Unexpected GET result=0x%02X", apdu[3]);
            dlms_skip_current_register("unexpected GET result");
            return;
        }

        s_get_waiting_response = false;

        if (s_expected_attribute == 2U) {
            if (!decode_register_value(
                    &apdu[4], apdu_length - 4U, &s_register_raw)) {
                const size_t encoded_value_length = apdu_length - 4U;
                if (apdu[4] == 0x05U && encoded_value_length != 5U) {
                    ESP_LOGE(TAG,
                             "%s: incomplete AXDR double-long value "
                             "(encoded length=%u, expected=5)",
                             reg->name, (unsigned)encoded_value_length);
                } else {
                    ESP_LOGE(TAG, "%s: unsupported value format", reg->name);
                }
                ESP_LOG_BUFFER_HEX_LEVEL(
                    TAG, &apdu[4], apdu_length - 4U, ESP_LOG_ERROR);
                dlms_skip_current_register("invalid or incomplete value");
                return;
            }

            if (apdu[4] == 0x12U && apdu_length - 4U == 2U) {
                ESP_LOGW(TAG,
                         "%s: meter sent a one-byte long-unsigned value; "
                         "assuming a zero high byte",
                         reg->name);
            }

            ESP_LOGI(TAG, "%s raw=%" PRId64,
                     reg->name, s_register_raw);

            s_expected_attribute = reg->scaler_attribute;
            if (dlms_start_get_register(
                    s_register_index, reg->scaler_attribute) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to request scaler/unit");
                dlms_skip_current_register("failed to request scaler/unit");
            }
            return;
        }

        /* AXDR structure: 02 02 0F scaler 16 unit */
        if (apdu_length != 10U ||
            apdu[4] != 0x02U ||
            apdu[5] != 0x02U ||
            apdu[6] != 0x0FU ||
            apdu[8] != 0x16U) {
            ESP_LOGE(TAG, "%s: unexpected scaler/unit format",
                     reg->name);
            ESP_LOG_BUFFER_HEX_LEVEL(
                TAG, &apdu[4], apdu_length - 4U, ESP_LOG_ERROR);
            dlms_skip_current_register("unexpected scaler/unit format");
            return;
        }

        const int8_t scaler = (int8_t)apdu[7];
        const uint8_t unit = apdu[9];

        // if (unit != reg->expected_unit) {
        if (reg->expected_unit != 0xFFU && unit != reg->expected_unit) {
            ESP_LOGE(TAG,
                     "%s: unit=0x%02X, expected=0x%02X; not converting",
                     reg->name, unit, reg->expected_unit);
            dlms_skip_current_register("unexpected unit");
            return;
        }

        const double scaled = apply_scaler(s_register_raw, scaler);

        switch (s_register_index) {
            case 0U:
                store_meter_value(0U, scaled / 1000.0);
                ESP_LOGI(TAG, "Import energy: %.3f kWh",
                         scaled / 1000.0); /* Wh → kWh */
                break;
            case 1U:
                store_meter_value(1U, scaled / 1000.0);
                ESP_LOGI(TAG, "Active power: %.3f kW",
                         scaled / 1000.0); /* W → kW */
                break;
            case 2U:
                // ESP_LOGI(TAG, "Voltage L1: %.3f V", scaled);
                /* สมมติฐานเฉพาะมิเตอร์นี้: ค่าแรงดันที่อ่านสูงกว่าหน้างาน 10 เท่า */

                  const double voltage_v = scaled / 10.0;
                  store_meter_value(2U, voltage_v);
                  ESP_LOGI(TAG, "Voltage L1: %.2f V (meter scaled=%.2f)",
                  voltage_v, scaled);
                break;

            case 3U:
                // ESP_LOGI(TAG, "Current L1: %.3f A", scaled);
                /* ทดลองเทียบกับแคลมป์มิเตอร์และกำลังไฟฟ้าที่อ่านได้ */
                    const double current_a = scaled / 10.0;
                    store_meter_value(3U, current_a);
                    ESP_LOGI(TAG, "Current L1: %.3f A (meter scaled=%.3f)",
                    current_a, scaled);
                break;

            case 4U:
                store_meter_value(4U, scaled);
                ESP_LOGI(TAG, "Frequency: %.3f Hz", scaled);
                break;

            case 5U:
                store_meter_value(5U, scaled);
                ESP_LOGI(TAG,
                        "Power factor: %.4f (raw=%" PRId64
                        ", scaler=%d, unit=0x%02X)",
                        scaled,
                        s_register_raw,
                        (int)scaler,
                        unit);

                     if (scaled < -1.0 || scaled > 1.0) {
                        ESP_LOGW(TAG,
                            "Power factor outside -1..1; check meter format");
                        }  
                break;

            case 6U:
                store_meter_value(6U, scaled / 1000.0);
                ESP_LOGI(TAG, "Export energy: %.3f kWh",
                        scaled / 1000.0);  /* Wh → kWh */
                break;

            case 7U:
                store_meter_value(7U, scaled / 1000.0);
                ESP_LOGI(TAG, "Reactive power: %.3f kvar",
                        scaled / 1000.0);  /* var → kvar */
                break;

            case 8U:
                store_meter_value(8U, scaled / 1000.0);
                ESP_LOGI(TAG, "Apparent power: %.3f kVA",
                        scaled / 1000.0);  /* VA → kVA */
                break;

            case 9U:
                store_meter_value(9U, scaled / 1000.0);
                ESP_LOGI(TAG, "Export active power: %.3f kW",
                         scaled / 1000.0);
                break;

            case 10U:
                store_meter_value(10U, scaled / 1000.0);
                ESP_LOGI(TAG, "Import reactive energy: %.3f kvarh",
                         scaled / 1000.0);
                break;

            case 11U:
                store_meter_value(11U, scaled / 1000.0);
                ESP_LOGI(TAG, "Export reactive energy: %.3f kvarh",
                         scaled / 1000.0);
                break;

            // case 12U:
            //     store_meter_value(12U, scaled / 1000.0);
            //     ESP_LOGI(TAG, "Maximum demand: %.3f kW",
            //              scaled / 1000.0);
            //     break;

            default:
                break;
        }

        dlms_request_next_register();
        return;
    }


} /* ปิด dlms_on_rx() */








#define BLINK_GPIO CONFIG_BLINK_GPIO
static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

static void blink_led(void) 
{
    if (s_led_state) {
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO
static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Application Start");


    ESP_ERROR_CHECK(dlms_hdlc_init());

    // Initialize the BLE host once; meter connections are started per cycle.
    ESP_ERROR_CHECK(ble_transport_driver_init());
    ESP_ERROR_CHECK(start_wifi());
    ESP_ERROR_CHECK(load_ble_selection());
    if (s_ble_selection_saved) {
        ESP_ERROR_CHECK(ble_transport_set_target_addr(
            s_ble_selected_address, s_ble_selected_address_type));
    } else if (s_ble_selection_cleared) {
        ESP_ERROR_CHECK(ble_transport_reset_target_addr());
    }
    ESP_ERROR_CHECK(ble_transport_set_rx_callback(dlms_on_rx));

    configure_led();
    ESP_ERROR_CHECK(tft_init());
    ESP_ERROR_CHECK(tft_render_startup_test_pattern());
    if (xTaskCreate(tft_display_task, "tft_display", 4096U, NULL, 3U,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TFT display task");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    TickType_t last_led_toggle_tick = xTaskGetTickCount();
    s_next_poll_tick = last_led_toggle_tick;
    s_poll_scheduled = true;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t blink_interval = pdMS_TO_TICKS(CONFIG_BLINK_PERIOD);

        if ((TickType_t)(now - last_led_toggle_tick) >= blink_interval) {
            blink_led();
            s_led_state = !s_led_state;
            last_led_toggle_tick = now;
        }

        if (s_meter_cycle_phase == METER_CYCLE_WAITING &&
            s_poll_scheduled &&
            (int32_t)(now - s_next_poll_tick) >= 0) {
            portENTER_CRITICAL(&s_dashboard_mux);
            memset(s_meter_value_valid, 0, sizeof(s_meter_value_valid));
            portEXIT_CRITICAL(&s_dashboard_mux);

            esp_err_t cycle_err = ble_transport_start();
            if (cycle_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start BLE meter cycle: %s",
                         esp_err_to_name(cycle_err));
                schedule_next_meter_read();
            } else {
                ESP_LOGI(TAG, "Starting 5-minute meter read cycle");
                s_poll_scheduled = false;
                s_meter_cycle_phase = METER_CYCLE_CONNECTING;
                s_cycle_connect_deadline = now +
                    pdMS_TO_TICKS(METER_CYCLE_CONNECT_TIMEOUT_MS);
                s_ble_stop_requested = false;
                s_release_response_waiting = false;
                s_dlms_session_start_pending = false;
                s_dlms_session_waiting_response = false;
                s_get_waiting_response = false;
                s_get_retry_frame_length = 0U;
                s_dlms_session_timeout_count = 0U;
                s_resume_cycle_after_session_restart = false;
                dlms_link_reset();
                dlms_state = 0U;
            }
        }

        if (s_meter_cycle_phase == METER_CYCLE_CONNECTING) {
            if (ble_transport_is_ready()) {
                ESP_LOGI(TAG,
                         "BLE connected and ready; starting DLMS handshake");
                s_meter_cycle_phase = METER_CYCLE_READING;
                dlms_link_reset();
                dlms_state = 0U;
                esp_err_t session_err = dlms_send_session_start();
                if (session_err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send DLMS SNRM: %s",
                             esp_err_to_name(session_err));
                } else {
                    ESP_LOGI(TAG, "DLMS SNRM sent; waiting for UA");
                }
            } else if ((int32_t)(now - s_cycle_connect_deadline) >= 0) {
                ESP_LOGW(TAG,
                         "BLE connection did not become ready before timeout");
                s_meter_cycle_phase = METER_CYCLE_DISCONNECTING;
                s_ble_stop_requested = false;
                s_disconnect_retry_tick = now;
            }
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            !ble_transport_is_ready()) {
            ESP_LOGW(TAG,
                     "BLE link lost during meter read; waiting for reconnect");
            s_get_waiting_response = false;
            s_get_retry_frame_length = 0U;
            s_dlms_session_waiting_response = false;
            s_dlms_session_start_pending = false;
            s_resume_cycle_after_session_restart = false;
            dlms_link_reset();
            dlms_state = 0U;
            s_meter_cycle_phase = METER_CYCLE_CONNECTING;
            s_cycle_connect_deadline = now +
                pdMS_TO_TICKS(METER_CYCLE_CONNECT_TIMEOUT_MS);
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            ble_transport_is_ready() &&
            s_dlms_session_start_pending &&
            (int32_t)(now - s_dlms_session_start_retry_tick) >= 0) {
            esp_err_t start_err = dlms_send_session_start();
            if (start_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send DLMS SNRM: %s",
                         esp_err_to_name(start_err));
            } else {
                ESP_LOGI(TAG, "DLMS SNRM sent; waiting for UA");
            }
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            (dlms_state == 0U || dlms_state == 1U) &&
            s_dlms_session_waiting_response &&
            (int32_t)(now - s_dlms_session_response_deadline) >= 0) {
            ++s_dlms_session_timeout_count;
            if (s_dlms_session_timeout_count >=
                DLMS_SESSION_MAX_TIMEOUTS) {
                ESP_LOGW(TAG,
                         "DLMS handshake timed out %u times; reconnecting "
                         "BLE transport",
                         (unsigned)s_dlms_session_timeout_count);
                s_dlms_session_timeout_count = 0U;
                dlms_restart_session("UA/AARE response timeout", false, 0U);
                esp_err_t reconnect_err = ble_transport_reconnect();
                if (reconnect_err != ESP_OK) {
                    ESP_LOGE(TAG, "BLE recovery reconnect failed: %s",
                             esp_err_to_name(reconnect_err));
                }
            } else {
                ESP_LOGW(TAG,
                         "DLMS session handshake timed out; retrying SNRM "
                         "(%u/%u)",
                         (unsigned)s_dlms_session_timeout_count,
                         (unsigned)DLMS_SESSION_MAX_TIMEOUTS);
                dlms_restart_session("UA/AARE response timeout", false, 0U);
            }
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            dlms_state == 2U &&
            s_get_waiting_response &&
            (int32_t)(now - s_get_response_deadline) >= 0) {
            if (s_get_retry_count < DLMS_GET_MAX_RETRIES) {
                ++s_get_retry_count;
                s_get_response_deadline = now +
                    pdMS_TO_TICKS(DLMS_GET_RESPONSE_TIMEOUT_MS);
                ESP_LOGW(TAG, "No response for %s attribute %u; retry %u/%u",
                         s_registers[s_register_index].name,
                         (unsigned)s_expected_attribute,
                         (unsigned)s_get_retry_count,
                         (unsigned)DLMS_GET_MAX_RETRIES);
                esp_err_t retry_err = dlms_retry_get_register();
                if (retry_err != ESP_OK) {
                    ESP_LOGE(TAG, "GET retry failed: %s",
                             esp_err_to_name(retry_err));
                }
            } else {
                s_get_waiting_response = false;
                ESP_LOGE(TAG, "No response for %s attribute %u after %u retries",
                         s_registers[s_register_index].name,
                         (unsigned)s_expected_attribute,
                         (unsigned)DLMS_GET_MAX_RETRIES);
                invalidate_meter_value(s_register_index);
                ESP_LOGW(TAG,
                         "Skipping %s for this read cycle after session "
                         "recovery",
                         s_registers[s_register_index].name);
                dlms_restart_session("GET response timeout", true,
                                     s_register_index + 1U);
            }
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            s_release_response_waiting &&
            (int32_t)(now - s_release_response_deadline) >= 0) {
            ESP_LOGW(TAG,
                     "DLMS Release Association response timed out; "
                     "disconnecting anyway");
            s_release_response_waiting = false;
            dlms_state = 6U;
        }

        if (s_meter_cycle_phase == METER_CYCLE_READING &&
            dlms_state == 6U) {
            s_meter_cycle_phase = METER_CYCLE_DISCONNECTING;
            s_ble_stop_requested = false;
            s_disconnect_retry_tick = now;
        }

        if (s_meter_cycle_phase == METER_CYCLE_DISCONNECTING) {
            if (!s_ble_stop_requested &&
                (int32_t)(now - s_disconnect_retry_tick) >= 0) {
                esp_err_t stop_err = ble_transport_stop();
                if (stop_err == ESP_OK) {
                    s_ble_stop_requested = true;
                } else {
                    ESP_LOGE(TAG, "Failed to stop BLE transport: %s",
                             esp_err_to_name(stop_err));
                    s_disconnect_retry_tick = now +
                        pdMS_TO_TICKS(BLE_DISCONNECT_RETRY_MS);
                }
            }

            if (ble_transport_is_idle()) {
                double values[REGISTER_COUNT];
                bool valid[REGISTER_COUNT];
                bool wifi_connected;
                portENTER_CRITICAL(&s_dashboard_mux);
                memcpy(values, s_meter_values, sizeof(values));
                memcpy(valid, s_meter_value_valid, sizeof(valid));
                wifi_connected = s_wifi_ip[0] != '\0';
                portEXIT_CRITICAL(&s_dashboard_mux);

                if (wifi_connected) {
                    esp_err_t mqtt_err = meter_mqtt_publish(
                        values, valid, REGISTER_COUNT);
                    if (mqtt_err != ESP_OK) {
                        ESP_LOGW(TAG,
                                 "Could not queue meter data for MQTT: %s",
                                 esp_err_to_name(mqtt_err));
                    }
                }

                ESP_LOGI(TAG,
                         "Meter cycle complete; BLE disconnected before "
                         "cloud upload");
                s_meter_cycle_phase = METER_CYCLE_WAITING;
                s_release_response_waiting = false;
                dlms_state = 0U;
                schedule_next_meter_read();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}