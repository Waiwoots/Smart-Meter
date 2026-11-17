#include "meter_mqtt.h"

#include <stdio.h>

#include "sdkconfig.h"

#define METER_MQTT_REGISTER_COUNT 12U
#define METER_MQTT_PAYLOAD_SIZE 32U

#if CONFIG_SMART_METER_MQTT_ENABLED
#include <math.h>
#include <stdint.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "METER_MQTT";
extern const uint8_t mqtt_ca_pem_start[] asm("_binary_emqx_ca_pem_start");

static const char *const s_topics[METER_MQTT_REGISTER_COUNT] = {
    "import_energy_kwh",
    "active_power",
    "voltage_l1",
    "current_l1",
    "frequency",
    "power_factor",
    "export_energy",
    "reactive_power",
    "apparent_power",
    "export_active",
    "import_reactive",
    "export_reactive",
    // "maximum_demand",
};

static esp_mqtt_client_handle_t s_client;
static SemaphoreHandle_t s_mqtt_api_mutex;
static portMUX_TYPE s_mqtt_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_connected;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        portENTER_CRITICAL(&s_mqtt_mux);
        s_connected = true;
        portEXIT_CRITICAL(&s_mqtt_mux);
        ESP_LOGI(TAG, "Connected to MQTT broker");
        break;
    case MQTT_EVENT_DISCONNECTED:
        portENTER_CRITICAL(&s_mqtt_mux);
        s_connected = false;
        portEXIT_CRITICAL(&s_mqtt_mux);
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT transport error");
        break;
    default:
        break;
    }
}
#endif

esp_err_t meter_mqtt_start(void)
{
#if !CONFIG_SMART_METER_MQTT_ENABLED
    return ESP_OK;
#else
    if (s_mqtt_api_mutex == NULL) {
        s_mqtt_api_mutex = xSemaphoreCreateMutex();
        if (s_mqtt_api_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create MQTT API mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    if (xSemaphoreTake(s_mqtt_api_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    if (s_client != NULL) {
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_OK;
    }
    if (CONFIG_SMART_METER_MQTT_BROKER_URI[0] == '\0') {
        ESP_LOGE(TAG, "MQTT is enabled but broker URI is empty");
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    const esp_mqtt_client_config_t config = {
        .broker.address.uri = CONFIG_SMART_METER_MQTT_BROKER_URI,
        .broker.verification.certificate = (const char *)mqtt_ca_pem_start,
        .credentials.username = CONFIG_SMART_METER_MQTT_USERNAME,
        .credentials.authentication.password =
            CONFIG_SMART_METER_MQTT_PASSWORD,
    };
    s_client = esp_mqtt_client_init(&config);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err == ESP_OK) {
        err = esp_mqtt_client_start(s_client);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s",
                 esp_err_to_name(err));
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        xSemaphoreGive(s_mqtt_api_mutex);
        return err;
    }
    ESP_LOGI(TAG, "MQTT client started");
    xSemaphoreGive(s_mqtt_api_mutex);
    return ESP_OK;
#endif
}

esp_err_t meter_mqtt_stop(void)
{
#if !CONFIG_SMART_METER_MQTT_ENABLED
    return ESP_OK;
#else
    if (s_mqtt_api_mutex == NULL) {
        return ESP_OK;
    }
    if (xSemaphoreTake(s_mqtt_api_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    if (s_client == NULL) {
        portENTER_CRITICAL(&s_mqtt_mux);
        s_connected = false;
        portEXIT_CRITICAL(&s_mqtt_mux);
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_OK;
    }

    esp_err_t err = esp_mqtt_client_stop(s_client);
    if (err == ESP_OK) {
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        portENTER_CRITICAL(&s_mqtt_mux);
        s_connected = false;
        portEXIT_CRITICAL(&s_mqtt_mux);
    } else {
        ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_mqtt_api_mutex);
    return err;
#endif
}

esp_err_t meter_mqtt_publish(const double *values, const bool *valid,
                             size_t count)
{
#if !CONFIG_SMART_METER_MQTT_ENABLED
    (void)values;
    (void)valid;
    (void)count;
    return ESP_OK;
#else
    if (values == NULL || valid == NULL ||
        count != METER_MQTT_REGISTER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mqtt_api_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mqtt_api_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    if (s_client == NULL) {
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_mqtt_mux);
    const bool connected = s_connected;
    portEXIT_CRITICAL(&s_mqtt_mux);
    if (!connected) {
        ESP_LOGW(TAG, "Skipping telemetry because MQTT is disconnected");
        xSemaphoreGive(s_mqtt_api_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t publish_err = ESP_OK;
    for (size_t i = 0U; i < METER_MQTT_REGISTER_COUNT; ++i) {
        char payload[METER_MQTT_PAYLOAD_SIZE];
        int written;
        if (valid[i] && isfinite(values[i])) {
            written = snprintf(payload, sizeof(payload), "%.6g", values[i]);
        } else {
            written = snprintf(payload, sizeof(payload), "null");
        }
        if (written < 0 || (size_t)written >= sizeof(payload)) {
            ESP_LOGE(TAG, "Failed to format telemetry for topic '%s'",
                     s_topics[i]);
            publish_err = ESP_ERR_INVALID_SIZE;
            continue;
        }

        const int message_id = esp_mqtt_client_enqueue(
            s_client, s_topics[i], payload, written, 1, 0, true);
        if (message_id < 0) {
            ESP_LOGE(TAG, "Failed to enqueue telemetry for topic '%s'",
                     s_topics[i]);
            publish_err = ESP_FAIL;
        } else {
            ESP_LOGD(TAG, "Queued '%s' (message id=%d)", s_topics[i],
                     message_id);
        }
    }
    xSemaphoreGive(s_mqtt_api_mutex);
    return publish_err;
#endif
}
