#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <stdbool.h>
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define BLE_TRANSPORT_SCAN_RESULT_COUNT 20U
#define BLE_TRANSPORT_DEVICE_NAME_LENGTH 32U

typedef struct {
    uint8_t address[6];
    uint8_t address_type;
    int8_t rssi;
    char name[BLE_TRANSPORT_DEVICE_NAME_LENGTH];
} ble_transport_scan_result_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_transport_driver_init(void);

esp_err_t ble_transport_start(void);

esp_err_t ble_transport_stop(void);

esp_err_t ble_transport_reconnect(void);

bool ble_transport_is_ready(void);

bool ble_transport_is_idle(void);

esp_err_t ble_transport_write(
        const uint8_t *data,
        size_t length);

esp_err_t ble_transport_get_rssi(int8_t *rssi);

esp_err_t ble_transport_set_target_addr(const uint8_t address[6],
                                        uint8_t address_type);

esp_err_t ble_transport_reset_target_addr(void);

esp_err_t ble_transport_get_target_addr(uint8_t address[6],
                                        uint8_t *address_type);

esp_err_t ble_transport_get_device_name(char *name, size_t capacity);

esp_err_t ble_transport_scan_start(void);

esp_err_t ble_transport_scan_get_results(
    ble_transport_scan_result_t *results,
    size_t capacity,
    size_t *count,
    bool *scanning);

typedef void (*ble_transport_rx_callback_t)(const uint8_t *data, size_t length);

esp_err_t ble_transport_set_rx_callback(ble_transport_rx_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif