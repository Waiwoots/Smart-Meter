#ifndef BLE_TRANSPORT_EVENT_H
#define BLE_TRANSPORT_EVENT_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"

#include "ble_transport_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Event Base
 *=========================================================*/

ESP_EVENT_DECLARE_BASE(BLE_TRANSPORT_EVENT);

/*==========================================================
 * Event IDs
 *=========================================================*/

typedef enum
{
    BLE_TRANSPORT_EVENT_STARTED = 0,

    BLE_TRANSPORT_EVENT_SCAN_COMPLETED,

    BLE_TRANSPORT_EVENT_CONNECTED,

    BLE_TRANSPORT_EVENT_READY,

    BLE_TRANSPORT_EVENT_DISCONNECTED,

    BLE_TRANSPORT_EVENT_DATA_RECEIVED,

    BLE_TRANSPORT_EVENT_ERROR

} ble_transport_event_id_t;

/*==========================================================
 * Event Payloads
 *=========================================================*/

/* Connected */

typedef struct
{
    uint8_t peer_address[BLE_TRANSPORT_ADDRESS_LEN];

} ble_transport_connected_event_t;

/* Disconnected */

typedef struct
{
    esp_err_t reason;

} ble_transport_disconnected_event_t;

/* Received Data */

typedef struct
{
    const uint8_t *data;

    size_t length;

} ble_transport_rx_event_t;

/* Error */

typedef struct
{
    esp_err_t error;

} ble_transport_error_event_t;

#ifdef __cplusplus
}
#endif

#endif