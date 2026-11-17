#ifndef BLE_TRANSPORT_TYPES_H
#define BLE_TRANSPORT_TYPES_H

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

 /*==========================================================
 * Constant Definitions
 *=========================================================*/

 #define BLE_TRANSPORT_ADDRESS_LEN                  6

 #define BLE_TRANSPORT_DEFAULT_MTU                 247
 #define BLE_TRANSPORT_DEFAULT_SCAN_TIMEOUT_MS    5000
 #define BLE_TRANSPORT_DEFAULT_CONNECT_TIMEOUT_MS 10000


/*==========================================================
 * Driver State
 *=========================================================*/

typedef enum
{
    BLE_TRANSPORT_STATE_UNINITIALIZED = 0,
    BLE_TRANSPORT_STATE_INITIALIZED,
    BLE_TRANSPORT_STATE_RUNNING,
    BLE_TRANSPORT_STATE_STOPPED

} ble_transport_state_t;

/*==========================================================
 * Link State
 *=========================================================*/

typedef enum
{
    BLE_LINK_STATE_DISCONNECTED = 0,
    BLE_LINK_STATE_SCANNING,
    BLE_LINK_STATE_CONNECTING,
    BLE_LINK_STATE_CONNECTED,

    BLE_LINK_STATE_MTU_EXCHANGE,

    BLE_LINK_STATE_DISCOVERING_SERVICE,

    BLE_LINK_STATE_DISCOVERING_CHARACTERISTIC,

    BLE_LINK_STATE_ENABLING_NOTIFY,

    BLE_LINK_STATE_READY

} ble_link_state_t;

/*==========================================================
 * BLE GATT Handle
 *=========================================================*/

typedef struct
{
    uint16_t service_start_handle;
    uint16_t service_end_handle;

    uint16_t write_char_handle;
    uint16_t notify_char_handle;

    uint16_t notify_cccd_handle;

    bool discovered;

} ble_transport_gatt_t;

/*==========================================================
 * BLE Transport Configuration
 *=========================================================*/


typedef struct
{
   
   uint8_t  peer_address[BLE_TRANSPORT_ADDRESS_LEN];

    uint16_t preferred_mtu;

    uint32_t scan_timeout_ms;

    uint32_t connect_timeout_ms;

    char device_name[32];

    ble_transport_gatt_t gatt;

   

} ble_transport_config_t;

/*==========================================================
 * BLE Transport Configuration
 *=========================================================*/


#define BLE_TRANSPORT_CONFIG_DEFAULT()               \
{                                                    \
    .peer_address       = {0},                       \
    .preferred_mtu      = BLE_TRANSPORT_DEFAULT_MTU, \
    .scan_timeout_ms    = BLE_TRANSPORT_DEFAULT_SCAN_TIMEOUT_MS, \
    .connect_timeout_ms = BLE_TRANSPORT_DEFAULT_CONNECT_TIMEOUT_MS, \
    .device_name        = "",                        \
    .gatt = {                                        \
        .service_start_handle = 0,                   \
        .service_end_handle   = 0,                   \
        .write_char_handle    = 0,                   \
        .notify_char_handle   = 0,                   \
        .notify_cccd_handle   = 0,                   \
        .discovered           = false                \
    }                                                \
}


#ifdef __cplusplus
}
#endif

#endif

