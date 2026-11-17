#ifndef BLE_SERVICE_UUID_H
#define BLE_SERVICE_UUID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *ble_service_name(uint16_t uuid);

#ifdef __cplusplus
}
#endif

#endif