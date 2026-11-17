#ifndef BLE_COMPANY_H
#define BLE_COMPANY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "ble_company.h"

const char *ble_company_name(uint16_t id);
const char *company_name(uint16_t company);

#ifdef __cplusplus
}
#endif

#endif