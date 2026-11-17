#include "ble_company.h"

const char *ble_company_name(uint16_t id)
{
    switch (id)
    {
        case 0x0006: return "Microsoft";
        case 0x004C: return "Apple";
        case 0x0075: return "Samsung";
        case 0x0131: return "Xiaomi";
        case 0x0157: return "Espressif";
        case 0x000F: return "Broadcom";
        case 0x00E0: return "Google";
        default:     return "Unknown";
    }
}
