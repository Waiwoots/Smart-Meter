#include "ble_service_uuid.h"

const char *ble_service_name(uint16_t uuid)
{
    switch (uuid)
    {
        /* Bluetooth SIG standard services */
        case 0x180F: return "Battery Service";
        case 0x180A: return "Device Information";
        case 0x181A: return "Environmental Sensing";
        case 0x181C: return "User Data";
        case 0x1826: return "Fitness Machine";

        /* Google / Fast Pair / Eddystone */
        case 0xFEAA: return "Google Eddystone";
        case 0xFE2C: return "Google Fast Pair";
        case 0xFEF3: return "Google Nearby / Finder";

        /* Xiaomi */
        case 0xFE95: return "Xiaomi BLE";

        /* Matter */
        case 0xFD6F: return "Matter";

        /* Nordic */
        case 0xFE59: return "Nordic DFU";

        /* Microsoft */
        case 0xFEF5: return "Microsoft Swift Pair";

        default:
            return "Unknown";
    }
}