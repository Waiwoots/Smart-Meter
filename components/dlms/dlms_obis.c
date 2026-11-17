#include "dlms_obis.h"

#include <stddef.h>



static const dlms_object_info_t g_dlms_object_table[] =
{
    /*======================================================
     * Identity
     *=====================================================*/

    {
        .object_id = DLMS_OBJECT_SERIAL_NUMBER,
        .category  = DLMS_OBJECT_CATEGORY_IDENTITY,
        .obis      = {0,0,96,1,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_MANUFACTURER,
        .category  = DLMS_OBJECT_CATEGORY_IDENTITY,
        .obis      = {0,0,96,1,1,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

        {
        .object_id = DLMS_OBJECT_MODEL,
        .category  = DLMS_OBJECT_CATEGORY_IDENTITY,
        .obis      = {0,0,96,1,2,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_FIRMWARE_VERSION,
        .category  = DLMS_OBJECT_CATEGORY_IDENTITY,
        .obis      = {1,0,0,2,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    /*======================================================
     * Instantaneous
     *=====================================================*/

    {
        .object_id = DLMS_OBJECT_VOLTAGE_L1,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,32,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_VOLTAGE_L2,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,52,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_VOLTAGE_L3,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,72,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

        {
        .object_id = DLMS_OBJECT_CURRENT_L1,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,31,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_CURRENT_L2,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,51,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_CURRENT_L3,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,71,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_FREQUENCY,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,14,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_POWER_FACTOR_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,13,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_POWER_FACTOR_L1,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,33,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_POWER_FACTOR_L2,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,53,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_POWER_FACTOR_L3,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,73,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_ACTIVE_POWER_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,1,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_ACTIVE_POWER_L1,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,21,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_ACTIVE_POWER_L2,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,41,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_ACTIVE_POWER_L3,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,61,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    /*======================================================
     * Energy
     *=====================================================*/

    {
        .object_id = DLMS_OBJECT_IMPORT_ACTIVE_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,1,8,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_IMPORT_ACTIVE_T1,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,1,8,1,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_IMPORT_ACTIVE_T2,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,1,8,2,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_IMPORT_ACTIVE_T3,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,1,8,3,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_IMPORT_ACTIVE_T4,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,1,8,4,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,2,8,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_T1,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,2,8,1,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_T2,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,2,8,2,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_T3,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,2,8,3,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_T4,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,2,8,4,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
    .object_id = DLMS_OBJECT_REACTIVE_POWER_TOTAL,
    .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
    .obis      = {1, 0, 3, 7, 0, 255},
    .attribute = 2,
    .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_APPARENT_POWER_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1, 0, 9, 7, 0, 255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },


/*======================================================
 * Demand
*=====================================================*/
    {
        .object_id = DLMS_OBJECT_MAXIMUM_DEMAND,
        .category  = DLMS_OBJECT_CATEGORY_DEMAND,
        .obis      = {1,0,1,6,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_ACTIVE_POWER,
        .category  = DLMS_OBJECT_CATEGORY_INSTANTANEOUS,
        .obis      = {1,0,2,7,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_IMPORT_REACTIVE_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,3,8,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EXPORT_REACTIVE_TOTAL,
        .category  = DLMS_OBJECT_CATEGORY_ENERGY,
        .obis      = {1,0,4,8,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_CLOCK,
        .category  = DLMS_OBJECT_CATEGORY_CLOCK,
        .obis      = {0,0,1,0,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    },

    {
        .object_id = DLMS_OBJECT_EVENT_LOG,
        .category  = DLMS_OBJECT_CATEGORY_EVENT,
        .obis      = {0,0,99,98,0,255},
        .attribute = 2,
        .access    = DLMS_ACCESS_READ
    }
};

    /* ... */


#define DLMS_OBJECT_TABLE_SIZE \
    (sizeof(g_dlms_object_table) / sizeof(g_dlms_object_table[0]))

//         _Static_assert(
//     DLMS_OBJECT_TABLE_SIZE == DLMS_OBJECT_COUNT,
//     "DLMS Object Table mismatch"
// );


// const dlms_object_info_t *dlms_obis_get(dlms_object_id_t object_id)
// {
//     if (object_id >= DLMS_OBJECT_COUNT)
//     {
//         return NULL;
//     }

//     return &g_dlms_object_table[object_id];
// }


// size_t dlms_obis_count(void)
// {
//     return DLMS_OBJECT_TABLE_SIZE;

// }
const dlms_object_info_t *dlms_obis_get(dlms_object_id_t object_id)
{
    for (size_t i = 0; i < DLMS_OBJECT_TABLE_SIZE; ++i) {
        if (g_dlms_object_table[i].object_id == object_id) {
            return &g_dlms_object_table[i];
        }
    }

    return NULL;
}

// const dlms_object_info_t *
// dlms_obis_find_by_obis(const uint8_t obis[DLMS_OBIS_CODE_LENGTH]);


