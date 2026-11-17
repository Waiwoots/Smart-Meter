#ifndef DLMS_OBIS_H
#define DLMS_OBIS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Constant Definitions
 *=========================================================*/

/*==========================================================
 * Object Category
 *=========================================================*/
typedef enum
{
    DLMS_OBJECT_CATEGORY_IDENTITY = 0,

    DLMS_OBJECT_CATEGORY_INSTANTANEOUS,

    DLMS_OBJECT_CATEGORY_ENERGY,

    DLMS_OBJECT_CATEGORY_DEMAND,

    DLMS_OBJECT_CATEGORY_PROFILE,

    DLMS_OBJECT_CATEGORY_EVENT,

    DLMS_OBJECT_CATEGORY_CLOCK,

    DLMS_OBJECT_CATEGORY_CONTROL,

    DLMS_OBJECT_CATEGORY_DIAGNOSTIC,

    DLMS_OBJECT_CATEGORY_MANUFACTURER

   

} dlms_object_category_t;
/*==========================================================
 * Object ID
 *=========================================================*/
typedef enum
{
    /*======================================================
     * Identity
     *=====================================================*/

    DLMS_OBJECT_SERIAL_NUMBER = 0,

    DLMS_OBJECT_MANUFACTURER,

    DLMS_OBJECT_MODEL,

    DLMS_OBJECT_FIRMWARE_VERSION,

    /*======================================================
     * Instantaneous
     *=====================================================*/

    DLMS_OBJECT_VOLTAGE_L1,

    DLMS_OBJECT_VOLTAGE_L2,

    DLMS_OBJECT_VOLTAGE_L3,

    DLMS_OBJECT_CURRENT_L1,

    DLMS_OBJECT_CURRENT_L2,

    DLMS_OBJECT_CURRENT_L3,

    DLMS_OBJECT_FREQUENCY,

    DLMS_OBJECT_POWER_FACTOR_TOTAL,

    DLMS_OBJECT_POWER_FACTOR_L1,

    DLMS_OBJECT_POWER_FACTOR_L2,

    DLMS_OBJECT_POWER_FACTOR_L3,

    DLMS_OBJECT_ACTIVE_POWER_TOTAL,

    DLMS_OBJECT_ACTIVE_POWER_L1,

    DLMS_OBJECT_ACTIVE_POWER_L2,

    DLMS_OBJECT_ACTIVE_POWER_L3,

    DLMS_OBJECT_REACTIVE_POWER_TOTAL,

    DLMS_OBJECT_APPARENT_POWER_TOTAL,

    /*======================================================
     * Energy
     *=====================================================*/

    DLMS_OBJECT_IMPORT_ACTIVE_TOTAL,

    DLMS_OBJECT_IMPORT_ACTIVE_T1,

    DLMS_OBJECT_IMPORT_ACTIVE_T2,

    DLMS_OBJECT_IMPORT_ACTIVE_T3,

    DLMS_OBJECT_IMPORT_ACTIVE_T4,

    DLMS_OBJECT_EXPORT_ACTIVE_TOTAL,

    DLMS_OBJECT_EXPORT_ACTIVE_T1,

    DLMS_OBJECT_EXPORT_ACTIVE_T2,

    DLMS_OBJECT_EXPORT_ACTIVE_T3,

    DLMS_OBJECT_EXPORT_ACTIVE_T4,

    /*======================================================
     * Demand
     *=====================================================*/

    DLMS_OBJECT_MAXIMUM_DEMAND,

    DLMS_OBJECT_EXPORT_ACTIVE_POWER,

    DLMS_OBJECT_IMPORT_REACTIVE_TOTAL,

    DLMS_OBJECT_EXPORT_REACTIVE_TOTAL,

    /*======================================================
     * Clock
     *=====================================================*/

    DLMS_OBJECT_CLOCK,

    /*======================================================
     * Event
     *=====================================================*/

    DLMS_OBJECT_EVENT_LOG,

    DLMS_OBJECT_COUNT

} dlms_object_id_t;
/*==========================================================
 * Object Information
 *=========================================================*/


#define DLMS_OBIS_CODE_LENGTH 6

typedef enum
{
    DLMS_ACCESS_READ = 0,
    DLMS_ACCESS_WRITE,
    DLMS_ACCESS_ACTION

} dlms_access_t;


typedef struct
{
    dlms_object_id_t object_id;

    dlms_object_category_t category;

    uint8_t obis[DLMS_OBIS_CODE_LENGTH];

    uint8_t attribute;

    dlms_access_t access;

} dlms_object_info_t;

/*==========================================================
 * Object Access
 *=========================================================*/

const dlms_object_info_t *dlms_obis_get(dlms_object_id_t object_id);

size_t dlms_obis_count(void);

#ifdef __cplusplus
}
#endif

#endif