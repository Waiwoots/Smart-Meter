#ifndef METER_TYPES_H
#define METER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Meter State
 *=========================================================*/

typedef enum
{
    METER_STATE_DISCONNECTED = 0,

    METER_STATE_CONNECTING,

    METER_STATE_AUTHENTICATING,

    METER_STATE_READING,

    METER_STATE_READY,

    METER_STATE_ERROR

} meter_state_t;


/*==========================================================
 * Meter Information
 *=========================================================*/

typedef struct
{
    char serial_number[32];

    char manufacturer[32];

    char model[32];

    char firmware[32];

    uint8_t phase_count;

} meter_info_t;


/*==========================================================
 * Meter Status
 *=========================================================*/

typedef struct
{
    meter_state_t state;

    uint32_t last_update;

    int32_t last_error;

} meter_status_t;


/*==========================================================
 * Electrical Measurements
 *=========================================================*/

typedef struct
{
    float voltage[3];

    float current[3];

    float active_power[3];

    float reactive_power[3];

    float apparent_power[3];

    float power_factor[3];

    float frequency;

} meter_measurement_t;


/*==========================================================
 * Energy
 *=========================================================*/
//  typedef enum
// {
//     METER_ENERGY_WH,
//     METER_ENERGY_KWH,
//     METER_ENERGY_MWH

// } meter_energy_unit_t;
 

 /**
 * Tariff Index
 *
 * [0] = Total
 * [1] = Tariff 1
 * [2] = Tariff 2
 * [3] = Tariff 3
 */
#define METER_MAX_PHASE      3
#define METER_MAX_TARIFF     4

typedef struct
{
    // meter_energy_unit_t unit;

    double active_import[METER_MAX_TARIFF];

    double active_export[METER_MAX_TARIFF];

    // double reactive_import[METER_MAX_TARIFF];

    // double reactive_export[METER_MAX_TARIFF];


} meter_energy_t;


/*==========================================================
 * Main Object
 *=========================================================*/

typedef struct
{
    meter_info_t info;

    meter_status_t status;

    meter_measurement_t measurement;

    meter_energy_t energy;

} meter_t;

#ifdef __cplusplus
}
#endif

#endif