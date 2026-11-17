#ifndef DLMS_CLIENT_EVENT_H
#define DLMS_CLIENT_EVENT_H

#include "esp_event.h"
#include "esp_err.h"

#include "dlms_client_types.h"
#include "meter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Event Base
 *=========================================================*/

ESP_EVENT_DECLARE_BASE(DLMS_CLIENT_EVENT);

/*==========================================================
 * Event IDs
 *=========================================================*/

typedef enum
{
    DLMS_CLIENT_EVENT_STARTED = 0,

    DLMS_CLIENT_EVENT_STOPPED,

    DLMS_CLIENT_EVENT_CONNECTED,

    DLMS_CLIENT_EVENT_DISCONNECTED,

    DLMS_CLIENT_EVENT_METER_UPDATED,

    DLMS_CLIENT_EVENT_ERROR

} dlms_client_event_id_t;

/*==========================================================
 * Event Payloads
 *=========================================================*/

typedef struct
{
    dlms_session_state_t session_state;

} dlms_client_connected_event_t;

typedef struct
{
    esp_err_t reason;

} dlms_client_disconnected_event_t;


typedef enum
{
    DLMS_UPDATE_IDENTITY = 0,

    DLMS_UPDATE_INSTANTANEOUS,

    DLMS_UPDATE_ENERGY,

    DLMS_UPDATE_DEMAND,

    DLMS_UPDATE_PROFILE,

    DLMS_UPDATE_EVENT

} dlms_update_type_t;


typedef struct
{
     dlms_update_type_t update_type;

    const meter_t *meter;

} dlms_client_meter_updated_event_t;


typedef struct
{
    esp_err_t error;

} dlms_client_error_event_t;

#ifdef __cplusplus
}
#endif

#endif