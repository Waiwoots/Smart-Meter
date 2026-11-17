#ifndef DLMS_CODEC_TYPES_H
#define DLMS_CODEC_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Constant Definitions
 *=========================================================*/

#define DLMS_CODEC_MAX_FRAME_SIZE      1024

#define DLMS_CODEC_MAX_OBIS_LENGTH     6




typedef struct
{
    uint8_t *data;
    size_t length;
} dlms_response_t;

typedef struct
{
    uint8_t *data;
    size_t length;
} dlms_data_value_t;

/*==========================================================
 * Codec State
 *=========================================================*/

typedef enum
{
    DLMS_CODEC_STATE_IDLE = 0,

    DLMS_CODEC_STATE_ENCODING,

    DLMS_CODEC_STATE_DECODING

} dlms_codec_state_t;

/*==========================================================
 * Request Type
 *=========================================================*/

typedef enum
{
    DLMS_REQUEST_TYPE_GET = 0,
    DLMS_REQUEST_TYPE_ACTION,
    DLMS_REQUEST_TYPE_AARQ

} dlms_request_type_t;


typedef struct
{
    dlms_request_type_t type;

    uint8_t invoke_id;

    uint8_t obis[6];

    uint8_t attribute;

} dlms_request_t;


/*==========================================================
 * Response Result
 *=========================================================*/

typedef enum
{
    DLMS_RESULT_OK = 0,

    DLMS_RESULT_TIMEOUT,

    DLMS_RESULT_REJECTED,

    DLMS_RESULT_INVALID_FRAME,

    DLMS_RESULT_PROTOCOL_ERROR,

    DLMS_RESULT_UNKNOWN

} dlms_result_t;

/*==========================================================
 * Transaction
 *=========================================================*/


typedef struct
{
    dlms_request_type_t type;
    uint32_t timeout_ms;
    uint8_t retry;

} dlms_transaction_t;


/*==========================================================
 * DLMS Object Types
 *=========================================================*/

typedef enum
{
    DLMS_OBJECT_IDENTITY = 0,

    DLMS_OBJECT_INSTANTANEOUS,

    DLMS_OBJECT_ENERGY,

    DLMS_OBJECT_DEMAND,

    DLMS_OBJECT_PROFILE,

    DLMS_OBJECT_EVENT_LOG

} dlms_object_t;

#ifdef __cplusplus
}
#endif

#endif