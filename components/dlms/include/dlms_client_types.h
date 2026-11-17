#ifndef DLMS_CLIENT_TYPES_H
#define DLMS_CLIENT_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Constant Definitions
 *=========================================================*/

#define DLMS_CLIENT_DEFAULT_RESPONSE_TIMEOUT_MS   5000

#define DLMS_CLIENT_DEFAULT_MAX_RETRY             3

/*==========================================================
 * Client State
 *=========================================================*/

typedef enum
{
    DLMS_CLIENT_STATE_UNINITIALIZED = 0,

    DLMS_CLIENT_STATE_INITIALIZED,

    DLMS_CLIENT_STATE_RUNNING,

    DLMS_CLIENT_STATE_STOPPED

} dlms_client_state_t;

/*==========================================================
 * Session State
 *=========================================================*/

typedef enum
{
    DLMS_SESSION_STATE_DISCONNECTED = 0,

    DLMS_SESSION_STATE_CONNECTING,

    DLMS_SESSION_STATE_ASSOCIATING,

    DLMS_SESSION_STATE_ASSOCIATED,
    
    DLMS_SESSION_STATE_BUSY,

    DLMS_SESSION_STATE_DISCONNECTING

} dlms_session_state_t;

/*==========================================================
 * Configuration
 *=========================================================*/

typedef struct
{
    uint32_t response_timeout_ms;

    uint8_t max_retry;

} dlms_client_config_t;

/*==========================================================
 * Default Configuration
 *=========================================================*/

#define DLMS_CLIENT_CONFIG_DEFAULT()               \
{                                                  \
    .response_timeout_ms =                         \
        DLMS_CLIENT_DEFAULT_RESPONSE_TIMEOUT_MS,   \
                                                   \
    .max_retry =                                  \
        DLMS_CLIENT_DEFAULT_MAX_RETRY              \
}

#ifdef __cplusplus
}
#endif

#endif