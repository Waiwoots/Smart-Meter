#ifndef DLMS_CLIENT_H
#define DLMS_CLIENT_H

#include <stdbool.h>

#include "esp_err.h"

#include "dlms_client_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 * Public Functions
 *=========================================================*/

/* Initialize DLMS Client */
esp_err_t dlms_client_init(const dlms_client_config_t *config);

/* Start DLMS Client */
esp_err_t dlms_client_start(void);

/* Stop DLMS Client */
esp_err_t dlms_client_stop(void);

/* Connect to Meter */
esp_err_t dlms_client_connect(void);

/* Disconnect from Meter */
esp_err_t dlms_client_disconnect(void);

/* Read Meter Identity */
esp_err_t dlms_client_read_identity(void);

/* Read Instantaneous Values */
esp_err_t dlms_client_read_instantaneous(void);

/* Read Energy Values */
esp_err_t dlms_client_read_energy(void);

/* Read Demand Values */
esp_err_t dlms_client_read_demand(void);

/* Read Load Profile */
esp_err_t dlms_client_read_profile(void);

/* Read Event Log */
esp_err_t dlms_client_read_event_log(void);

esp_err_t dlms_client_refresh(void);

esp_err_t dlms_client_sync(void);

/* Client State */
dlms_client_state_t dlms_client_get_state(void);

/* Session State */
dlms_session_state_t dlms_client_get_session_state(void);

#ifdef __cplusplus
}
#endif

#endif