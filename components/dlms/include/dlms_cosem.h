#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


#include "esp_err.h"




/*
 * COSEM Interface Class
 */
#define DLMS_CLASS_ASSOCIATION_LN    15U

/*
 * Association LN
 *
 * 0.0.40.0.0.255
 *
 * Attribute 2 = Object List
 */
#define DLMS_ASSOCIATION_LN_OBIS \
    { 0x00, 0x00, 0x28, 0x00, 0x00, 0xFF }

#define DLMS_ASSOCIATION_LN_ATTRIBUTE_OBJECT_LIST  2U

/*
 * Send GET request for Association View / Object List.
 */
esp_err_t dlms_cosem_get_association_view(void);

/*
 * Handle a received COSEM APDU.
 *
 * This is only the first-stage parser.
 * It will identify:
 *   - GET-Response
 *   - DataAccessResult
 *   - Data
 */
esp_err_t dlms_cosem_parse_get_response(
    const uint8_t *data,
    size_t length);

esp_err_t dlms_cosem_start_get_data_block(
    const uint8_t *data,
    size_t length);

esp_err_t dlms_cosem_append_get_data(
    const uint8_t *data,
    size_t length);

bool dlms_cosem_is_waiting_hdlc_segment(void);
bool dlms_cosem_is_association_view_complete(void);    

bool dlms_cosem_is_get_data_complete(void);
bool dlms_cosem_is_current_block_complete(void);
bool dlms_cosem_is_transfer_complete(void);
bool dlms_cosem_is_last_block(void);
uint32_t dlms_cosem_get_current_block_number(void);

esp_err_t dlms_cosem_get_next_block(
    uint8_t invoke_id,
    uint32_t block_number);


//  #define DLMS_COSEM_MAX_OBJECTS 16U
// #define DLMS_COSEM_MAX_OBJECTS 32
#define DLMS_COSEM_MAX_OBJECTS 256U

typedef enum {
    DLMS_COSEM_PARSE_OK = 0,
    DLMS_COSEM_PARSE_INCOMPLETE,
    DLMS_COSEM_PARSE_TOO_MANY_OBJECTS,
    DLMS_COSEM_PARSE_INVALID_AXDR
} dlms_cosem_parse_result_t;


typedef struct {
    uint16_t class_id;
    uint8_t logical_name[6];
    uint8_t version;
} dlms_cosem_object_t;

// esp_err_t dlms_cosem_parse_association_view(void);
esp_err_t dlms_cosem_parse_association_view(
    const uint8_t *data,
    size_t length);

size_t dlms_cosem_get_object_count(void);

esp_err_t dlms_cosem_get_object(
    size_t index,
    dlms_cosem_object_t *object
);