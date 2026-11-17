#ifndef DLMS_CODEC_H
#define DLMS_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "dlms_codec_types.h"
#include "dlms_codec.h"


#ifdef __cplusplus
extern "C" {    
#endif

/*==========================================================
 * Initialization
 *=========================================================*/

esp_err_t dlms_codec_init(void);

/*==========================================================
 * Request Encoder
 *=========================================================*/

esp_err_t dlms_codec_build_request(
    const dlms_request_t *request,
    uint8_t *buffer,
    size_t buffer_size,
    size_t *encoded_size);

/*==========================================================
 * Response Decoder
 *=========================================================*/

esp_err_t dlms_codec_parse_response(
    const uint8_t *buffer,
    size_t length,
    dlms_response_t *response);

/*==========================================================
 * Data Decoder
 *=========================================================*/

esp_err_t dlms_codec_decode_data(
    const uint8_t *buffer,
    size_t length,
    dlms_data_value_t *value);

#ifdef __cplusplus
}
#endif

#endif