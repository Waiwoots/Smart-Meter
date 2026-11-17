#ifndef DLMS_HDLC_H
#define DLMS_HDLC_H

#pragma once
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "dlms_hdlc_types.h"

#ifdef __cplusplus
extern "C" {
#endif


size_t dlms_build_hdlc(
    uint8_t control,
    const uint8_t *payload,
    size_t payload_length,
    uint8_t *frame);

uint16_t dlms_crc16(
    const uint8_t *data,
    size_t length);    
/**
 * Initialize the HDLC parser.
 */
esp_err_t dlms_hdlc_init(void);

/**
 * Parse one complete HDLC frame.
 *
 * Expected format:
 *   7E | HDLC header | information | HCS/FCS | 7E
 *
 * The parser does not modify the supplied buffer.
 */
dlms_hdlc_parse_result_t dlms_hdlc_parse_frame(
    const uint8_t *buffer,
    size_t length,
    dlms_hdlc_frame_t *frame);

/**
 * Print a parsed HDLC frame to the ESP-IDF log.
 */
void dlms_hdlc_dump_frame(const dlms_hdlc_frame_t *frame);

/**
 * Convert parser result to a readable string.
 */
const char *dlms_hdlc_parse_result_to_string(
    dlms_hdlc_parse_result_t result);


size_t dlms_hdlc_build_rr(
    const dlms_hdlc_frame_t *rx_frame,
    uint8_t *buffer,
    size_t capacity);

    
#ifdef __cplusplus
}
#endif

#endif /* DLMS_HDLC_H */
