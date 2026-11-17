#ifndef DLMS_HDLC_TYPES_H
#define DLMS_HDLC_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DLMS_HDLC_FLAG 0x7E

typedef enum
{
    DLMS_HDLC_PARSE_OK = 0,
    DLMS_HDLC_PARSE_INVALID_ARGUMENT,
    DLMS_HDLC_PARSE_TOO_SHORT,
    DLMS_HDLC_PARSE_INVALID_FLAG,
    DLMS_HDLC_PARSE_INVALID_LENGTH,
    DLMS_HDLC_PARSE_INVALID_FRAME
} dlms_hdlc_parse_result_t;

/*
 * Parsed HDLC frame.
 *
 * The address fields are kept as raw HDLC address values for now.
 * This avoids making assumptions about the meter's address encoding
 * until the actual SNRM/UA exchange has been verified.
 */
typedef struct
{
    const uint8_t *raw_frame;
    size_t raw_length;

    uint16_t frame_length_field;

    uint32_t destination_address;
    size_t destination_address_length;

    uint32_t source_address;
    size_t source_address_length;

    uint8_t control;

    bool has_information;

    const uint8_t *information;
    size_t information_length;

    uint16_t hcs;
    bool has_hcs;

    uint16_t fcs;
    bool has_fcs;

} dlms_hdlc_frame_t;

#ifdef __cplusplus
}
#endif

#endif /* DLMS_HDLC_TYPES_H */
