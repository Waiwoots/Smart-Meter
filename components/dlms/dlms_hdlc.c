#include "dlms_hdlc.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"

static const char *TAG = "DLMS_HDLC";

/*
 * HDLC address fields use one or more bytes.
 * The least significant bit set to 1 marks the final address byte.
 */
static bool hdlc_find_address_length(
    const uint8_t *buffer,
    size_t available,
    size_t *address_length)
{
    if (buffer == NULL || address_length == NULL || available == 0) {
        return false;
    }

    for (size_t i = 0; i < available; ++i) {
        if ((buffer[i] & 0x01U) != 0U) {
            *address_length = i + 1;
            return true;
        }
    }

    return false;
}

static uint32_t hdlc_decode_address(
    const uint8_t *buffer,
    size_t length)
{
    uint32_t value = 0;

    for (size_t i = 0; i < length; ++i) {
        value = (value << 8) | buffer[i];
    }

    return value;
}

// static uint16_t read_be16(const uint8_t *p)
// {
//     return ((uint16_t)p[0] << 8) | p[1];
// }
static uint16_t read_le16(const uint8_t *p)
{
    return ((uint16_t)p[1] << 8) | p[0];
}


esp_err_t dlms_hdlc_init(void)
{
    ESP_LOGI(TAG, "HDLC parser initialized");
    return ESP_OK;
}

dlms_hdlc_parse_result_t dlms_hdlc_parse_frame(
    const uint8_t *buffer,
    size_t length,
    dlms_hdlc_frame_t *frame)
{
    if (buffer == NULL || frame == NULL) {
        return DLMS_HDLC_PARSE_INVALID_ARGUMENT;
    }

    memset(frame, 0, sizeof(*frame));

    if (length < 10) {
        return DLMS_HDLC_PARSE_TOO_SHORT;
    }

    if (buffer[0] != DLMS_HDLC_FLAG ||
        buffer[length - 1] != DLMS_HDLC_FLAG) {
        return DLMS_HDLC_PARSE_INVALID_FLAG;
    }

    /*
     * HDLC Frame Format:
     *
     *   Flag
     *   Format + Length (2 bytes)
     *   Destination Address
     *   Source Address
     *   Control
     *   HCS
     *   Information (optional)
     *   FCS
     *   Flag
     *
     * The exact presence/meaning of HCS is determined from the
     * frame format. This first parser intentionally exposes the
     * bytes without attempting to validate CRC yet.
     */

    frame->raw_frame = buffer;
    frame->raw_length = length;

    // frame->frame_length_field =
    //     ((uint16_t)(buffer[1] & 0x07U) << 8) | buffer[2];
    frame->frame_length_field =
    ((uint16_t)(buffer[1] & 0x07U) << 8) |
    (uint16_t)buffer[2];
    /*
     * The HDLC length field is the number of octets following
     * the format/length field and before the final flag.
     *
     * We accept the received frame first and report a mismatch
     * rather than making assumptions about vendor-specific framing.
     */
    size_t index = 3;

    size_t remaining = length - 1 - index;

    size_t destination_length = 0;
    if (!hdlc_find_address_length(
            &buffer[index],
            remaining,
            &destination_length)) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    frame->destination_address_length = destination_length;
    frame->destination_address =
        hdlc_decode_address(&buffer[index], destination_length);

    index += destination_length;

    remaining = length - 1 - index;

    size_t source_length = 0;
    if (!hdlc_find_address_length(
            &buffer[index],
            remaining,
            &source_length)) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    frame->source_address_length = source_length;
    frame->source_address =
        hdlc_decode_address(&buffer[index], source_length);

    index += source_length;

    /*
     * Control field.
     */
    if (index >= length - 1) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    frame->control = buffer[index++];

    /*
     * A complete HDLC header contains HCS.
     * We expose it here. CRC validation will be added after
     * the real SNRM/UA frame format has been verified.
     */
    if (index + 2 > length - 1) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    // frame->hcs = read_be16(&buffer[index]);
    frame->hcs = read_le16(&buffer[index]);
    frame->has_hcs = true;
    index += 2;

    /*
     * Minimum trailer is FCS(2) + final flag(1).
     */
    if (index + 3 > length) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    size_t information_end = length - 3;

    if (information_end < index) {
        return DLMS_HDLC_PARSE_INVALID_FRAME;
    }

    frame->information = &buffer[index];
    frame->information_length = information_end - index;
    frame->has_information = frame->information_length > 0;

    // frame->fcs = read_be16(&buffer[information_end]);
    frame->fcs = read_le16(&buffer[information_end]);
    frame->has_fcs = true;

    return DLMS_HDLC_PARSE_OK;
}

static uint16_t dlms_hdlc_rr_crc16(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];

        for (unsigned int bit = 0; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return (uint16_t)~crc;
}


size_t dlms_hdlc_build_rr(
    const dlms_hdlc_frame_t *rx_frame,
    uint8_t *buffer,
    size_t capacity)
{
    if (rx_frame == NULL ||
        buffer == NULL ||
        capacity < 9U) {
        return 0U;
    }

    /*
     * Current meter uses one-byte HDLC addresses.
     */
    if (rx_frame->destination_address_length != 1U ||
        rx_frame->source_address_length != 1U) {
        return 0U;
    }

    /*
     * Received I-frame:
     *
     *   N(S) = bits 1..3
     *   P/F  = bit 4
     *   N(R) = bits 5..7
     *
     * RR acknowledges all I-frames through N(R)-1
     * and announces the next expected receive sequence N(R).
     */
    uint8_t receive_sequence =
        (uint8_t)((rx_frame->control >> 5U) & 0x07U);

    uint8_t poll_final =
        (uint8_t)(rx_frame->control & 0x10U);

    /*
     * S-frame:
     *
     * bit 0     = 1
     * bits 1..3 = 000  -> RR
     * bit 4     = P/F
     * bits 5..7 = N(R)
     */
    uint8_t rr_control =
        (uint8_t)(
            0x01U |
            ((receive_sequence & 0x07U) << 5U) |
            poll_final);

    /*
     * For the current one-byte-address meter:
     *
     * TX first address  = received source
     * TX second address = received destination
     *
     * Example:
     * RX = 21 03
     * TX = 03 21
     */
    buffer[0] = 0x7EU;
    buffer[1] = 0xA0U;
    buffer[2] = 0x07U;
    buffer[3] = (uint8_t)rx_frame->source_address;
    buffer[4] = (uint8_t)rx_frame->destination_address;
    buffer[5] = rr_control;

    /*
     * HCS covers:
     *
     * A0 07 address address control
     */
    uint16_t hcs =
        dlms_hdlc_rr_crc16(&buffer[1], 5U);

    buffer[6] = (uint8_t)(hcs & 0xFFU);
    buffer[7] = (uint8_t)(hcs >> 8U);

    /*
     * S-frame has no information field and therefore
     * no information-field FCS.
     */
    buffer[8] = 0x7EU;

    return 9U;
}

void dlms_hdlc_dump_frame(const dlms_hdlc_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    ESP_LOGI(TAG, "HDLC Frame");
    ESP_LOGI(TAG, "  Raw length       : %u",
             (unsigned)frame->raw_length);
    ESP_LOGI(TAG, "  Length field     : %u",
             (unsigned)frame->frame_length_field);
    ESP_LOGI(TAG, "  Destination      : 0x%08" PRIX32 " (%u bytes)",
             frame->destination_address,
             (unsigned)frame->destination_address_length);
    ESP_LOGI(TAG, "  Source           : 0x%08" PRIX32 " (%u bytes)",
             frame->source_address,
             (unsigned)frame->source_address_length);
    ESP_LOGI(TAG, "  Control          : 0x%02X",
             frame->control);
    ESP_LOGI(TAG, "  HCS              : 0x%04X",
             frame->hcs);
    ESP_LOGI(TAG, "  Information len   : %u",
             (unsigned)frame->information_length);
    ESP_LOGI(TAG, "  FCS              : 0x%04X",
             frame->fcs);

    if (frame->information != NULL && frame->information_length > 0) {
        ESP_LOG_BUFFER_HEXDUMP(
            TAG,
            frame->information,
            frame->information_length,
            ESP_LOG_INFO);
    }
}

const char *dlms_hdlc_parse_result_to_string(
    dlms_hdlc_parse_result_t result)
{
    switch (result) {
        case DLMS_HDLC_PARSE_OK:
            return "OK";

        case DLMS_HDLC_PARSE_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case DLMS_HDLC_PARSE_TOO_SHORT:
            return "TOO_SHORT";

        case DLMS_HDLC_PARSE_INVALID_FLAG:
            return "INVALID_FLAG";

        case DLMS_HDLC_PARSE_INVALID_LENGTH:
            return "INVALID_LENGTH";

        case DLMS_HDLC_PARSE_INVALID_FRAME:
            return "INVALID_FRAME";

        default:
            return "UNKNOWN";
    }
}

 size_t dlms_build_hdlc(
     uint8_t control,
     const uint8_t *payload,
     size_t payload_length,
      uint8_t *frame)
{
    size_t index = 0;
    // uint16_t length = (uint16_t)(2U + 1U + 2U + payload_length + 2U);
    uint16_t length =
     (uint16_t)(7U + 1U + 1U + payload_length);

    frame[index++] = 0x7E;
    frame[index++] = 0xA0;
    frame[index++] = (uint8_t)length;

    frame[index++] = 0x03;
    frame[index++] = 0x21;
    frame[index++] = control;

    // uint16_t hcs = dlms_crc16(&frame[1], index - 1U);
    // frame[index++] = (uint8_t)hcs;
    // frame[index++] = (uint8_t)(hcs >> 8U);

    uint16_t hcs = dlms_crc16(&frame[1], index - 1U);

    
    frame[index++] = (uint8_t)(hcs & 0xFFU);
    frame[index++] = (uint8_t)(hcs >> 8U);

    memcpy(&frame[index], payload, payload_length);
    index += payload_length;

    uint16_t fcs = dlms_crc16(&frame[1], index - 1U);


    
    frame[index++] = (uint8_t)(fcs & 0xFFU);
    frame[index++] = (uint8_t)(fcs >> 8U);

    frame[index++] = 0x7E;
    return index;
}


 uint16_t dlms_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? (crc >> 1U) ^ 0x8408U : crc >> 1U;
        }
    }
    return (uint16_t)~crc;
}
