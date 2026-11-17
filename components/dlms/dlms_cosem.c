#include "dlms_cosem.h"

#include <string.h>

#include "esp_log.h"

#include "ble_transport.h"
#include "dlms_hdlc.h"

#include <inttypes.h>
#include "dlms_link.h"

static uint32_t s_association_view_current_block = 0U;



#define DLMS_ASSOCIATION_VIEW_BUFFER_SIZE   16384U



#define DLMS_COSEM_OBJECT_ENTRY_TAG  0x02
#define DLMS_COSEM_OBJECT_FIELD_COUNT 0x04
#define DLMS_COSEM_CLASS_ID_TAG      0x12
#define DLMS_COSEM_VERSION_TAG       0x11
#define DLMS_COSEM_LOGICAL_NAME_TAG  0x09
#define DLMS_COSEM_LOGICAL_NAME_SIZE 6






static uint8_t s_association_view_data[
    DLMS_ASSOCIATION_VIEW_BUFFER_SIZE
];

static size_t s_association_view_data_length = 0U;

static uint32_t s_association_view_expected_block = 1U;

// static uint32_t s_association_view_block_length = 0U;

// static uint32_t s_association_view_received = 0U;

static uint8_t s_association_view_invoke_id = 0U;

static uint8_t s_association_view_last_block = 0U;

static bool s_association_view_waiting_segment = false;

static size_t s_get_data_expected_length = 0U;
static size_t s_get_data_received_length = 0U;
static bool s_get_data_complete = false;

// static dlms_cosem_object_t s_objects[DLMS_COSEM_MAX_OBJECTS];
// static size_t s_object_count = 0U;

static const char *TAG = "DLMS_COSEM";

static bool s_association_view_complete = false;


bool dlms_cosem_is_waiting_hdlc_segment(void)
{
    return s_association_view_waiting_segment;
}

bool dlms_cosem_is_association_view_complete(void)
{
    return s_association_view_complete;
}




/*
 * Association LN Object List
 *
 * GET.request-normal
 *
 * C0 01
 * C1
 * 00
 *
 * Class ID:
 *   00 0F       = class 15
 *
 * Instance ID:
 *   00 00 28 00 00 FF
 *   = 0.0.40.0.0.255
 *
 * Attribute:
 *   02
 *
 * Access selector:
 *   00 = no selective access
 */

static esp_err_t dlms_axdr_decode_length(
    const uint8_t *data,
    size_t length,
    size_t offset,
    size_t *value,
    size_t *encoded_length)
{
    if (data == NULL ||
        value == NULL ||
        encoded_length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (offset >= length) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t first = data[offset];

    /*
     * Short form:
     *
     * 00..7F
     */
    if ((first & 0x80U) == 0U) {
        *value = first;
        *encoded_length = 1U;
        return ESP_OK;
    }

    /*
     * Long form:
     *
     * 81 xx
     * 82 xx xx
     * ...
     */
    size_t octets = first & 0x7FU;

    if (octets == 0U || octets > 4U) {
        ESP_LOGE(
            TAG,
            "Unsupported AXDR length field: 0x%02X",
            first);

        return ESP_ERR_INVALID_RESPONSE;
    }

    if (offset + 1U + octets > length) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t result = 0U;

    for (size_t i = 0U; i < octets; ++i) {
        result = (result << 8U) |
                 data[offset + 1U + i];
    }

    *value = result;
    *encoded_length = 1U + octets;

    return ESP_OK;
}


static esp_err_t dlms_cosem_send_get_next_block(
    uint8_t invoke_id,
    uint32_t block_number)
{
    /*
     * GET.request
     *   C0
     *
     * get-request-for-next-data-block
     *   02
     *
     * invoke-id-and-priority
     *   C1
     *
     * block number
     *   4 bytes, big-endian
     */
    uint8_t apdu[] = {
        0xE6U, 0xE6U, 0x00U,          /* LLC */

        0xC0U, 0x02U, invoke_id,

        (uint8_t)(block_number >> 24U),

        (uint8_t)(block_number >> 16U),

        (uint8_t)(block_number >> 8U),

        (uint8_t)(block_number)
    };

    uint8_t frame[32];

    size_t frame_length = dlms_build_hdlc(
        dlms_link_next_i_control(),
        apdu,
        sizeof(apdu),
        frame);

    if (frame_length == 0U) {
        ESP_LOGE(
            TAG,
            "Failed to build GET-NEXT HDLC frame");

        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "Sending GET-NEXT block=%" PRIu32
        " invoke=0x%02X",
        block_number,
        invoke_id);

    ESP_LOGI(
        TAG,
        "GET-NEXT APDU length=%u HDLC length=%u",
        (unsigned)sizeof(apdu),
        (unsigned)frame_length);

    ESP_LOG_BUFFER_HEX_LEVEL(
        TAG,
        apdu,
        sizeof(apdu),
        ESP_LOG_INFO);

    return ble_transport_write(
        frame,
        frame_length);
}

        

   
esp_err_t dlms_cosem_get_association_view(void)
{
    s_association_view_data_length = 0U;
    s_association_view_expected_block = 1U;
    

    s_get_data_expected_length = 0U;
    s_get_data_received_length = 0U;
    s_get_data_complete = false;
    s_association_view_waiting_segment = false;
    s_association_view_complete = false;
    
    static const uint8_t get_association_apdu[] = {
        0xE6U, 0xE6U, 0x00U,   /* LLC */
        0xC0U, /* GET.request */
        0x01U, /* get-request-normal */
        0xC1U, /* Invoke-Id-And-Priority */

        0x00U, 0x0FU, /* Attribute descriptor *//* Class ID = 15 */

        0x00U, 0x00U, 0x28U, /* Instance ID = 0.0.40.0.0.255 */
        0x00U, 0x00U, 0xFFU,

        0x02U,  /* Attribute ID = 2 */
        0x00U   /* Access selector = 0 */
    };

    uint8_t frame[64];

    size_t frame_length = dlms_build_hdlc(
        dlms_link_next_i_control() ,  //0x32U,
        get_association_apdu,
        sizeof(get_association_apdu),
        frame);

    ESP_LOGI(
        TAG,
        "Request Association View: APDU=%u bytes, HDLC=%u bytes",
        (unsigned)sizeof(get_association_apdu),
        (unsigned)frame_length);

    return ble_transport_write(
        frame,
        frame_length);
}
  


/*
 * First-stage GET response parser.
 *
 * Expected successful response:
 *
 * E6 E7 00
 * C4
 * 01
 * C1
 * 00
 * ...
 *
 * Expected DataAccessError:
 *
 * C4 01 C1 01 xx
 *
 * Example from our previous request:
 *
 * C4 01 C1 01 04 CC F6
 *
 * where:
 *
 * 04 = DataAccessResult::UndefinedObject
 */
esp_err_t dlms_cosem_start_get_data_block(
    const uint8_t *data,
    size_t length)
{








    
    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (length > DLMS_ASSOCIATION_VIEW_BUFFER_SIZE) {
        ESP_LOGE(
            TAG,
            "GET data block too large: %u",
            (unsigned)length);

        return ESP_ERR_NO_MEM;
    }

    memcpy(
        s_association_view_data,
        data,
        length);

    s_get_data_received_length = length;
    s_get_data_expected_length = length;
    s_get_data_complete = true;
    s_association_view_waiting_segment = false;

    return ESP_OK;
}


esp_err_t dlms_cosem_append_get_data(
    const uint8_t *data,
    size_t length)
{
    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_association_view_waiting_segment) {
        ESP_LOGW(
            TAG,
            "No GET data block is waiting for continuation");

        return ESP_ERR_INVALID_STATE;
    }

    if (s_get_data_received_length + length >
        DLMS_ASSOCIATION_VIEW_BUFFER_SIZE) {

        ESP_LOGE(
            TAG,
            "GET data buffer overflow");

        return ESP_ERR_NO_MEM;
    }

    memcpy(
        &s_association_view_data[
            s_get_data_received_length],
        data,
        length);

    s_get_data_received_length += length;

    s_association_view_data_length = s_get_data_received_length;

    ESP_LOGI(
        TAG,
        "GET data appended: +%u bytes, total=%u/%u",
        (unsigned)length,
        (unsigned)s_get_data_received_length,
        (unsigned)s_get_data_expected_length);

    // if (s_get_data_received_length >= s_get_data_expected_length) {
    //     s_get_data_complete = true;
    //     s_association_view_waiting_segment = false;

    //     ESP_LOGI(TAG, "GET data block COMPLETE: %u bytes", (unsigned)s_get_data_received_length);
      
      
                
    //   } 

       if (s_get_data_received_length < s_get_data_expected_length) {
          return ESP_ERR_NOT_FINISHED;   /* รอ segment ถัดไป */
           }

            s_get_data_complete = true;
            s_association_view_waiting_segment = false;

    ESP_LOGI(TAG, "Block %" PRIu32 " COMPLETE (total %u bytes)",
             s_association_view_current_block,
             (unsigned)s_association_view_data_length);

    if (s_association_view_last_block != 0U) {
        s_association_view_complete = true;
        ESP_LOGI(TAG, "Association View transfer COMPLETE");
        return ESP_OK; /* สำคัญ: ไม่ส่ง GET-NEXT หลังบล็อกสุดท้าย */
    }

    s_association_view_expected_block = s_association_view_current_block + 1U;

    return dlms_cosem_send_get_next_block(
        s_association_view_invoke_id,
        s_association_view_current_block);
       
  

}


bool dlms_cosem_is_get_data_complete(void)
{
    return s_get_data_complete;
}



esp_err_t dlms_cosem_parse_get_response(
    const uint8_t *data,
    size_t length)
{
    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(
        TAG,
        "COSEM APDU length = %u",
        (unsigned)length);

    ESP_LOG_BUFFER_HEX_LEVEL(
        TAG,
        data,
        length,
        ESP_LOG_INFO);

    /*
     * GET-Response
     */
    if (data[0] != 0xC4U) {
        ESP_LOGE(
            TAG,
            "Not a GET-Response. First byte = 0x%02X",
            data[0]);

        return ESP_ERR_INVALID_RESPONSE;
    }

    if (length < 4U) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t response_choice = data[1];
    uint8_t invoke_id = data[2];

    ESP_LOGI(
        TAG,
        "GET response: choice=0x%02X invoke=0x%02X",
        response_choice,
        invoke_id);

    /*
     * -------------------------------------------------------
     * GET-Response-Normal
     * -------------------------------------------------------
     *
     * C4 01 invoke result ...
     */
    if (response_choice == 0x01U) {

        ESP_LOGI(
            TAG,
            "GET-Response-Normal");

        if (length < 5U) {
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t result_choice = data[3];

        /*
         * Result choice:
         *
         * 00 = data
         * 01 = data-access-result
         */
        if (result_choice == 0x01U) {
            uint8_t error = data[4];

            ESP_LOGE(
                TAG,
                "GET DataAccessError = 0x%02X",
                error);

            return ESP_ERR_INVALID_RESPONSE;
        }

        if (result_choice != 0x00U) {
            ESP_LOGE(
                TAG,
                "Unknown GET result choice = 0x%02X",
                result_choice);

            return ESP_ERR_INVALID_RESPONSE;
        }

        if (length <= 4U) {
            return ESP_ERR_INVALID_SIZE;
        }

        ESP_LOGI(
            TAG,
            "GET normal response contains DATA");

        ESP_LOG_BUFFER_HEX_LEVEL(
            TAG,
            &data[4],
            length - 4U,
            ESP_LOG_INFO);

        return ESP_OK;
    }

    /*
     * -------------------------------------------------------
     * GET-Response-With-Data-Block
     * -------------------------------------------------------
     *
     * C4 02
     *    invoke-id-and-priority
     *    last-block
     *    block-number
     *    result
     *    block-length
     *    block-data
     */
    if (response_choice == 0x02U) {

        ESP_LOGI(
            TAG,
            "GET-Response-With-Data-Block");

        /*
         * Minimum:
         *
         * C4
         * 02
         * invoke
         * last-block
         * block-number[4]
         * result
         *
         * = 9 bytes
         */
        if (length < 9U) {
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t last_block = data[3];

        uint32_t block_number =
            ((uint32_t)data[4] << 24U) |
            ((uint32_t)data[5] << 16U) |
            ((uint32_t)data[6] << 8U) |
            (uint32_t)data[7];

        uint8_t result_choice = data[8];

        ESP_LOGI(
            TAG,
            "  Invoke ID    : 0x%02X",
            invoke_id);

        ESP_LOGI(
            TAG,
            "  Last block   : %u",
            (unsigned)last_block);

        ESP_LOGI(
            TAG,
            "  Block number : %" PRIu32,
            block_number);

        ESP_LOGI(
            TAG,
            "  Result       : 0x%02X",
            result_choice);

        /*
         * Result choice:
         *
         * 00 = success
         * 01 = data-access-result
         */
        if (result_choice == 0x01U) {

            if (length < 10U) {
                return ESP_ERR_INVALID_SIZE;
            }

            uint8_t error = data[9];

            ESP_LOGE(
                TAG,
                "GET DataAccessError = 0x%02X",
                error);

            return ESP_ERR_INVALID_RESPONSE;
        }

        if (result_choice != 0x00U) {

            ESP_LOGE(
                TAG,
                "Unknown data-block result = 0x%02X",
                result_choice);

            return ESP_ERR_INVALID_RESPONSE;
        }

        /*
         * Decode AXDR length.
         */
        size_t block_length = 0U;
        size_t length_field_size = 0U;

        esp_err_t err = dlms_axdr_decode_length(
            data,
            length,
            9U,
            &block_length,
            &length_field_size);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Invalid AXDR block length");

            return err;
        }

        size_t block_data_offset =
            9U + length_field_size;

        if (block_data_offset > length) {
            return ESP_ERR_INVALID_SIZE;
        }

        size_t block_data_available =
            length - block_data_offset;

        ESP_LOGI(
            TAG,
            "  Declared data : %u bytes",
            (unsigned)block_length);

        ESP_LOGI(
            TAG,
            "  Received data : %u bytes",
            (unsigned)block_data_available);

        /*
         * ---------------------------------------------------
         * CRITICAL:
         * The DLMS block itself must be complete before
         * we can ask for the next DLMS block.
         * ---------------------------------------------------
         */
        // if (block_data_available < block_length) {

        //     ESP_LOGW(
        //         TAG,
        //         "DLMS data block is incomplete!");

        //     ESP_LOGW(
        //         TAG,
        //         "Waiting for remaining HDLC segment. "
        //         "Expected=%u Received=%u",
        //         (unsigned)block_length,
        //         (unsigned)block_data_available);

        //     return ESP_ERR_INVALID_SIZE;
        // }
      

      

    /*
     * -----------------------------------------------
     * เก็บข้อมูลส่วนแรกไว้ก่อน
     * -----------------------------------------------
     */
    if (block_data_available < block_length) {

       ESP_LOGW(TAG,

                     "DLMS data block is incomplete! Expected=%u Received=%u",

                     (unsigned)block_length,

                     (unsigned)block_data_available);

        if (block_number != s_association_view_expected_block) {
            ESP_LOGW(TAG, "Unexpected block. Expected=%" PRIu32
                     " Received=%" PRIu32,
                     s_association_view_expected_block, block_number);
            return ESP_ERR_INVALID_RESPONSE;
        }

        if (s_association_view_data_length + block_length >
            DLMS_ASSOCIATION_VIEW_BUFFER_SIZE) {
            ESP_LOGE(TAG, "Association View buffer overflow");
            return ESP_ERR_NO_MEM;
        }

        memcpy(&s_association_view_data[s_association_view_data_length],
               &data[block_data_offset],
               block_data_available);

        /* ใช้ offset แบบ absolute ภายในบัฟเฟอร์ */
        s_get_data_expected_length =
            s_association_view_data_length + block_length;
        s_get_data_received_length =
            s_association_view_data_length + block_data_available;
        s_association_view_data_length = s_get_data_received_length;

        s_get_data_complete                = false;
        s_association_view_waiting_segment = true;
        s_association_view_invoke_id       = invoke_id;
        s_association_view_last_block      = last_block;
        s_association_view_current_block   = block_number;

        ESP_LOGI(TAG, "Block %" PRIu32 " partial: %u/%u bytes",
                 block_number,
                 (unsigned)s_get_data_received_length,
                 (unsigned)s_get_data_expected_length);

        return ESP_ERR_NOT_FINISHED;
    }

        if (block_data_available > block_length) {

            ESP_LOGW(
                TAG,
                "Extra bytes after DLMS block: %u",
                (unsigned)(
                    block_data_available - block_length));
        }

        /*
         * Check block order.
         */
        if (block_number != s_association_view_expected_block) {

            ESP_LOGW(
                TAG,
                "Unexpected block number. "
                "Expected=%" PRIu32
                " Received=%" PRIu32,
                s_association_view_expected_block,
                block_number);

            return ESP_ERR_INVALID_RESPONSE;
        }

        /*
         * Check destination buffer.
         */
        if (block_length >
            (DLMS_ASSOCIATION_VIEW_BUFFER_SIZE -
             s_association_view_data_length)) {

            ESP_LOGE(
                TAG,
                "Association View buffer overflow");

            return ESP_ERR_NO_MEM;
        }

        /*
         * Append this block.
         */
        memcpy(
            &s_association_view_data[
                s_association_view_data_length],
            &data[block_data_offset],
            block_length);

        s_association_view_data_length += block_length;

        s_get_data_complete                = true;
        s_association_view_waiting_segment = false;
        s_association_view_invoke_id       = invoke_id;
        s_association_view_last_block      = last_block;


        ESP_LOGI(
            TAG,
            "Association View buffer = %u bytes",
            (unsigned)s_association_view_data_length);

        /*
         * ---------------------------------------------------
         * Last block?
         * ---------------------------------------------------
         */
        if (last_block != 0U) {

           s_association_view_complete = true;

            ESP_LOGI(
                TAG,
                "Association View Complete = %u bytes",
                (unsigned)s_association_view_data_length);

            /*
             * Layer 1.2 will parse this buffer.
             */
            return ESP_OK;
        }

        /*
         * More DLMS data blocks are required.
         */
        s_association_view_current_block = block_number;
        s_association_view_expected_block = block_number + 1U;

       /* ส่งเลขบล็อกที่รับแล้ว ไม่ใช่ +1 */

        // return dlms_cosem_send_get_next_block( invoke_id,s_association_view_expected_block);
          return dlms_cosem_send_get_next_block(invoke_id, block_number);
    }

    /*
     * Unsupported GET response.
     */
    ESP_LOGE(
        TAG,
        "Unsupported GET response choice: 0x%02X",
        response_choice);

    return ESP_ERR_INVALID_RESPONSE;
}

// static void dlms_cosem_log_obis(
//     const uint8_t obis[6])
// {
//     ESP_LOGI(
//         TAG,
//         "OBIS: %u.%u.%u.%u.%u.%u",
//         obis[0],
//         obis[1],
//         obis[2],
//         obis[3],
//         obis[4],
//         obis[5]);
// }