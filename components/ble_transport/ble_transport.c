/******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>

#include "ble_transport.h"
#include "ble_transport_event.h"
#include "ble_transport_types.h"
#include "dlms_hdlc.h"

#if defined(__has_include)
#  if __has_include(<esp_err.h>)
#    include <esp_err.h>
#  elif __has_include("esp_err.h")
#    include "esp_err.h"
#  else
#    include "esp_err.h"
#  endif
#else
#  include "esp_err.h"
#endif

#if defined(__has_include)
#  if __has_include(<esp_event.h>)
#    include <esp_event.h>
#  elif __has_include("esp_event.h")
#    include "esp_event.h"
#  else
#    include "esp_event.h"
#  endif
#else
#  include "esp_event.h"
#endif

#if defined(__has_include)
#  if __has_include(<esp_log.h>)
#    include <esp_log.h>
#  elif __has_include("esp_log.h")
#    include "esp_log.h"
#  else
#    include "esp_log.h"
#  endif
#else
#  include "esp_log.h"
#endif

#if defined(__has_include)
#  if __has_include(<nvs_flash.h>)
#    include <nvs_flash.h>
#  elif __has_include("nvs_flash.h")
#    include "nvs_flash.h"
#  else
#    include "nvs_flash.h"
#  endif
#else
#  include "nvs_flash.h"
#endif



#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/timers.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"

#include "ble_company.h"
#include "ble_service_uuid.h"


/* ---------- Meter Discovery Prototypes ---------- */
static void md_start(uint16_t conn_handle);
static void md_reset(void);
static void md_fail(uint16_t conn_handle, const char *reason);
static void md_discover_services(uint16_t conn_handle);

/******************************************************************************
 * Defines
 ******************************************************************************/
static const char *TAG = "BLE_TRANSPORT";



#define RX_ASSEMBLY_BUF_SIZE 512
#define BLE_CONNECT_RETRY_INITIAL_MS 1000U
#define BLE_CONNECT_RETRY_MAX_MS 30000U

static uint8_t s_rx_assembly_buf[RX_ASSEMBLY_BUF_SIZE];

static uint16_t s_rx_assembly_len = 0;
static TimerHandle_t s_connect_retry_timer;
static volatile bool s_connect_retry_pending;
static uint32_t s_connect_retry_delay_ms = BLE_CONNECT_RETRY_INITIAL_MS;
/******************************************************************************
 * Private Types
 ******************************************************************************/


typedef struct
{
    volatile ble_transport_state_t driver_state;

    volatile ble_link_state_t link_state;

    volatile bool ready;

    volatile bool connected;

    ble_addr_t peer_addr;

    char device_name[32];

    uint16_t conn_id;

    uint16_t mtu;

    uint16_t service_start;
    uint16_t service_end;

    uint16_t tx_handle;
    uint16_t rx_handle;

    bool target_found;

    ble_transport_rx_callback_t rx_callback;

} ble_transport_context_t;

/******************************************************************************
 * Private Variables
 ******************************************************************************/
static ble_transport_context_t g_ble =
{
    .driver_state = BLE_TRANSPORT_STATE_UNINITIALIZED,

    .link_state = BLE_LINK_STATE_DISCONNECTED,

    .mtu = BLE_TRANSPORT_DEFAULT_MTU,
};

static uint8_t g_own_addr_type;
static const uint8_t TARGET_METER_ADDR[6] = {
    0x96, 0x24, 0x35, 0x00, 0x68, 0x00
};
static uint8_t s_target_meter_addr[6];
static uint8_t s_target_address_type;
static bool s_target_address_type_valid;
static bool s_target_address_enabled;
static volatile bool s_manual_scan;
static volatile bool s_ble_scanning;
static ble_transport_scan_result_t
    s_ble_scan_results[BLE_TRANSPORT_SCAN_RESULT_COUNT];
static size_t s_ble_scan_result_count;
static portMUX_TYPE s_ble_scan_mux = portMUX_INITIALIZER_UNLOCKED;


/* ---------- Dynamic GATT discovery state ---------- */
#define BLE_MAX_SERVICES  8
#define BLE_MAX_CHRS      16

typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
} ble_svc_range_t;

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
    uint8_t  properties;
} ble_chr_info_t;

static ble_svc_range_t s_svcs[BLE_MAX_SERVICES];
static uint8_t s_svc_count;
static uint8_t s_svc_index;

static ble_chr_info_t s_chrs[BLE_MAX_CHRS];
static uint8_t s_chr_count;

/* ตัวแปรเก็บ Handle ที่ค้นพบแบบไดนามิก */
static uint16_t meter_tx_handle   = 0;
static uint16_t meter_rx_handle   = 0;
static uint16_t meter_cccd_handle = 0;
static uint8_t  meter_tx_props    = 0;
static uint8_t  meter_rx_props    = 0;
/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/* ---------- Prototypes ---------- */

static void process_incoming_fragment(const uint8_t *data, uint16_t len);

static void gatt_reset_state(void);


static bool is_standard_service(const ble_uuid_any_t *uuid);


static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);

// static void gap_event_handler(void);
// static void gattc_event_handler(void);

static esp_err_t start_scan(void);
static void ble_schedule_connect_retry(void);
static void ble_connect_retry_timer_callback(TimerHandle_t timer);
static esp_err_t connect_device(void);
static esp_err_t discover_service(void);
static esp_err_t enable_notify(void);
static esp_err_t ble_set_link_state(ble_link_state_t state);
static void ble_set_driver_state(ble_transport_state_t state);
static void ble_host_task(void *param);
static bool ble_is_running(void);
static void dlms_hdlc_parse_and_log( const uint8_t *buffer, size_t length);

static int  ble_gap_event(struct ble_gap_event *event, void *arg);
static int dlms_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr,
                         void *arg);
static void print_addr(const ble_addr_t *addr);
static void ble_on_reset(int reason);
static void ble_on_sync(void);
static void print_adv_fields(const struct ble_hs_adv_fields *fields);
static void dump_adv_packet(const uint8_t *data, uint8_t len);
static void decode_adv_packet(const uint8_t *data, uint8_t len);
static void print_hex(const uint8_t *data, uint8_t len);
const char *company_name(uint16_t id);

static int dlms_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr,
                         void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;

    if (error->status == 0) {
        ESP_LOGI(TAG, "DLMS GATT write completed successfully");
    } else {
        ESP_LOGE(TAG, "DLMS GATT write failed, status=%d", error->status);
    }
    return 0;
}


    // ตัวอย่าง Global/State Variables สำหรับเก็บ Handle
// static uint16_t meter_tx_handle = 16; // Write Handle
// static uint16_t meter_rx_handle = 18; // Notify Handle
// static uint16_t meter_cccd_handle = 19; // CCCD Handle (มักจะ = rx_handle + 1)

// ตัวอย่าง: สมมติว่ามิเตอร์ของคุณคือ MAC Address 00:68:00:35:24:96 มิเตอร์ที่บ้าน My Home
// // ตัวอย่าง: สมมติว่ามิเตอร์ของคุณคือ MAC Address 90:3D:BD:4D:4B:76 มิเตอร์ที่ทำงาน My office
//   static const uint8_t TARGET_METER_ADDR[6] = {0x76, 0x4B, 0x4D, 0xBD, 0x3D, 0x90}; // เรียงจาก LSB ไป MSB
 

/******************************************************************************
 * Private Functions
 ******************************************************************************/
// static void dlms_hdlc_parse_and_log(
//     const uint8_t *buffer,
//     size_t length)
// {
//     if (buffer == NULL || length == 0)
//     {
//         ESP_LOGW(TAG, "Cannot parse empty HDLC data");
//         return;
//     }

//     dlms_hdlc_frame_t frame = {0};

//     dlms_hdlc_parse_result_t result =
//         dlms_hdlc_parse_frame(buffer, length, &frame);

//     ESP_LOGI(TAG,
//              "HDLC parse result: %s",
//              dlms_hdlc_parse_result_to_string(result));

//     if (result == DLMS_HDLC_PARSE_OK)
//     {
//         dlms_hdlc_dump_frame(&frame);
//     }
//     else
//     {
//         ESP_LOGE(TAG,
//                  "HDLC frame parsing failed: %s",
//                  dlms_hdlc_parse_result_to_string(result));
//     }
// }


static void dlms_hdlc_parse_and_log(
    const uint8_t *buffer,
    size_t length)
{
    if (buffer == NULL || length == 0)
    {
        ESP_LOGW(TAG, "Cannot parse empty HDLC data");
        return;
    }

    dlms_hdlc_frame_t frame = {0};

    dlms_hdlc_parse_result_t result =
        dlms_hdlc_parse_frame(buffer, length, &frame);

    ESP_LOGI(TAG,
             "HDLC parse result: %s",
             dlms_hdlc_parse_result_to_string(result));

    if (result == DLMS_HDLC_PARSE_OK)
    {
        dlms_hdlc_dump_frame(&frame);

        /*
         * Forward the complete HDLC frame
         * to the upper DLMS layer.
         */
        if (g_ble.rx_callback != NULL)
        {
            g_ble.rx_callback(buffer, length);
        }
    }
    else
    {
        ESP_LOGE(TAG,
                 "HDLC frame parsing failed: %s",
                 dlms_hdlc_parse_result_to_string(result));
    }
}


static void ble_context_init(void)
  {
    memset(&g_ble, 0, sizeof(g_ble));
    memcpy(s_target_meter_addr, TARGET_METER_ADDR,
           sizeof(s_target_meter_addr));
    s_target_address_type = 0U;
    s_target_address_type_valid = false;
    s_target_address_enabled = true;
    s_manual_scan = false;
    s_ble_scanning = false;
    portENTER_CRITICAL(&s_ble_scan_mux);
    memset(s_ble_scan_results, 0, sizeof(s_ble_scan_results));
    s_ble_scan_result_count = 0U;
    portEXIT_CRITICAL(&s_ble_scan_mux);
    

    g_ble.driver_state = BLE_TRANSPORT_STATE_UNINITIALIZED;
    g_ble.link_state   = BLE_LINK_STATE_DISCONNECTED;
    g_ble.mtu          = BLE_TRANSPORT_DEFAULT_MTU;
  }


static esp_err_t start_scan(void)
{
    if (s_ble_scanning) {
        return ESP_OK;
    }

    struct ble_gap_disc_params scan_params;

    memset(&scan_params, 0, sizeof(scan_params));

    scan_params.passive = 0;

    scan_params.itvl = 0x50;

    scan_params.window = 0x30;

    scan_params.filter_duplicates = 0;

    int rc = ble_gap_disc(
                g_own_addr_type,
                15000U,
                &scan_params,
                ble_gap_event,
                NULL);

    if (rc != 0)
    {
        ESP_LOGE(TAG,
                 "ble_gap_disc() failed rc=%d",
                 rc);

        return ESP_FAIL;
    }

    s_ble_scanning = true;
    ESP_LOGI(TAG, "BLE Scan Started");

    return ESP_OK;
}

static void ble_schedule_connect_retry(void)
{
    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING ||
        s_manual_scan) {
        return;
    }

    if (s_connect_retry_timer == NULL) {
        ESP_LOGE(TAG, "Cannot schedule BLE reconnect: retry timer unavailable");
        return;
    }

    const uint32_t delay_ms = s_connect_retry_delay_ms;
    const TickType_t delay_ticks = pdMS_TO_TICKS(delay_ms);
    if (xTimerChangePeriod(s_connect_retry_timer,
                           delay_ticks > 0U ? delay_ticks : 1U,
                           0U) != pdPASS) {
        ESP_LOGE(TAG, "Failed to schedule BLE reconnect retry");
        return;
    }

    s_connect_retry_pending = true;
    ESP_LOGW(TAG, "Retrying BLE scan in %u ms", (unsigned)delay_ms);
    if (s_connect_retry_delay_ms < BLE_CONNECT_RETRY_MAX_MS) {
        s_connect_retry_delay_ms *= 2U;
        if (s_connect_retry_delay_ms > BLE_CONNECT_RETRY_MAX_MS) {
            s_connect_retry_delay_ms = BLE_CONNECT_RETRY_MAX_MS;
        }
    }
}

static void ble_connect_retry_timer_callback(TimerHandle_t timer)
{
    (void)timer;
    s_connect_retry_pending = false;

    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING ||
        g_ble.connected ||
        s_ble_scanning ||
        s_manual_scan) {
        return;
    }

    esp_err_t err = start_scan();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE reconnect scan failed: %s", esp_err_to_name(err));
        ble_schedule_connect_retry();
    }
}


// static esp_err_t connect_device(void)
// {
//     int rc;

//     rc = ble_gap_connect(
//             g_own_addr_type,
//             &g_ble.peer_addr,
//             10000,
//             NULL,
//             ble_gap_event,
//             NULL);

//     if(rc)
//     {
//         ESP_LOGE(TAG,
//                  "Connect failed rc=%d",
//                  rc);

//         return ESP_FAIL;
//     }

//     ESP_LOGI(TAG,
//              "Connecting...");

//     return ESP_OK;
// }

static esp_err_t connect_device(void)
{
    int rc;

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Connecting to meter...");

    ESP_LOGI(TAG,
             "Own Addr Type = %d",
             g_own_addr_type);

    ESP_LOGI(TAG,
             "Peer Addr Type = %d",
             g_ble.peer_addr.type);

    ESP_LOGI(TAG,
             "Peer Addr = %02X:%02X:%02X:%02X:%02X:%02X",
             g_ble.peer_addr.val[5],
             g_ble.peer_addr.val[4],
             g_ble.peer_addr.val[3],
             g_ble.peer_addr.val[2],
             g_ble.peer_addr.val[1],
             g_ble.peer_addr.val[0]);

    rc = ble_gap_connect(
            g_own_addr_type,
            &g_ble.peer_addr,
            10000,
            NULL,
            ble_gap_event,
            NULL);

    ESP_LOGI(TAG,
             "ble_gap_connect() rc=%d",
             rc);

    if (rc != 0)
    {
        ESP_LOGE(TAG,
                 "Connect failed immediately, rc=%d",
                 rc);

        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Connecting...");

    ESP_LOGI(TAG, "================================");

    return ESP_OK;
}




// Callback หลังเปิด Notification สำเร็จ
static int ble_transport_on_notify(struct ble_gap_event *event, void *arg)
{
    // เมื่อมิเตอร์ส่งข้อมูลตอบกลับผ่าน Notification จะมาเข้าที่นี่
    return 0;
}
// Callback สำหรับรับ Characteristic Discovery


int ble_transport_send(const uint8_t *data, uint16_t len)
{
    if (!g_ble.ready || !g_ble.connected) {
        ESP_LOGE(TAG, "Transport not ready");
        return -1;
    }

    // เปลี่ยนมาใช้ Write Without Response
    int rc = ble_gattc_write_no_rsp_flat(g_ble.conn_id, meter_tx_handle, data, len);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send DLMS frame via write_no_rsp, rc=%d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "Sent DLMS Frame (%d bytes) via Write No Rsp to Handle %d", len, meter_tx_handle);
    return 0;
}


// Callback เมื่อมี Notification/Indication ขามาจากมิเตอร์ (Handle 18)
int ble_transport_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        
    case BLE_GAP_EVENT_NOTIFY_RX:
        // ตรวจสอบว่าเป็น Notification มาจาก Notify Handle ของ DLMS (Handle 18) หรือไม่
        if (event->notify_rx.attr_handle == 18) {
            ESP_LOGI("BLE_TRANSPORT", "Received DLMS Raw Data (%d bytes)", 
                     event->notify_rx.om->om_len);
            
            // ดึงข้อมูล Raw Bytes จาก OS mbuf
            uint8_t rx_buf[256];
            uint16_t rx_len = event->notify_rx.om->om_len;
            os_mbuf_copydata(event->notify_rx.om, 0, rx_len, rx_buf);

            // TODO: ส่ง rx_buf ไปยัง dlms_client / dlms_codec component เพื่อ Decode
            // dlms_client_on_data_received(rx_buf, rx_len);
        }
        break;

    default:
        break;
    }
    return 0;
}
//    // 1. Callback รับข้อมูล Characteristics
// static int chr_discovery_cb(uint16_t conn_handle,
//                             const struct ble_gatt_error *error,
//                             const struct ble_gatt_chr *chr,
//                             void *arg)
// {
//     if (error->status == BLE_HS_EDONE) {
//         ESP_LOGI(TAG, "Characteristic discovery complete");
//         // TODO: ย้าย State ไปแลกเปลี่ยน MTU (Exchange MTU) หรือเตรียม Handshake DLMS
//         return 0;
//     }

//     if (error->status != 0 || chr == NULL) {
//         return 0;
//     }

//     ESP_LOGI(TAG, "--- Characteristic Found ---");
//     ESP_LOGI(TAG, "Val Handle : %d", chr->val_handle);
//     ESP_LOGI(TAG, "Properties : 0x%02X", chr->properties);

//     // แสดง UUID128 ของ Characteristic
//     if (chr->uuid.u.type == BLE_UUID_TYPE_128) {
//         ESP_LOGI(TAG, "UUID128    : %02X%02X%02X%02X-...", 
//                  chr->uuid.u128.value[15], chr->uuid.u128.value[14]);
//     }
//     return 0;
// }




// 2. เรียกค้นหา Characteristic เมื่อ Service Discovery เสร็จสิ้น





static void gatt_reset_state(void)
{
    meter_tx_handle   = 0;
    meter_rx_handle   = 0;
    meter_cccd_handle = 0;
    meter_tx_props    = 0;
    meter_rx_props    = 0;
    s_rx_assembly_len = 0;

    s_svc_count = 0;
    s_svc_index = 0;
    s_chr_count = 0;

    g_ble.mtu   = BLE_TRANSPORT_DEFAULT_MTU;
    g_ble.ready = false;
}

static bool is_standard_service(const ble_uuid_any_t *uuid)
{
    if (uuid->u.type != BLE_UUID_TYPE_16) {
        return false;
    }

    switch (uuid->u16.value) {
        case 0x1800: /* Generic Access */
        case 0x1801: /* Generic Attribute */
        case 0x180A: /* Device Information */
        case 0x180F: /* Battery */
            return true;
        default:
            return false;
    }
}











static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr,
                         void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "CCCD write failed status=%d (0x%X)",
                 error->status, (unsigned)error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    g_ble.ready = true;
    ESP_LOGI(TAG, "BLE Transport Ready: TX=%u RX=%u CCCD=%u MTU=%u",
             (unsigned)meter_tx_handle, (unsigned)meter_rx_handle,
             (unsigned)meter_cccd_handle, (unsigned)g_ble.mtu);

    // แจ้งเตือนเลเยอร์หลักว่าพร้อมส่ง SNRM แล้ว
    if (g_ble.rx_callback != NULL) {
        // หรือให้ main.c เช็คจาก ble_transport_is_ready()
    }
    return 0;
}









static esp_err_t enable_notify(void)
   {
    return ESP_OK;
   }

static esp_err_t ble_set_link_state(ble_link_state_t state)
  {
    if (g_ble.link_state == state)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG,
             "Link State: %d -> %d",
             g_ble.link_state,
             state);

    g_ble.link_state = state;
    return ESP_OK;  
  }

static void ble_host_task(void *param)
   {
    ESP_LOGI(TAG, "NimBLE Host Started");

    nimble_port_run();

    nimble_port_freertos_deinit();
   }

static void ble_on_reset(int reason)
   {
    ESP_LOGW(TAG, "BLE Reset (%d)", reason);
   }


static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE Host Sync");

    int rc;

    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);

    if (rc != 0)
    {
        ESP_LOGE(TAG,
                 "ble_hs_id_infer_auto() failed (%d)",
                 rc);
    }
}

static bool ble_is_running(void)
   {
    return (g_ble.driver_state == BLE_TRANSPORT_STATE_RUNNING);
   }

static void print_addr(const ble_addr_t *addr)
  {
    ESP_LOGI(TAG,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5],
             addr->val[4],
             addr->val[3],
             addr->val[2],
             addr->val[1],
             addr->val[0]);
   }

static void ble_store_scan_result(const struct ble_gap_disc_desc *discovery)
{
    struct ble_hs_adv_fields fields = {0};
    int parse_rc = ble_hs_adv_parse_fields(
        &fields, discovery->data, discovery->length_data);
    if (parse_rc != 0) {
        ESP_LOGD(TAG, "Could not parse BLE advertisement fields: %d",
                 parse_rc);
    }

    portENTER_CRITICAL(&s_ble_scan_mux);
    size_t index = 0U;
    for (; index < s_ble_scan_result_count; ++index) {
        if (s_ble_scan_results[index].address_type ==
                discovery->addr.type &&
            memcmp(s_ble_scan_results[index].address,
                   discovery->addr.val, sizeof(discovery->addr.val)) == 0) {
            break;
        }
    }
    if (index == s_ble_scan_result_count &&
        s_ble_scan_result_count < BLE_TRANSPORT_SCAN_RESULT_COUNT) {
        ++s_ble_scan_result_count;
    } else if (index == s_ble_scan_result_count) {
        portEXIT_CRITICAL(&s_ble_scan_mux);
        return;
    }

    ble_transport_scan_result_t *result = &s_ble_scan_results[index];
    memcpy(result->address, discovery->addr.val, sizeof(result->address));
    result->address_type = discovery->addr.type;
    result->rssi = discovery->rssi;
    if (fields.name != NULL && fields.name_len > 0U) {
        size_t name_length = fields.name_len;
        if (name_length >= sizeof(result->name)) {
            name_length = sizeof(result->name) - 1U;
        }
        memcpy(result->name, fields.name, name_length);
        result->name[name_length] = '\0';
    }
    portEXIT_CRITICAL(&s_ble_scan_mux);
}

   // GAP Event Main Handler
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
        // 1. ตรวจพบอุปกรณ์จากการสแกน
        case BLE_GAP_EVENT_DISC:
        {
            if (s_manual_scan) {
                ble_store_scan_result(&event->disc);
                break;
            }

            if (s_target_address_enabled &&
                (!s_target_address_type_valid ||
                 event->disc.addr.type == s_target_address_type) &&
                memcmp(event->disc.addr.val, s_target_meter_addr,
                       sizeof(s_target_meter_addr)) == 0)
            {
                ESP_LOGI(TAG, "Found target DLMS meter! RSSI: %d dBm", event->disc.rssi);
                ESP_LOGI(TAG, "Stopping scan and connecting...");

                // หยุดการสแกนก่อน
                s_ble_scanning = false;
                (void)ble_gap_disc_cancel();

                // บันทึก Peer Address และเริ่มเชื่อมต่อ
                g_ble.peer_addr = event->disc.addr;
                struct ble_hs_adv_fields fields = {0};
                if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                            event->disc.length_data) == 0 &&
                    fields.name != NULL) {
                    size_t name_length = fields.name_len;
                    if (name_length >= sizeof(g_ble.device_name)) {
                        name_length = sizeof(g_ble.device_name) - 1U;
                    }
                    portENTER_CRITICAL(&s_ble_scan_mux);
                    memcpy(g_ble.device_name, fields.name, name_length);
                    g_ble.device_name[name_length] = '\0';
                    portEXIT_CRITICAL(&s_ble_scan_mux);
                }
                
                // เชื่อมต่อโดยระบุ peer address และ type ที่ได้สแกนเจอ
                ble_set_link_state(BLE_LINK_STATE_CONNECTING);
                int rc = ble_gap_connect(g_own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to connect to meter, rc=%d", rc);
                    ble_set_link_state(BLE_LINK_STATE_DISCONNECTED);
                    ble_schedule_connect_retry();
              }  
              
               
           }
            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            s_ble_scanning = false;
            s_manual_scan = false;
            if (!g_ble.connected &&
                g_ble.link_state != BLE_LINK_STATE_CONNECTING &&
                !s_connect_retry_pending &&
                g_ble.driver_state == BLE_TRANSPORT_STATE_RUNNING) {
                (void)start_scan();
            }
            break;

        // 2. สถานะการเชื่อมต่อ (Connect Status)
          case BLE_GAP_EVENT_CONNECT:
        {
            if (event->connect.status == 0) {
                s_connect_retry_pending = false;
                s_connect_retry_delay_ms = BLE_CONNECT_RETRY_INITIAL_MS;
                if (s_connect_retry_timer != NULL) {
                    (void)xTimerStop(s_connect_retry_timer, 0U);
                }
                ESP_LOGI(TAG, "Connected successfully! Conn Handle = %d", event->connect.conn_handle);
                g_ble.conn_id = event->connect.conn_handle;
                g_ble.connected = true;
                ble_set_link_state(BLE_LINK_STATE_CONNECTED);

                if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING) {
                    ESP_LOGW(TAG,
                             "Connection completed after transport stopped; "
                             "disconnecting");
                    (void)ble_gap_terminate(g_ble.conn_id,
                                            BLE_ERR_REM_USER_CONN_TERM);
                    break;
                }

                // เปลี่ยนมาเรียกใช้งานชุดไดนามิกใหม่ตามโปรไฟล์ UUID
                md_start(event->connect.conn_handle);
            } else {
                ESP_LOGW(TAG,
                         "Connection failed, status=%d%s",
                         event->connect.status,
                         event->connect.status == 13
                             ? " (HCI limited resources)"
                             : "");
                ble_set_link_state(BLE_LINK_STATE_DISCONNECTED);
                ble_schedule_connect_retry();
            }
            break;
        }

        // 3. สถานะการหลุดเชื่อมต่อ (Disconnect)
    case BLE_GAP_EVENT_DISCONNECT:
        {
            ESP_LOGW(TAG, "Disconnected from meter, reason=%d", event->disconnect.reason);
            gatt_reset_state();
            g_ble.connected = false;
            g_ble.ready = false;
            ble_set_link_state(BLE_LINK_STATE_DISCONNECTED);

            if (g_ble.driver_state == BLE_TRANSPORT_STATE_RUNNING) {
                ble_schedule_connect_retry();
            }
            break;
        }
        // 4. รับข้อมูล Notification / Indication จากมิเตอร์ (RX Data)
//         case BLE_GAP_EVENT_NOTIFY_RX:
// {
//     if (event->notify_rx.attr_handle == meter_rx_handle)
//     {
//         uint16_t rx_len = event->notify_rx.om->om_len;
//         uint8_t rx_buf[256];

//         os_mbuf_copydata(
//             event->notify_rx.om,
//             0,
//             rx_len,
//             rx_buf
//         );

//         ESP_LOGI(
//             TAG,
//             "Received DLMS Response (%d bytes) from Handle %d:",
//             rx_len,
//             event->notify_rx.attr_handle
//         );

//         print_hex(rx_buf, rx_len);

//         /*
//          * Send received BLE payload to DLMS HDLC parser.
//          */
//          dlms_hdlc_parse_and_log(rx_buf, rx_len);
//     }

//     break;
// }
case BLE_GAP_EVENT_NOTIFY_RX:
{
    uint16_t handle = event->notify_rx.attr_handle;
    struct os_mbuf *om = event->notify_rx.om;

    if (handle == meter_rx_handle && om != NULL)
    {
        uint8_t rx_buf[256];
        uint16_t rx_len = OS_MBUF_PKTLEN(om);

        if (rx_len <= sizeof(rx_buf))
        {
            if (os_mbuf_copydata(om, 0, rx_len, rx_buf) == 0)
            {
                // ส่งผ่านตัวพักและต่อข้อมูล (รองรับทั้งแบบส่งก้อนเดียวและแบ่งท่อน)
                process_incoming_fragment(rx_buf, rx_len);
            }
        }
    }
    break;
}

        default:
            break;
    }
    return 0;
}
// static int ble_gap_event(struct ble_gap_event *event, void *arg)
//   {
    
//     switch (event->type)
//     {
       
//         case BLE_GAP_EVENT_DISC:
//             {   
//                  if (g_ble.link_state != BLE_LINK_STATE_SCANNING)
//                     {
//                         return 0;
//                     }

//                 ESP_LOGI(TAG,
//                         "Addr type = %d",
//                         event->disc.addr.type);

//                 g_ble.device_name[0] = '\0';
//                 decode_adv_packet(event->disc.data,
//                                   event->disc.length_data);
                    

//                 if (strcmp(g_ble.device_name, "GK006800352496") == 0 
//                 //  || strcmp(g_ble.device_name, "SS21541121") == 0
//                     )
//                     {
//                         ESP_LOGI(TAG, "Target Meter Found");

//                         g_ble.peer_addr = event->disc.addr;
//                         g_ble.target_found = true;

//                         int rc = ble_gap_disc_cancel();

//                         ESP_LOGI(TAG,
//                                 "ble_gap_disc_cancel rc=%d", rc);
                        
                        
//                         vTaskDelay(pdMS_TO_TICKS(200));

//                         connect_device();
//                         return 0;
//                     }


                    
//                 break;
//             }

//         case BLE_GAP_EVENT_CONNECT:
//                 {   
//                     ESP_LOGI(TAG,
//                         "Connect status=%d",
//                         event->connect.status);

//                     if(event->connect.status == 0)
//                     {
//                             ESP_LOGI(TAG, "================================");
//                             ESP_LOGI(TAG, "BLE CONNECTED");
//                             ESP_LOGI(TAG,
//                                     "Connection Handle = %d",
//                                     event->connect.conn_handle);
//                             ESP_LOGI(TAG, "================================");

//                         g_ble.conn_id =
//                             event->connect.conn_handle;

//                         ble_set_link_state(
//                             BLE_LINK_STATE_CONNECTED);

//                         ESP_LOGI(TAG,
//                                "Calling discover_service()");

//                         discover_service();
//                     }
//                     else
//                     {
//                       ESP_LOGE(TAG,
//                                "Connect Failed status=%d",
//                                 event->connect.status);

//                         start_scan();
//                     }

//                     break;
//                 }

//       case BLE_GAP_EVENT_DISCONNECT:
//                     {
//                         ESP_LOGW(TAG,
//                                 "Disconnected, reason=%d",
//                                 event->disconnect.reason);

//                         ble_set_link_state(BLE_LINK_STATE_DISCONNECTED);

//                         start_scan();

//                         break;
//                     }
           
//         case BLE_GAP_EVENT_DISC_COMPLETE:
//             {
//                 ESP_LOGI(TAG, "Scan Complete");

//                 if (g_ble.target_found)
//                 {
//                     ESP_LOGI(TAG,
//                             "Target found -> start connecting");

//                     ble_set_link_state(
//                         BLE_LINK_STATE_CONNECTING);

//                     connect_device();
//                 }

//                 break;
//             }
           

//             default:
//              break;
//     }

//     return 0;
//   }

  static void ble_set_driver_state(ble_transport_state_t state)
  {
    if (g_ble.driver_state == state)
        return;

    ESP_LOGI(TAG,
             "Driver State: %d -> %d",
             g_ble.driver_state,
             state);

    g_ble.driver_state = state;
  }

  static void print_adv_fields(
        const struct ble_hs_adv_fields *fields)
{
    // if (fields->flags_is_present)
    // {
        ESP_LOGI(TAG,"Flags : 0x%02X", fields->flags);
    // }

    if (fields->tx_pwr_lvl_is_present)
    {
        ESP_LOGI(TAG,
                 "TX Power : %d",
                 fields->tx_pwr_lvl);
    }

    if (fields->name != NULL)
    {
        char name[32] = {0};

        size_t len = fields->name_len;

        if (len >= sizeof(name))
            len = sizeof(name) - 1;

        memcpy(name, fields->name, len);

        name[len] = 0;

        ESP_LOGI(TAG,
                 "Name : %s",
                 name);
    }
}

static void dump_adv_packet(const uint8_t *data, uint8_t len)
{
    for (int i = 0; i < len; i += 16)
    {
        char line[80];
        int pos = 0;

        pos += snprintf(line + pos,
                        sizeof(line) - pos,
                        "%02d : ",
                        i);

        for (int j = 0; j < 16 && (i + j) < len; j++)
        {
            pos += snprintf(line + pos,
                            sizeof(line) - pos,
                            "%02X ",
                            data[i + j]);
        }

        ESP_LOGI(TAG, "%s", line);
    }
}

static void decode_adv_packet(const uint8_t *data,
                              uint8_t len)
{
    uint8_t index = 0;

    while (index < len)
    {
        uint8_t field_len = data[index];

        if (field_len == 0)
            break;

        if ((index + field_len) >= len)
            break;

        uint8_t type = data[index + 1];

        const uint8_t *field = &data[index + 2];

        uint8_t value_len = field_len - 1;

        ESP_LOGI(TAG,"------------------------");
        ESP_LOGI(TAG,
                 "Type : 0x%02X",
                 type);

        switch(type)
        {
         case 0x01:
                ESP_LOGI(TAG,"Flags : 0x%02X", field[0]);
               
                break;

         case 0x09:
            {
                char name[32]={0};

                size_t n=value_len;

                if(n>sizeof(name)-1)
                    n=sizeof(name)-1;

                memcpy(name,field,n);

                ESP_LOGI(TAG, "Name : %s", name);
                strncpy(g_ble.device_name,
                        name,
                        sizeof(g_ble.device_name)-1);

                break;
            }

        case 0x08:
            {
                char name[32]={0};

                size_t n=value_len;

                if(n>sizeof(name)-1)
                    n=sizeof(name)-1;

                memcpy(name,field,n);

                ESP_LOGI(TAG, "Short Name : %s", name);

                break;
            }

        //  case 0xFF:
        //         ESP_LOGI(TAG, "Manufacturer");
        //         print_hex(field,value_len);

        //         break;
        case 0xFF:
         {
            if (value_len < 2)
                break;

            uint16_t company =
                field[0] |
                (field[1] << 8);

            ESP_LOGI(TAG, "Manufacturer Specific Data");
            ESP_LOGI(TAG, "Company ID : 0x%04X", company);
            ESP_LOGI(TAG, "Vendor     : %s",
                    ble_company_name(company));

            if (value_len > 2)
            {
                ESP_LOGI(TAG, "Payload");
                print_hex(field + 2, value_len - 2);
            }

            break;
         }

         case 0x02:
         case 0x03:
                ESP_LOGI(TAG, "UUID16");
                print_hex(field,value_len);

                break;

         case 0x06:
         case 0x07:
                ESP_LOGI(TAG, "UUID128");
                print_hex(field,value_len);

                break;

         case 0x16:
                {
                    if (value_len < 2)
                        break;

                    uint16_t uuid =
                        field[0] |
                        (field[1] << 8);

                    ESP_LOGI(TAG, "Service Data");
                    ESP_LOGI(TAG, "UUID16 : 0x%04X", uuid);
                    ESP_LOGI(TAG, "Service : %s",
                            ble_service_name(uuid));

                    if (value_len > 2)
                    {
                        ESP_LOGI(TAG, "Payload");
                        print_hex(field + 2, value_len - 2);
                    }

                    break;
                }

         case 0x0A:
                ESP_LOGI(TAG, "TX Power=%d", (int8_t)field[0]);

                break;

         case 0x19:
                ESP_LOGI(TAG, "Appearance");
                print_hex(field,value_len);

                break;  

        default:
                ESP_LOGI(TAG, "Unknown Type");
                print_hex(field,value_len);

                break;
        }

        index += field_len + 1;
    }
}

static void print_hex(const uint8_t *data, uint8_t len)
{
    char line[128];
    char *p;

    for (uint8_t i = 0; i < len; i += 16)
    {
        p = line;

        p += sprintf(p, "%02X : ", i);

        uint8_t remain = len - i;

        if (remain > 16)
            remain = 16;

        for (uint8_t j = 0; j < remain; j++)
        {
            p += sprintf(p, "%02X ", data[i + j]);
        }

        ESP_LOGI(TAG, "%s", line);
    }
}

 const char *company_name(uint16_t id)
{
    switch (id)
    {
        case 0x0006:
            return "Microsoft";

        case 0x004C:
            return "Apple";

        case 0x000F:
            return "Broadcom";

        case 0x0075:
            return "Samsung";

        case 0x00E0:
            return "Google";

        case 0x00D2:
            return "Dialog";

        case 0x0131:
            return "Xiaomi";

        case 0x0157:
            return "Espressif";

        default:
            return "Unknown";
    }
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

esp_err_t ble_transport_driver_init(void)
  {
    ble_context_init();

    // memset(&g_ble, 0, sizeof(g_ble));
    esp_err_t ret;

        /*----------------------------------------------------------
        * NVS
        *---------------------------------------------------------*/

        ret = nvs_flash_init();

        if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
            ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());

            ret = nvs_flash_init();
        }

        ESP_ERROR_CHECK(ret);
      
        ret = nimble_port_init();

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG,
                    "nimble_port_init failed");

            return ESP_FAIL ;
        }
        s_connect_retry_timer = xTimerCreate(
            "ble_connect_retry",
            pdMS_TO_TICKS(BLE_CONNECT_RETRY_INITIAL_MS),
            pdFALSE,
            NULL,
            ble_connect_retry_timer_callback);
        if (s_connect_retry_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create BLE reconnect retry timer");
            return ESP_ERR_NO_MEM;
        }
        ble_hs_cfg.reset_cb = ble_on_reset;

        ble_hs_cfg.sync_cb = ble_on_sync;
        
        ble_set_driver_state(BLE_TRANSPORT_STATE_INITIALIZED);
        
        nimble_port_freertos_init(ble_host_task);
  
    ESP_LOGI(TAG, "BLE Transport initialized");

    return ESP_OK;
  }

esp_err_t ble_transport_start(void)
{
    if ((g_ble.driver_state != BLE_TRANSPORT_STATE_INITIALIZED &&
         g_ble.driver_state != BLE_TRANSPORT_STATE_STOPPED) ||
        g_ble.connected)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Transport Start");

    s_connect_retry_pending = false;
    s_connect_retry_delay_ms = BLE_CONNECT_RETRY_INITIAL_MS;
    ble_set_driver_state(BLE_TRANSPORT_STATE_RUNNING);

    ble_set_link_state(BLE_LINK_STATE_SCANNING);

    esp_err_t err = start_scan();
    if (err != ESP_OK) {
        ble_set_driver_state(BLE_TRANSPORT_STATE_STOPPED);
        ble_set_link_state(BLE_LINK_STATE_DISCONNECTED);
    }
    return err;
}

esp_err_t ble_transport_set_target_addr(const uint8_t address[6],
                                        uint8_t address_type)
{
    if (address == NULL || address_type > 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_target_meter_addr, address, sizeof(s_target_meter_addr));
    s_target_address_type = address_type;
    s_target_address_type_valid = true;
    s_target_address_enabled = true;
    s_manual_scan = false;
    portENTER_CRITICAL(&s_ble_scan_mux);
    memset(g_ble.device_name, 0, sizeof(g_ble.device_name));
    portEXIT_CRITICAL(&s_ble_scan_mux);

    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING) {
        return ESP_OK;
    }

    if (g_ble.connected &&
        ((s_target_address_type_valid &&
          g_ble.peer_addr.type != address_type) ||
         memcmp(g_ble.peer_addr.val, address,
                sizeof(g_ble.peer_addr.val)) != 0)) {
        int rc = ble_gap_terminate(g_ble.conn_id, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to disconnect previous BLE target: %d", rc);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    return s_ble_scanning ? ESP_OK : start_scan();
}

esp_err_t ble_transport_reset_target_addr(void)
{
    const bool was_enabled = s_target_address_enabled;
    bool disconnecting = false;
    memset(s_target_meter_addr, 0, sizeof(s_target_meter_addr));
    s_target_address_type = 0U;
    s_target_address_type_valid = false;
    s_target_address_enabled = false;
    s_manual_scan = false;
    portENTER_CRITICAL(&s_ble_scan_mux);
    memset(g_ble.device_name, 0, sizeof(g_ble.device_name));
    portEXIT_CRITICAL(&s_ble_scan_mux);

    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING) {
        return ESP_OK;
    }

    if (g_ble.connected &&
        (!s_target_address_enabled ||
         (s_target_address_type_valid &&
          g_ble.peer_addr.type != s_target_address_type) ||
         memcmp(g_ble.peer_addr.val, s_target_meter_addr,
                sizeof(s_target_meter_addr)) != 0)) {
        int rc = ble_gap_terminate(g_ble.conn_id, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to disconnect previous BLE target: %d", rc);
            return ESP_FAIL;
        }
        disconnecting = true;
    }
    memset(&g_ble.peer_addr, 0, sizeof(g_ble.peer_addr));

    if (disconnecting) {
        return ESP_OK;
    }

    if (was_enabled && s_ble_scanning) {
        (void)ble_gap_disc_cancel();
        s_ble_scanning = false;
    }
    return s_ble_scanning ? ESP_OK : start_scan();
}

esp_err_t ble_transport_get_target_addr(uint8_t address[6],
                                        uint8_t *address_type)
{
    if (address == NULL || address_type == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(address, s_target_meter_addr, sizeof(s_target_meter_addr));
    *address_type = s_target_address_type;
    return ESP_OK;
}

esp_err_t ble_transport_get_device_name(char *name, size_t capacity)
{
    if (name == NULL || capacity == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_ble_scan_mux);
    size_t name_length = 0U;
    while (name_length < sizeof(g_ble.device_name) &&
           g_ble.device_name[name_length] != '\0') {
        ++name_length;
    }
    if (name_length == 0U) {
        portEXIT_CRITICAL(&s_ble_scan_mux);
        return ESP_ERR_NOT_FOUND;
    }
    if (name_length >= capacity) {
        portEXIT_CRITICAL(&s_ble_scan_mux);
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(name, g_ble.device_name, name_length + 1U);
    portEXIT_CRITICAL(&s_ble_scan_mux);
    return ESP_OK;
}

esp_err_t ble_transport_scan_start(void)
{
    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_ble_scan_mux);
    s_ble_scan_result_count = 0U;
    memset(s_ble_scan_results, 0, sizeof(s_ble_scan_results));
    portEXIT_CRITICAL(&s_ble_scan_mux);

    s_manual_scan = true;
    if (s_ble_scanning) {
        return ESP_OK;
    }

    esp_err_t err = start_scan();
    if (err != ESP_OK) {
        s_manual_scan = false;
        ESP_LOGE(TAG, "Failed to start manual BLE scan");
    }
    return err;
}

esp_err_t ble_transport_scan_get_results(
    ble_transport_scan_result_t *results,
    size_t capacity,
    size_t *count,
    bool *scanning)
{
    if (results == NULL || count == NULL || scanning == NULL ||
        capacity == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_ble_scan_mux);
    *count = s_ble_scan_result_count < capacity
        ? s_ble_scan_result_count : capacity;
    memcpy(results, s_ble_scan_results, *count * sizeof(results[0]));
    portEXIT_CRITICAL(&s_ble_scan_mux);
    *scanning = s_ble_scanning;
    return ESP_OK;
}

esp_err_t ble_transport_stop(void)
{
    ESP_LOGI(TAG, "Transport Stop");
    g_ble.ready = false;
    ble_set_driver_state(BLE_TRANSPORT_STATE_STOPPED);
    s_connect_retry_pending = false;
    if (s_connect_retry_timer != NULL) {
        (void)xTimerStop(s_connect_retry_timer, 0U);
    }
    s_manual_scan = false;
    if (s_ble_scanning) {
        const int rc = ble_gap_disc_cancel();
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to stop BLE scan, rc=%d", rc);
            return ESP_FAIL;
        }
        s_ble_scanning = false;
    }

    if (g_ble.link_state == BLE_LINK_STATE_CONNECTING) {
        const int rc = ble_gap_conn_cancel();
        if (rc != 0) {
            ESP_LOGW(TAG, "Could not cancel pending BLE connection, rc=%d",
                     rc);
        }
    }

    if (g_ble.connected) {
        const int rc = ble_gap_terminate(g_ble.conn_id,
                                         BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to disconnect BLE target, rc=%d", rc);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t ble_transport_reconnect(void)
{
    if (g_ble.driver_state != BLE_TRANSPORT_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_ble.connected) {
        int rc = ble_gap_terminate(g_ble.conn_id, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to reconnect to meter, rc=%d", rc);
            return ESP_FAIL;
        }

        g_ble.ready = false;
        ESP_LOGW(TAG, "Disconnecting from meter to recover BLE transport");
        return ESP_OK;
    }

    if (!s_ble_scanning) {
        return start_scan();
    }

    return ESP_OK;
}

esp_err_t ble_transport_write(const uint8_t *data, size_t length)
{
    if (!g_ble.ready || meter_tx_handle == 0) {
        ESP_LOGE(TAG, "Transport not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "TX DLMS Frame (%u bytes):", (unsigned)length);
    print_hex(data, (uint16_t)length);

    int rc;
    if (meter_tx_props & BLE_GATT_CHR_F_WRITE) {
        rc = ble_gattc_write_flat(g_ble.conn_id, meter_tx_handle,
                                  data, length, dlms_write_cb, NULL);
    } else {
        rc = ble_gattc_write_no_rsp_flat(g_ble.conn_id, meter_tx_handle,
                                         data, length);
    }

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send BLE data, rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent DLMS Frame (%u bytes) to Handle %u",
             (unsigned)length, (unsigned)meter_tx_handle);
    return ESP_OK;
}

esp_err_t ble_transport_get_rssi(int8_t *rssi)
{
    if (rssi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_ble.connected) {
        return ESP_ERR_INVALID_STATE;
    }

    return ble_gap_conn_rssi(g_ble.conn_id, rssi) == 0
        ? ESP_OK
        : ESP_FAIL;
}

bool ble_transport_is_ready(void)
  {
    return g_ble.ready;
  }

bool ble_transport_is_idle(void)
{
    return !g_ble.connected &&
           g_ble.link_state != BLE_LINK_STATE_CONNECTING &&
           !s_ble_scanning;
}

esp_err_t ble_transport_set_rx_callback(ble_transport_rx_callback_t callback)
{
        g_ble.rx_callback = callback;
        return ESP_OK;
}



/* ---------- Meter-specific dynamic discovery ---------- */


#define MD_MAX_CHRS 32


static const char MD_SERVICE_UUID[] =

    "b973f2e0-b19e-11e2-9e96-0800200c9a66";


typedef struct {

    const char *name;

    const char *tx_uuid;

    const char *rx_uuid;

} md_profile_t;


/*

 * Prefer pair A.

 * Pair B is only selected if pair A is not present.

 */

static const md_profile_t md_profiles[] = {

    {

        .name = "PAIR_A",

        .tx_uuid = "e973f2e2-b19e-11e2-9e96-0800200c9a66",

        .rx_uuid = "d973f2e1-b19e-11e2-9e96-0800200c9a66",

    },

    {

        .name = "PAIR_B",

        .tx_uuid = "ea73f3e2-b19e-11e2-9e96-0800200c9a66",

        .rx_uuid = "da73f3e1-b19e-11e2-9e96-0800200c9a66",

    },

};


typedef struct {

    uint16_t def_handle;

    uint16_t val_handle;

    uint8_t properties;

    ble_uuid_any_t uuid;

} md_chr_t;


static struct {

    uint16_t svc_start;

    uint16_t svc_end;

    md_chr_t chrs[MD_MAX_CHRS];

    unsigned chr_count;

    bool overflow;

} md;


static int md_service_cb(uint16_t conn_handle,

                         const struct ble_gatt_error *error,

                         const struct ble_gatt_svc *service,

                         void *arg);


static int md_chr_cb(uint16_t conn_handle,

                     const struct ble_gatt_error *error,

                     const struct ble_gatt_chr *chr,

                     void *arg);


static int md_dsc_cb(uint16_t conn_handle,

                     const struct ble_gatt_error *error,

                     uint16_t chr_val_handle,

                     const struct ble_gatt_dsc *dsc,

                     void *arg);


static int md_cccd_cb(uint16_t conn_handle,

                      const struct ble_gatt_error *error,

                      struct ble_gatt_attr *attr,

                      void *arg);


static bool md_uuid_matches(const ble_uuid_t *uuid,

                            const char *expected)

{

    char text[BLE_UUID_STR_LEN];

    ble_uuid_to_str(uuid, text);

    return strcmp(text, expected) == 0;

}


static void md_reset(void)

{

    memset(&md, 0, sizeof(md));


    meter_tx_handle = 0;

    meter_rx_handle = 0;

    meter_cccd_handle = 0;

    meter_tx_props = 0;

    meter_rx_props = 0;


    g_ble.ready = false;

    g_ble.mtu = BLE_TRANSPORT_DEFAULT_MTU;

}


static void md_fail(uint16_t conn_handle, const char *reason)

{

    g_ble.ready = false;

    ESP_LOGE(TAG, "Meter setup failed: %s", reason);


    int rc = ble_gap_terminate(conn_handle,

                               BLE_ERR_REM_USER_CONN_TERM);

    if (rc != 0) {

        ESP_LOGW(TAG, "Terminate start failed rc=%d", rc);

    }

}


static void md_discover_services(uint16_t conn_handle)

{

    int rc = ble_gattc_disc_all_svcs(conn_handle,

                                    md_service_cb, NULL);

    if (rc != 0) {

        ESP_LOGE(TAG, "Service discovery start failed rc=%d", rc);

        md_fail(conn_handle, "Cannot start service discovery");

    }

}


static int md_mtu_cb(uint16_t conn_handle,

                     const struct ble_gatt_error *error,

                     uint16_t mtu,

                     void *arg)

{

    (void)arg;


    if (error->status == 0) {

        g_ble.mtu = mtu;

        ESP_LOGI(TAG, "MTU negotiated: %u", (unsigned)mtu);

    } else {

        ESP_LOGW(TAG, "MTU exchange failed status=%d",

                 error->status);

    }


    md_discover_services(conn_handle);

    return 0;

}


static void md_start(uint16_t conn_handle)

{

    md_reset();


    int rc = ble_gattc_exchange_mtu(conn_handle,

                                    md_mtu_cb, NULL);

    if (rc != 0) {

        ESP_LOGW(TAG, "MTU exchange start failed rc=%d", rc);

        md_discover_services(conn_handle);

    }

}


static int md_service_cb(uint16_t conn_handle,

                         const struct ble_gatt_error *error,

                         const struct ble_gatt_svc *service,

                         void *arg)

{

    (void)arg;


    if (error->status == 0 && service != NULL) {

        char text[BLE_UUID_STR_LEN];


        ESP_LOGI(TAG, "SVC %s start=%u end=%u",

                 ble_uuid_to_str(&service->uuid.u, text),

                 (unsigned)service->start_handle,

                 (unsigned)service->end_handle);


        if (md_uuid_matches(&service->uuid.u, MD_SERVICE_UUID)) {

            /* Do not silently choose between duplicate services. */

            if (md.svc_start != 0) {

                md_fail(conn_handle, "Duplicate target service");

                return BLE_HS_EUNKNOWN;

            }


            md.svc_start = service->start_handle;

            md.svc_end = service->end_handle;

        }

        return 0;

    }


    if (error->status != BLE_HS_EDONE) {

        ESP_LOGE(TAG, "Service discovery status=%d",

                 error->status);

        md_fail(conn_handle, "Service discovery error");

        return 0;

    }


    if (md.svc_start == 0) {

        md_fail(conn_handle, "Target service UUID not found");

        return 0;

    }


    ESP_LOGI(TAG, "Target service range=%u-%u",

             (unsigned)md.svc_start,

             (unsigned)md.svc_end);


    int rc = ble_gattc_disc_all_chrs(conn_handle,

                                    md.svc_start,

                                    md.svc_end,

                                    md_chr_cb, NULL);

    if (rc != 0) {

        ESP_LOGE(TAG, "Characteristic discovery start rc=%d", rc);

        md_fail(conn_handle, "Cannot discover characteristics");

    }

    return 0;

}


static int md_find_chr(const char *uuid)

{

    for (unsigned i = 0; i < md.chr_count; ++i) {

        if (md_uuid_matches(&md.chrs[i].uuid.u, uuid)) {

            return (int)i;

        }

    }

    return -1;

}


static int md_chr_cb(uint16_t conn_handle,

                     const struct ble_gatt_error *error,

                     const struct ble_gatt_chr *chr,

                     void *arg)

{

    (void)arg;


    if (error->status == 0 && chr != NULL) {

        char text[BLE_UUID_STR_LEN];


        ESP_LOGI(TAG, "CHR %s def=%u val=%u props=0x%02X",

                 ble_uuid_to_str(&chr->uuid.u, text),

                 (unsigned)chr->def_handle,

                 (unsigned)chr->val_handle,

                 chr->properties);


        if (md.chr_count >= MD_MAX_CHRS) {

            md.overflow = true;

            return 0;

        }


        md_chr_t *item = &md.chrs[md.chr_count++];

        item->def_handle = chr->def_handle;

        item->val_handle = chr->val_handle;

        item->properties = chr->properties;

        item->uuid = chr->uuid;

        return 0;

    }


    if (error->status != BLE_HS_EDONE || md.overflow) {

        md_fail(conn_handle, "Characteristic discovery error/overflow");

        return 0;

    }


    int tx = -1;

    int rx = -1;

    const md_profile_t *selected = NULL;


    for (unsigned p = 0;

         p < sizeof(md_profiles) / sizeof(md_profiles[0]); ++p) {

        int t = md_find_chr(md_profiles[p].tx_uuid);

        int r = md_find_chr(md_profiles[p].rx_uuid);


        if (t < 0 || r < 0) {

            continue;

        }


        if (!(md.chrs[t].properties &

              (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP))) {

            continue;

        }


        if (!(md.chrs[r].properties & BLE_GATT_CHR_F_NOTIFY)) {

            continue;

        }


        tx = t;

        rx = r;

        selected = &md_profiles[p];

        break;

    }


    if (selected == NULL) {

        md_fail(conn_handle, "No supported UUID pair found");

        return 0;

    }


    meter_tx_handle = md.chrs[tx].val_handle;

    meter_tx_props = md.chrs[tx].properties;

    meter_rx_handle = md.chrs[rx].val_handle;

    meter_rx_props = md.chrs[rx].properties;

    meter_cccd_handle = 0;


    /*

     * Find the next characteristic declaration by handle,

     * rather than assuming array order.

     */

    uint16_t dsc_end = md.svc_end;


    for (unsigned i = 0; i < md.chr_count; ++i) {

        uint16_t next_def = md.chrs[i].def_handle;


        if (next_def > meter_rx_handle &&

            next_def <= dsc_end) {

            dsc_end = (uint16_t)(next_def - 1U);

        }

    }


    ESP_LOGI(TAG, "Selected %s: TX=%u RX=%u",

             selected->name,

             (unsigned)meter_tx_handle,

             (unsigned)meter_rx_handle);


    if (dsc_end <= meter_rx_handle) {

        md_fail(conn_handle, "No descriptor range for selected RX");

        return 0;

    }


    ESP_LOGI(TAG, "RX descriptor range=%u-%u",

             (unsigned)(meter_rx_handle + 1U),

             (unsigned)dsc_end);


    /*

     * NimBLE takes the characteristic value handle as

     * start_handle; do not pass RX + 1 here.

     */

    int rc = ble_gattc_disc_all_dscs(conn_handle,

                                    meter_rx_handle,

                                    dsc_end,

                                    md_dsc_cb, NULL);

    if (rc != 0) {

        ESP_LOGE(TAG, "Descriptor discovery start rc=%d", rc);

        md_fail(conn_handle, "Cannot discover RX descriptors");

    }

    return 0;

}


static int md_dsc_cb(uint16_t conn_handle,

                     const struct ble_gatt_error *error,

                     uint16_t chr_val_handle,

                     const struct ble_gatt_dsc *dsc,

                     void *arg)

{

    (void)arg;


    if (error->status == 0 && dsc != NULL) {

        char text[BLE_UUID_STR_LEN];


        ESP_LOGI(TAG, "DSC %s handle=%u chr=%u",

                 ble_uuid_to_str(&dsc->uuid.u, text),

                 (unsigned)dsc->handle,

                 (unsigned)chr_val_handle);


        if (chr_val_handle == meter_rx_handle &&

            ble_uuid_cmp(&dsc->uuid.u,

                         BLE_UUID16_DECLARE(0x2902)) == 0) {

            meter_cccd_handle = dsc->handle;

        }

        return 0;

    }


    if (error->status != BLE_HS_EDONE) {

        md_fail(conn_handle, "Descriptor discovery error");

        return 0;

    }


    if (meter_cccd_handle == 0) {

        md_fail(conn_handle, "Selected RX has no discovered CCCD");

        return 0;

    }


    const uint8_t enable_notify[2] = {0x01, 0x00};


    ESP_LOGI(TAG, "Enable notify: CCCD=%u",

             (unsigned)meter_cccd_handle);


    int rc = ble_gattc_write_flat(conn_handle,

                                  meter_cccd_handle,

                                  enable_notify,

                                  sizeof(enable_notify),

                                  md_cccd_cb, NULL);

    if (rc != 0) {

        ESP_LOGE(TAG, "CCCD write start rc=%d", rc);

        md_fail(conn_handle, "Cannot start CCCD write");

    }

    return 0;

}


static int md_cccd_cb(uint16_t conn_handle,

                      const struct ble_gatt_error *error,

                      struct ble_gatt_attr *attr,

                      void *arg)

{

    (void)attr;

    (void)arg;


    if (error->status != 0) {

        ESP_LOGE(TAG, "CCCD write failed status=%d (0x%X)",

                 error->status, (unsigned)error->status);


        if (error->status == BLE_HS_ATT_ERR(0x05)) {

            ESP_LOGE(TAG,

                     "Insufficient Authentication: check BLE pairing");

        } else if (error->status == BLE_HS_ATT_ERR(0x0F)) {

            ESP_LOGE(TAG,

                     "Insufficient Encryption: check BLE security");

        }


        md_fail(conn_handle, "CCCD write rejected");

        return 0;

    }


    g_ble.ready = true;


    ESP_LOGI(TAG, "BLE Transport Ready: TX=%u RX=%u CCCD=%u MTU=%u",

             (unsigned)meter_tx_handle,

             (unsigned)meter_rx_handle,

             (unsigned)meter_cccd_handle,

             (unsigned)g_ble.mtu);

    return 0;

}
static void process_incoming_fragment(const uint8_t *data, uint16_t len)

{

    if (data == NULL || len == 0) return;

    if (len > RX_ASSEMBLY_BUF_SIZE) {
        ESP_LOGW(TAG, "Ignoring oversized RX fragment (%u bytes)",
                 (unsigned)len);
        s_rx_assembly_len = 0;
        return;
    }

    // ป้องกัน Buffer Overflow

    if (s_rx_assembly_len > RX_ASSEMBLY_BUF_SIZE - len) {

        ESP_LOGW(TAG, "RX assembly buffer overflow, resetting...");

        s_rx_assembly_len = 0;

    }


    // นำข้อมูลใหม่มาต่อท้าย (Reassemble)

    memcpy(&s_rx_assembly_buf[s_rx_assembly_len], data, len);

    s_rx_assembly_len += len;


    // วนลูปเช็คขอบเขตแพ็กเก็ต HDLC (7E ... 7E)

    while (s_rx_assembly_len > 0) {

        uint16_t start_idx = 0;

        while (start_idx < s_rx_assembly_len && s_rx_assembly_buf[start_idx] != 0x7E) {

            start_idx++;

        }


        if (start_idx >= s_rx_assembly_len) {

            s_rx_assembly_len = 0;

            break;

        }


        if (start_idx > 0) {

            s_rx_assembly_len -= start_idx;

            memmove(s_rx_assembly_buf, &s_rx_assembly_buf[start_idx], s_rx_assembly_len);

        }


        uint16_t end_idx = 1;

        while (end_idx < s_rx_assembly_len && s_rx_assembly_buf[end_idx] != 0x7E) {

            end_idx++;

        }


        // ถ้ายังหาตัวปิด 0x7E ไม่เจอ ให้รอรับท่อนถัดไป

        if (end_idx >= s_rx_assembly_len) {

            break;

        }


        uint16_t complete_packet_len = end_idx + 1;

        if (complete_packet_len < 9U) {
            ESP_LOGD(TAG, "Ignoring short HDLC frame (%u bytes)",
                     (unsigned)complete_packet_len);
            s_rx_assembly_len -= complete_packet_len;
            if (s_rx_assembly_len > 0U) {
                memmove(s_rx_assembly_buf,
                        &s_rx_assembly_buf[complete_packet_len],
                        s_rx_assembly_len);
            }
            continue;
        }

        ESP_LOGI(TAG, "Assembled complete HDLC frame (%u bytes)", complete_packet_len);

        

        // ส่งก้อนที่ประกอบร่างเสร็จแล้วให้ Parser

        dlms_hdlc_parse_and_log(s_rx_assembly_buf, complete_packet_len);


        s_rx_assembly_len -= complete_packet_len;

        if (s_rx_assembly_len > 0) {

            memmove(s_rx_assembly_buf, &s_rx_assembly_buf[complete_packet_len], s_rx_assembly_len);

        }

    }

}