#include "dlms_link.h"
#include "esp_log.h"
#include "ble_transport.h"

#define DLMS_SERVER_ADDR 0x03U
#define DLMS_CLIENT_ADDR 0x21U

static const char *TAG = "DLMS_LINK";

static uint8_t s_ns = 0U; /* N(S) ของ client ที่จะส่งครั้งถัดไป */
static uint8_t s_nr = 0U; /* N(S) ที่ client คาดว่าจะได้จากมิเตอร์ครั้งถัดไป */

/* CRC-16/X.25 ที่ DLMS HDLC ใช้ (HCS/FCS) */
static uint16_t hdlc_fcs16(const uint8_t *p, size_t len)
{
    uint16_t fcs = 0xFFFFU;
    while (len--) {
        fcs ^= *p++;
        for (int i = 0; i < 8; i++) {
            fcs = (fcs & 1U) ? (uint16_t)((fcs >> 1) ^ 0x8408U)
                             : (uint16_t)(fcs >> 1);
        }
    }
    return (uint16_t)~fcs;
}

void dlms_link_reset(void)
{
    s_ns = 0U;
    s_nr = 0U;
}

void dlms_link_on_rx_control(uint8_t ctrl)
{
    /* I-frame: bit0 = 0 → อัปเดต N(R) = N(S) ของมิเตอร์ + 1 */
    if ((ctrl & 0x01U) == 0U) {
        uint8_t ns = (ctrl >> 1) & 0x07U;
        s_nr = (ns + 1U) & 0x07U;
    }
}

uint8_t dlms_link_next_i_control(void)
{
    uint8_t c = (uint8_t)((s_nr << 5) | 0x10U | (s_ns << 1));
    s_ns = (s_ns + 1U) & 0x07U;
    return c;
}

esp_err_t dlms_link_send_rr(void)
{
    uint8_t f[9];
    f[0] = 0x7E;
    f[1] = 0xA0;
    f[2] = 0x07;
    f[3] = DLMS_SERVER_ADDR;
    f[4] = DLMS_CLIENT_ADDR;
    f[5] = (uint8_t)((s_nr << 5) | 0x11U);   /* RR + P-bit */

    uint16_t fcs = hdlc_fcs16(&f[1], 5);
    f[6] = (uint8_t)(fcs & 0xFFU);
    f[7] = (uint8_t)(fcs >> 8);
    f[8] = 0x7E;

    ESP_LOGI(TAG, "TX RR control=0x%02X", f[5]);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, f, sizeof(f), ESP_LOG_INFO);

    return ble_transport_write(f, sizeof(f));
}