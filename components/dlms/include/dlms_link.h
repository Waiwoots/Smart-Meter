#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

void      dlms_link_reset(void);                 /* เรียกก่อนส่ง SNRM */
void      dlms_link_on_rx_control(uint8_t ctrl); /* เรียกทุกครั้งที่รับเฟรม */
uint8_t   dlms_link_next_i_control(void);        /* control สำหรับ I-frame ถัดไป */
esp_err_t dlms_link_send_rr(void);               /* ส่ง RR ขอ segment ถัดไป */
