idf_component_register(
    SRCS
        "dlms_client.c"
        "dlms_codec.c"
        "dlms_obis.c"
        "dlms_hdlc.c"
        "dlms_cosem.c"
        "dlms_link.c"

    INCLUDE_DIRS
        "."
        "include"

    REQUIRES
        esp_common
        ble_transport
)