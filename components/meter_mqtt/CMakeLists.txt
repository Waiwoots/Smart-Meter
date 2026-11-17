idf_component_register(
    SRCS "meter_mqtt.c"
    INCLUDE_DIRS "include"
    REQUIRES mqtt
    EMBED_TXTFILES "emqx_ca.pem"
)
