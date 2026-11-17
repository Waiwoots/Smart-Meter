#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

esp_err_t meter_mqtt_start(void);
esp_err_t meter_mqtt_stop(void);
esp_err_t meter_mqtt_publish(const double *values, const bool *valid,
                             size_t count);
