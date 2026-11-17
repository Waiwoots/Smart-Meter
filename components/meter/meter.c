#ifndef METER_H
#define METER_H

#include "meter_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize Meter Component */
esp_err_t meter_init(void);

/* Reset Meter Data */
esp_err_t meter_reset(void);

/* Update Meter Data */
esp_err_t meter_update(const meter_t *meter);

/* Read Current Meter Data */
esp_err_t meter_get(meter_t *meter);

#ifdef __cplusplus
}
#endif

#endif

#include "meter.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*==========================================================
 * Private Variables
 *=========================================================*/

static meter_t g_meter;

static SemaphoreHandle_t g_meter_mutex = NULL;




/*==========================================================
 * Private Functions
 *=========================================================*/


static void meter_default(meter_t *meter)
{
    memset(meter, 0, sizeof(meter_t));

    meter->status.state = METER_STATE_DISCONNECTED;
}


/*==========================================================
 * Public Functions
 *=========================================================*/


esp_err_t meter_init(void)
{
    if (g_meter_mutex != NULL)
    {
        return ESP_OK;
    }

    meter_default(&g_meter);

    g_meter_mutex = xSemaphoreCreateMutex();

    if (g_meter_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t meter_reset(void)
{
    if (xSemaphoreTake(g_meter_mutex, portMAX_DELAY) != pdTRUE)
{
    return ESP_FAIL;
}

meter_default(&g_meter);

xSemaphoreGive(g_meter_mutex);

return ESP_OK;
}

esp_err_t meter_update(const meter_t *meter)
{
    if (meter == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_meter_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }


    if (xSemaphoreTake(g_meter_mutex, portMAX_DELAY) != pdTRUE)
     {
            return ESP_FAIL;
}

    g_meter = *meter;

    xSemaphoreGive(g_meter_mutex);

    return ESP_OK;
}

esp_err_t meter_get(meter_t *meter)
{
    if (meter == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_meter_mutex, portMAX_DELAY) != pdTRUE)
    {
    return ESP_FAIL;
    }

     *meter = g_meter;

    xSemaphoreGive(g_meter_mutex);

return ESP_OK;
}