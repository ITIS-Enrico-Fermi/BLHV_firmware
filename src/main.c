#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/adc_types.h"

#include "adc.h"
#include "dac.h"

int particle_filter(int in) {
    return in;
}

void app_main() {
    int adc_val;
    int filtered_value;

    adc_setup();
    dac_setup();

    while(true) {
        adc_val = adc_read();
        
        filtered_value = particle_filter(adc_val);

        dac_write(filtered_value >> 4);

        ESP_LOGI("ADC", "Reading: %d", adc_val);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
