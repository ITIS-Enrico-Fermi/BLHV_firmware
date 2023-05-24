#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "adc.h"

static adc_oneshot_unit_handle_t adc1 = NULL;
static adc_channel_t channel_num;

bool adc_setup() {
    int unit_num;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(34, &unit_num, &channel_num));

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = unit_num,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc1));

    adc_oneshot_chan_cfg_t channel_conf = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, channel_num, &channel_conf));

    return true;
}

int adc_read() {
    int ret_value;
    adc_oneshot_read(adc1, channel_num, &ret_value);
    
    return ret_value;
}
