#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "driver/dac.h"

bool dac_setup() {
    dac_output_enable(DAC_CHANNEL_1);   //GPIO 25

    return true;
}

int dac_write(uint8_t datum) {
    dac_output_voltage(DAC_CHANNEL_1, datum);
    return 1;
}
