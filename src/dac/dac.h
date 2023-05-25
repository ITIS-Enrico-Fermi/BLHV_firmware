#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

bool dac_setup();
int dac_write(uint8_t datum);

#define DAC_I2S_MCLK_PIN    3
#define DAC_I2S_BCLK_PIN    18
#define DAC_I2S_DATA_PIN    19
#define DAC_I2S_WS_PIN      21
