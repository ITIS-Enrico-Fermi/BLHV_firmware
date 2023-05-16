#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

bool dac_setup();
int dac_write(uint8_t datum);
