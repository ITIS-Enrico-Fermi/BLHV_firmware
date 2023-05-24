#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Sets up ADC1 on channel 6 for one shot readings, that corresponds to GPIO 34.
 * One shot readings are suitable for low frequency readings.
 * 
 * The ADC readings are 12 bit long.
*/
bool adc_setup();

/**
 * Reads a single value from the ADC. This function is thread-safe.
*/
int adc_read();
