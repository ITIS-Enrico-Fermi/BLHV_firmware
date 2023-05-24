#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/adc_types.h"

#include "adc/adc.h"
#include "dac/dac.h"
#include "utils/utils.h"

static int target = 2000;    // < target current measured in ADC units

/**
 * Compute actuator output to reach the set point.
 * 
 * @param setpoint the desidered value to reach
 * @param processvar current process variable value, in the same unit as setpoint
 * @return an integer representing actuator output, to be converted to a suitable unit
 * for your actuator. Unit: same as input.
*/
float pid_compensator(int setpoint, int processvar) {
  const float kp = 0.1;
  const float ki = 0.05;
  const float kd = 0;

  // State variables
  int error = setpoint - processvar;  // < e(t) = error at current time
  static float integralerror = 0;
  static int lasterror = 0;

  // Proportional component
  float prop = kp * error;

  // Integral component
  integralerror += error;
  float integ = ki * integralerror;

  // Derivative component
  float deriv = kd * (error - lasterror);
  lasterror = error;

  return prop + integ + deriv;
}

/**
 * Converts PID compensator output (floating point, 12 bits) to DAC
 * and clamps result in [0, 255] interval.
*/
int prepare_output(float compensator_out) {
  return clamp(
    (int) compensator_out >> 4,
    0,
    255
  );
}

void app_main() {
  int adc_val, dac_val;
  float actuator_output;

  adc_setup();
  dac_setup();

  while(true) {
    adc_val = adc_read();
    
    actuator_output = pid_compensator(target, adc_val);
    ESP_LOGI("PID", "Reading: %d. Output: %.3f", adc_val, actuator_output);

    dac_val = prepare_output(actuator_output);
    dac_write(dac_val);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
