#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/adc_types.h"

#include "adc/adc.h"
#include "dac/dac.h"
#include "utils/utils.h"

static float target = 0.5;    // < target current measured in ADC units
static struct {
  float kp;
  float ki;
  float kd;
  float sat_min;  /** Saturation lower bound */
  float sat_max;  /** Saturation upper bound */
} pid_tuning = {
  .kp = 2,
  .ki = 5e-3,
  .kd = 0,
  .sat_min = -1,
  .sat_max = 1
};

/**
 * Compute actuator output to reach the set point.
 * 
 * @param setpoint the desidered value to reach
 * @param processvar current process variable value, in the same unit as setpoint
 * @return an integer representing actuator output, to be converted to a suitable unit
 * for your actuator. Unit: same as input.
*/
float pid_compensator(float setpoint, float processvar) {
  // State variables
  float error = setpoint - processvar;  // < e(t) = error at current time
  static float integralerror = 0;
  static float lasterror = 0;

  // Proportional component
  float prop = pid_tuning.kp * error;

  // Integral component
  integralerror += error;
  float integ = pid_tuning.ki * integralerror;

  // Derivative component
  float deriv = pid_tuning.kd * (error - lasterror);
  lasterror = error;

  return clamp(prop + integ + deriv, pid_tuning.sat_min, pid_tuning.sat_max);
}

/**
 * Converts PID compensator output (floating point) to DAC
 * and clamps result in [0, 255] interval.
*/
int prepare_output(float compensator_out) {
  return clamp(
    normalize(compensator_out, 0, FLT_MAX) * 255.f,
    0,
    255
  );
}

void app_main() {
  float adc_val_normalized;
  int dac_val;
  float pid_out;

  adc_setup();
  dac_setup();

  while(true) {
    adc_val_normalized = clamp(
      normalize(
        adc_read(),
        0, 1 << 12
      ),
      0, 1
    );
    
    pid_out = pid_compensator(target, adc_val_normalized);
    dac_val = pid_out * 255;
    ESP_LOGI("PID", "Reading: %f.2. Output: %d, %f.2", adc_val_normalized, dac_val, pid_out);

    dac_write(dac_val);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
