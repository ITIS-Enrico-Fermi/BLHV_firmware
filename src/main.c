#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "driver/uart.h"

#include "adc/adc.h"
#include "dac/dac.h"
#include "utils/utils.h"

static int target = 2000;    // < target current measured in ADC units

static float kp = 0.1;
static float ki = 0.05;
static float kd = 0;

/**
 * Compute actuator output to reach the set point.
 * 
 * @param setpoint the desidered value to reach
 * @param processvar current process variable value, in the same unit as setpoint
 * @return an integer representing actuator output, to be converted to a suitable unit
 * for your actuator. Unit: same as input.
*/
float pid_compensator(int setpoint, int processvar) {
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

    int read_bytes;
    char incoming[100];

    adc_setup();
    dac_setup();

    uart_config_t uart_conf = {.baud_rate=115200, .data_bits=UART_DATA_8_BITS, .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1, .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .rx_flow_ctrl_thresh=122, .source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_conf));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 2048, 10, NULL, 0));

    while(true) {
        read_bytes = uart_read_bytes(UART_NUM_0, incoming, 15, 10 / portTICK_PERIOD_MS);

        if(read_bytes > 0) {
            sscanf(incoming, "%f%f%f", &kp, &ki, &kd);
            ESP_LOGI("Tuner", "Read parameters: %.2f, %.2f, %.2f", kp, ki, kd);
        }

        adc_val = adc_read();
        
        actuator_output = pid_compensator(target, adc_val);
        ESP_LOGI("PID", "Reading: %d. Output: %.3f", adc_val, actuator_output);

    dac_val = prepare_output(actuator_output);
    dac_write(dac_val);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
