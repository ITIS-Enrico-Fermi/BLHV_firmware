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
#include "driver/uart.h"

#include "adc/adc.h"
#include "dac/dac.h"
#include "utils/utils.h"

static float target = 0.17; // < target current measured in ADC units
static struct {
    float kp;
    float ki;
    float kd;
    float sat_min; /** Saturation lower bound */
    float sat_max; /** Saturation upper bound */
} pid_tuning = {
    .kp = 0.05,
    .ki = 0.002,
    .kd = 0,
    .sat_min = -0.235,
    .sat_max = 0.235};

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
    float error = setpoint - processvar; // < e(t) = error at current time
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

void app_main() {
    float adc_val_normalized;
    int dac_val, adc_val;
    float pid_out;

    int read_bytes;
    char incoming[100];

    adc_setup();
    dac_setup();

    uart_config_t uart_conf = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_conf));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 2048, 10, NULL, 0));

    while (true) {
        read_bytes = uart_read_bytes(UART_NUM_0, incoming, 15, 10 / portTICK_PERIOD_MS);

        if (read_bytes > 0) {
            sscanf(incoming, "%f%f%f", &pid_tuning.kp, &pid_tuning.ki, &pid_tuning.kd);
            ESP_LOGI("Tuner", "Read parameters: %.2f, %.2f, %.2f", pid_tuning.kp, pid_tuning.ki, pid_tuning.kd);
        }

        adc_val = adc_read();
        adc_val_normalized = clamp(
            normalize(adc_val, 0, 1 << 12),
            0, 1
        );

        pid_out = pid_compensator(target, adc_val_normalized);
        dac_val = pid_out * 255;
        ESP_LOGI("PID", "Reading: %d, %.5f. Output: %d, %.2f", adc_val, adc_val_normalized, dac_val, pid_out);

        dac_write(dac_val);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
