#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// #include "hal/adc_types.h"
// #include "driver/uart.h"

// #include "adc/adc.h"
// #include "dac/dac.h"
// #include "utils/utils.h"


// static float target = 0.17; // < target current measured in ADC units
// static struct {
//     float kp;
//     float ki;
//     float kd;
//     float sat_min; /** Saturation lower bound */
//     float sat_max; /** Saturation upper bound */
// } pid_tuning = {
//     .kp = 0.05,
//     .ki = 0.002,
//     .kd = 0,
//     .sat_min = -0.235,
//     .sat_max = 0.235
// };

// /**
//  * Compute actuator output to reach the set pouint8_t.
//  *
//  * @param setpouint8_t the desidered value to reach
//  * @param processvar current process variable value, in the same unit as setpouint8_t
//  * @return an uint8_teger representing actuator output, to be converted to a suitable unit
//  * for your actuator. Unit: same as input.
//  */
// float pid_compensator(float setpouint8_t, float processvar) {
//     // State variables
//     float error = setpouint8_t - processvar; // < e(t) = error at current time
//     static float uint8_tegralerror = 0;
//     static float lasterror = 0;
//     static float output = 0;

//     // Proportional component
//     float prop = pid_tuning.kp * error;

//     // uint8_tegral component
//     if (output > pid_tuning.sat_min && output < pid_tuning.sat_max) {
//         uint8_tegralerror += error;
//     }
//     float uint8_teg = pid_tuning.ki * uint8_tegralerror;

//     // Derivative component
//     float deriv = pid_tuning.kd * (error - lasterror);
//     lasterror = error;

//     output = prop + uint8_teg + deriv;
//     return clamp(output, pid_tuning.sat_min, pid_tuning.sat_max);
// }

// float adc_val_normalized;
// uint8_t dac_val, adc_val;
// float pid_out;

// uint8_t read_bytes;
// char incoming[100];

// void setup() {
//     adc_setup();
//     dac_setup();

//     // uart_config_t uart_conf = {
//     //     .baud_rate = 115200,
//     //     .data_bits = UART_DATA_8_BITS,
//     //     .parity = UART_PARITY_DISABLE,
//     //     .stop_bits = UART_STOP_BITS_1,
//     //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     //     .rx_flow_ctrl_thresh = 122,
//     //     .source_clk = UART_SCLK_DEFAULT
//     // };
//     // ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_conf));
//     // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
//     // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 2048, 10, NULL, 0));
// }

// void loop() {
//     // read_bytes = uart_read_bytes(UART_NUM_0, incoming, 15, 10 / portTICK_PERIOD_MS);

//     // if (read_bytes > 0) {
//     //     sscanf(incoming, "%f%f%f", &pid_tuning.kp, &pid_tuning.ki, &pid_tuning.kd);
//     //     ESP_LOGI("Tuner", "Read parameters: %.2f, %.2f, %.2f", pid_tuning.kp, pid_tuning.ki, pid_tuning.kd);
//     // }

//     adc_val = adc_read();
//     adc_val_normalized = clamp(
//         normalize(adc_val, 0, 1 << 12),
//         0, 1
//     );

//     pid_out = pid_compensator(target, adc_val_normalized);
//     dac_val = pid_out * 255;
//     ESP_LOGI("PID", "Reading: %d, %.5f. Output: %d, %.2f", adc_val, adc_val_normalized, dac_val, pid_out);

//     dac_write(dac_val);

//     vTaskDelay(100 / portTICK_PERIOD_MS);
// }


#include <Arduino.h>
#include <Wire.h>
#include <dac.hpp>
#include <adc.hpp>
#include <controller.hpp>
#include <pin_defs.hpp>
#include <helper.hpp>

#define EVER ;;

constexpr auto ENDODER_SCALE_K = 50;
controls::PID *pid = nullptr;
uint16_t target = 0;
bool ramp = false, hold = false;

void buzzTask(void *pvParams) {
    static auto inUse = xSemaphoreCreateBinary();
    constexpr auto debounceTime = 1e3;

    if (inUse != nullptr and xSemaphoreTake(inUse, 0) == pdTRUE) vTaskDelete(nullptr);

    auto startTick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(200)) {
        digitalWrite((uint8_t) Pinout::BUZZER, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite((uint8_t) Pinout::BUZZER, LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(debounceTime);
    xSemaphoreGive(inUse);
    vTaskDelete(nullptr);
}


void setup() {
    helpers::initAll();
    esp_log_level_set("*", ESP_LOG_DEBUG);
    
    attachInterrupt((uint8_t) Pinout::RUN_SW, [](){
        // Switch from manual to auto mode: sample Spellman DAC and hold via SPS30 
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, HIGH);
        xTaskCreate(buzzTask, "buzzTask", 512, nullptr, 5, nullptr);
        ramp = true;
        hold = false;
    }, FALLING);

    attachInterrupt((uint8_t) Pinout::STOP_SW, [](){
        // Switch from auto to manual mode
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, LOW);
        digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, LOW);
        ramp = true;
        hold = false;
    }, FALLING);
   
    attachInterrupt((uint8_t) Pinout::REMOTE_TRIGGER_SW, [](){
        digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, HIGH);
    }, FALLING);

    attachInterrupt((uint8_t) Pinout::ENCODER_SW, [](){
        ramp = false;
        hold = true;
    }, FALLING);

    attachInterrupt((uint8_t) Pinout::ROT_ENC_A, [](){
        // auto dir = digitalRead((uint8_t) Pinout::ROT_ENC_B);
        auto dir = 1;
        target += dir;
    }, FALLING);


    auto controllerOut = new dac::DAC8571(Wire);
    auto controllerIn = new adc::MCP3428(Wire);
    pid = new controls::PID(controllerIn, controllerOut);
}

void loop() {
    // Testing DAC: not working
    // digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, HIGH);
    // delay(2000);
    // digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, LOW);

    if (ramp)
        pid->rampAsync(target * ENDODER_SCALE_K);
    
    if (hold)
        pid->holdAsync(target * ENDODER_SCALE_K);
    delay(1e2);
}