#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <iostream>

#include <Arduino.h>
#include <Wire.h>

#include <dac.hpp>
#include <adc.hpp>
#include <controller.hpp>
#include <pin_defs.hpp>
#include <helper.hpp>
#include <tasks.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void setup() {
    helpers::initAll();
    esp_log_level_set("*", ESP_LOG_DEBUG);
    ctrlOn = startPressed = false;
    encoderVal = 0;
    
    auto controllerOut = new dac::DAC8571(Wire);
    auto controllerIn = new adc::MCP3428(Wire);
    pid = new controls::PID(controllerIn, controllerOut);
    pid->setTuning((controls::PidTuning) {
        .kp = 0.05,
        .ki = 0,  // TODO: real-life testing demostrates oscillation with 1e-9. Fix it
        .kd = 0.05,
        .satMin = -1e-2,
        .satMax = 1e-2,
        .compMin = -0.00002,
        .compMax = 0.00002
    });

    xTaskCreatePinnedToCore(mainTask, "mainTask", 4096, nullptr, 1, nullptr, CORE_ID_APP);
    xTaskCreatePinnedToCore(encoderPollingTask, "encoderPollingTask", 2048, nullptr, 2, nullptr, CORE_ID_PRO);
    xTaskCreatePinnedToCore(switchesPollingTask, "switchesPollingTask", 2048, nullptr, 3, nullptr, CORE_ID_PRO);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1e2));
}