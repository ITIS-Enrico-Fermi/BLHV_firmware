#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <atomic>
#include <iostream>

#include <Arduino.h>
#include <Wire.h>

#include <dac.hpp>
#include <adc.hpp>
#include <controller.hpp>
#include <pin_defs.hpp>
#include <helper.hpp>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define EVER ;;

constexpr auto ENDODER_INCREMENT = 50;
controls::PID *pid = nullptr;
std::atomic<bool> ctrlOn, startPressed;
uint16_t encoderVal = 0;
// float target = 0.318; // < target current measured in ADC units

/**
 * @param dir Rotation direction of the encoder. In (-1 | 1)
*/
void encoderCallback(bool A, bool B) {
    auto dir = 1;
    if (A == B) dir = -1;

    auto prevVal = encoderVal;
    encoderVal += dir * ENDODER_INCREMENT;
    if (dir < 0 and encoderVal > prevVal) encoderVal = 0;
    if (dir > 0 and encoderVal < prevVal) encoderVal = (1 << 16);
}

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

void pollingTask(void *pvParams) {
    static uint16_t bvRemoteTrigger = 0;  /** < Bit Vectors */
    static uint16_t bvEncoderA = 0;
    static uint16_t bvEncoderB = 0;
    static uint16_t bvRun = 0;
    static uint16_t bvStop = 0;
    static bool remoteTriggerStatus = false;
    static bool runStatus = false;
    static bool stopStatus = false;
    static bool encoderA = false;
    static bool encoderB = false;

    for (EVER) {
        bvRemoteTrigger = (bvRemoteTrigger << 1) | digitalRead((uint8_t) Pinout::REMOTE_TRIGGER_SW) | 0xe000;
        if (bvRemoteTrigger == 0xf000) {
            digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, remoteTriggerStatus ? HIGH : LOW);
            remoteTriggerStatus = !remoteTriggerStatus;
        }

        bvEncoderA = (bvEncoderA << 1) | digitalRead((uint8_t) Pinout::ROT_ENC_A);
        if ((bvEncoderA | 0xe000) == 0xf000) {
            encoderA = true;
            encoderCallback(encoderA, encoderB);
        }
        else if (bvEncoderA == 0x0001) {
            encoderA = false;
            encoderCallback(encoderA, encoderB);
        }

        bvEncoderB = (bvEncoderB << 1) | digitalRead((uint8_t) Pinout::ROT_ENC_B);
        if ((bvEncoderB | 0xe000) == 0xf000) {
            encoderB = true;
        }
        else if (bvEncoderB == 0x0001) {
            encoderB = false;
        }

        bvRun = (bvRun << 1) | digitalRead((uint8_t) Pinout::RUN_SW) | 0xe000;
        if (bvRun == 0xf000) {
            startPressed = true;
        }

        bvStop = (bvStop << 1) | digitalRead((uint8_t) Pinout::STOP_SW) | 0xe000;
        if (bvStop == 0xf000) {
            digitalWrite((uint8_t) Pinout::RUN_SW_LED, LOW);
            ctrlOn = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    helpers::initAll();
    esp_log_level_set("*", ESP_LOG_DEBUG);
    ctrlOn = false;
    startPressed = false;
    
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

    xTaskCreatePinnedToCore(pollingTask, "pollingTask", 1024, nullptr, 1, nullptr, CORE_ID_PRO);
}

void loop() {
    if (startPressed) {
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, HIGH);
        xTaskCreate(buzzTask, "buzzTask", 512, nullptr, 5, nullptr);
        pid->sampleTarget();
        pid->setOutput(encoderVal);
        ctrlOn = true;
        startPressed = false;
    }

    if (ctrlOn) {
        encoderVal = pid->loopAsync();
        vTaskDelay(pdMS_TO_TICKS(1e3));
    }
    else {
        pid->forward(encoderVal);
        vTaskDelay(pdMS_TO_TICKS(1e2));
    }
}