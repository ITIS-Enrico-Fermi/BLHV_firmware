#include "tasks.h"
#include <error.h>

#define EVER ;;

controls::PID *pid;
std::atomic_bool ctrlOn, startPressed;
uint16_t encoderVal;

void buzzTask(void *pvParams) {
    static SemaphoreHandle_t inUse = nullptr;
    if (not inUse) {
        inUse = xSemaphoreCreateBinary();
        xSemaphoreGive(inUse);
    }

    if (inUse != nullptr and xSemaphoreTake(inUse, 0) == pdFALSE) vTaskDelete(nullptr);

    auto startTick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(200)) {
        digitalWrite((uint8_t) Pinout::BUZZER, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite((uint8_t) Pinout::BUZZER, LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    xSemaphoreGive(inUse);
    vTaskDelete(nullptr);
}

[[noreturn]] void encoderPollingTask(void *pvParams) {
    static uint16_t bvEncoderA = 0;  /** < Bit Vectors */
    static uint16_t bvEncoderB = 0;
    static bool encoderA = false;
    static bool encoderB = false;

    for (EVER) {
        bvEncoderA = (bvEncoderA << 1) | digitalRead((uint8_t) Pinout::ROT_ENC_A);
        if ((bvEncoderA | 0xe000) == 0xf000) {
            encoderA = true;
            encoderCallback(encoderA, encoderB);
        }
        else if ((bvEncoderA | 0x0007) == 0x000f) {
            encoderA = false;
            encoderCallback(encoderA, encoderB);
        }

        bvEncoderB = (bvEncoderB << 1) | digitalRead((uint8_t) Pinout::ROT_ENC_B);
        if ((bvEncoderB | 0xe000) == 0xf000) {
            encoderB = true;
        }
        else if ((bvEncoderB | 0x0007) == 0x000f) {
            encoderB = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

[[noreturn]] void switchesPollingTask(void *pvParams) {
    static uint16_t bvRemoteTrigger = 0;  /** < Bit Vectors */
    static uint16_t bvRun = 0;
    static uint16_t bvStop = 0;
    static bool remoteTriggerStatus = false;
    static bool runStatus = false;
    static bool stopStatus = false;

    for (EVER) {
        bvRemoteTrigger = (bvRemoteTrigger << 1) | digitalRead((uint8_t) Pinout::REMOTE_TRIGGER_SW) | 0xe000;
        if (bvRemoteTrigger == 0xf000) {
            digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, remoteTriggerStatus ? HIGH : LOW);
            remoteTriggerStatus = !remoteTriggerStatus;
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

[[noreturn]] void mainTask(void *pvParams) {
    for (EVER) {
        try {
            if (startPressed) {
                startCallback();
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
        catch (...) {
            printf("Error\n");
            errorHandler();
        }
    }
}