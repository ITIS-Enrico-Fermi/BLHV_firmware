#include "error.h"
#include <helper.hpp>


void beep() {
    digitalWrite((uint8_t) Pinout::BUZZER, LOW);
    vTaskDelay(pdMS_TO_TICKS(2));
    digitalWrite((uint8_t) Pinout::BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1));
}

[[noreturn]] void errorHandler() {
    while (true) {
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, HIGH);
        for (int i=0; i<1e2; i++) beep();
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}