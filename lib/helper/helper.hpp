#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "pin_defs.hpp"

namespace helpers {
    inline void initAll() noexcept {
        pinMode((uint8_t) Pinout::PROFILE_1_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::PROFILE_2_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::PROFILE_3_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::ROT_ENC_A, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::ROT_ENC_B, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::ENCODER_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::RUN_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::STOP_SW, INPUT_PULLUP);
        pinMode((uint8_t) Pinout::REMOTE_TRIGGER_SW, INPUT_PULLUP);
        
        pinMode((uint8_t) Pinout::BUZZER, OUTPUT);
        pinMode((uint8_t) Pinout::RUN_SW_LED, OUTPUT);
        pinMode((uint8_t) Pinout::REMOTE_TRIGGER_LV, OUTPUT);

        digitalWrite((uint8_t) Pinout::BUZZER, LOW);
        digitalWrite((uint8_t) Pinout::RUN_SW_LED, LOW);
        digitalWrite((uint8_t) Pinout::REMOTE_TRIGGER_LV, LOW);
    
        Wire.setPins((uint8_t) Pinout::SDA, (uint8_t) Pinout::SCL);
        Wire.begin();
    }
}