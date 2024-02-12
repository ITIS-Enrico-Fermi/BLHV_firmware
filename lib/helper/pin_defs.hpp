#pragma once
/**
 * A list of ESP32's pinout compliant to schematic pin names.
*/

enum class Pinout {
    PROFILE_3_SW = 5,
    ROT_ENC_A,
    ROT_ENC_B,
    PROFILE_2_SW,
    PROFILE_1_SW,
    REMOTE_TRIGGER_SW,
    ENCODER_SW,
    BUZZER,
    STOP_SW,
    RUN_SW_LED,
    RUN_SW,
    DISPLAY_SDA,
    DISPLAY_SCL,

    REMOTE_TRIGGER_LV = 19,
    SDA,
    SCL,
};