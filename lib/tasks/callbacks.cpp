#include "tasks.h"

void encoderCallback(bool A, bool B) {
    auto dir = 1;
    if (A == B) dir = -1;

    auto prevVal = encoderVal;
    encoderVal += dir * ENDODER_INCREMENT;
    if (dir < 0 and encoderVal > prevVal) encoderVal = 0;
    if (dir > 0 and encoderVal < prevVal) encoderVal = (1 << 16);
}

void startCallback() {
    digitalWrite((uint8_t) Pinout::RUN_SW_LED, HIGH);
    xTaskCreate(buzzTask, "buzzTask", 1024, nullptr, 5, nullptr);
    pid->sampleTarget();
    pid->setOutput(encoderVal);
    ctrlOn = true;
}
