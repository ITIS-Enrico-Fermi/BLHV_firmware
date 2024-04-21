#pragma once

#include <controller.hpp>
#include <atomic>
#include <helper.hpp>

constexpr auto ENDODER_INCREMENT = 50;
extern controls::PID *pid;
extern std::atomic_bool ctrlOn, startPressed;
extern uint16_t encoderVal;

void buzzTask(void*);
[[noreturn]] void encoderPollingTask(void*);
[[noreturn]] void switchesPollingTask(void*);
[[noreturn]] void mainTask(void*);
void startCallback();
void encoderCallback(bool, bool);
