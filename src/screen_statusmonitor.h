#pragma once

#include "screen.h"
#include "key.h"

#include "lcdgfx.h"

#include <optional>

namespace Screens {
    class StatusMonitor: public Screen {
    public:
        struct Status {
            bool running;
            int input_val;
            int output_val;
            int program;
        };

    private:
        Status internal_status;

    public:
        StatusMonitor(Display &d): Screen(d) {}

        virtual void update(void *stp) {
            DisplaySSD1306_128x32_I2C &hardware = display.getHardwareHandle();
            
            if (stp != nullptr) {
                Status *new_status = reinterpret_cast<Status *>(stp);
                internal_status = *new_status;
            }

            hardware.setTextCursor(0, 0);
            hardware.print(internal_status.output_val);

            hardware.setTextCursor(0, 16);
            hardware.print(internal_status.input_val);
        };
        virtual void signal(HMI::Action action) {

        };
    };
}