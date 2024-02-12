#pragma once

#include "screen.h"
#include "key.h"
#include "icons.h"

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

            hardware.setFixedFont(ssd1306xled_font8x16);

            //  Print DAC value
            hardware.setTextCursor(0, 0);
            hardware.write("DAC: ");
            hardware.print(internal_status.output_val);

            //  Print ADC value
            hardware.setTextCursor(0, 16);
            hardware.write("ADC: ");
            hardware.print(internal_status.input_val);

            //  Print set program
            hardware.setFixedFont(courier_new_font11x16_digits);
            //todo: print P for program
            static char prog_buf[2];
            sprintf(prog_buf, "%d", internal_status.program % 10);
            hardware.printFixed(100, 10, prog_buf);

            //  Print start stop icons
            hardware.fillRect({{120, 10}, {120+8, 10+8}});
            hardware.drawBuffer1(
                120, 10, 8, 8,
                internal_status.running ? HMI::Icons::start : HMI::Icons::pause
            );

        };
        virtual void signal(HMI::Action action) {

        };
    };
}