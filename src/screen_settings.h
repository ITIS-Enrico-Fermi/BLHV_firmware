#pragma once

#include "screen.h"
#include "key.h"
#include "icons.h"

#include "lcdgfx.h"
#include "lcdgfx_gui.h"

static const char *settings_menu_items[] = {
    "Tuning",
    "Shutdown",
    "Unlock PS"
};

namespace Screens {
    class Settings: public Screen {
    private:
        LcdGfxMenu menu;
        ScreenSwitcher &switcher;
        bool update_flag = true;

    public:
        Settings(Display &d, ScreenSwitcher &switcher): Screen(d), menu(settings_menu_items, sizeof(settings_menu_items) / sizeof(const char *)), switcher(switcher) {}

        virtual void begin() {
            DisplaySSD1306_128x32_I2C &hardware = display.getHardwareHandle();
            Screen::begin();
            hardware.setFixedFont(ssd1306xled_font8x16);
        }

        virtual void update(void *stp) {
            DisplaySSD1306_128x32_I2C &hardware = display.getHardwareHandle();

            if (update_flag)
                menu.show(hardware);

            update_flag = false;
        };

        virtual void signal(HMI::Action action) {
            if (action == HMI::Action::KNOB_CLOCKWISE)
                menu.down();

            else if (action == HMI::Action::KNOB_COUNTERCLOCKWISE)
                menu.up();

            else if (action == HMI::Action::KNOB_CLICK)
                //switcher.setPage(menu.selection());
                switcher.setPage(1);

            update_flag = true;
        };
    };
}