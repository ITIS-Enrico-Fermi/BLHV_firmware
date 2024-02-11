#pragma once

#include "lcdgfx.h"
#include "lcdgfx_gui.h"

class Display {
private:
    DisplaySSD1306_128x32_I2C hardware;

public:
    Display(): hardware(-1, {-1, 0x3c, 17, 16, 0}) {}

    void setup() {
        hardware.begin();
        hardware.setFixedFont(ssd1306xled_font8x16);
        hardware.clear();
    }

    /**
     * Prints a debug string
    */
    void test() {
        hardware.write("Hello world");
    }
};
