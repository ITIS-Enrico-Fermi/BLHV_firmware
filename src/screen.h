#pragma once

#include "key.h"
#include "display.h"

/**
 * This class abstracts what's being displayed on the OLED. Can either be custom
 * graphics or a menu.
 * 
 * Screen is a fully virtual class (used like an interface) and exposes some methods that are
 * called by the graphics engine.
 * 
 * Subclasses should encapsulate the screen status and only interact with the code with the
 * functions: `update` that updates the OLED display and `signal` that handles and processes
 * a keystroke or knob rotation.
*/
class Screen {
protected:
    Display &display;
    Screen(Display &d): display(d) {}
public:
    /**
     * Update the display, and the internal status. An opaque pointer is passed with all the necessary
     * data that is specified by the application.
    */
    virtual void update(void * new_status) = 0;
    virtual void signal(HMI::Action action) = 0;
};
