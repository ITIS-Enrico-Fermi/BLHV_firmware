#pragma once

#include <variant>

namespace HMI {
    enum Action {
        KNOB_CLICK,
        KNOB_CLOCKWISE,
        KNOB_COUNTERCLOCKWISE,
        RECORD,
        PROGRAM1,
        PROGRAM2,
        PROGRAM3,
        START,
        STOP
    };
}
