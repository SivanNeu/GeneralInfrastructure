#ifndef TUNING_STATE_H
#define TUNING_STATE_H

#include <string>

enum class TUNING_STATE {
    NONE = 0,
    HOVER = 1,
    PITCH = 2,
    ROLL = 3,
    YAW = 4,
    THRUST = 5,
    GUIDANCE = 6
};

TUNING_STATE tuning_state_from_str(const std::string& label);

#endif // TUNING_STATE_H

