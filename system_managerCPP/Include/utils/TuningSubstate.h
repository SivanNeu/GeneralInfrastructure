#ifndef TUNING_SUBSTATE_H
#define TUNING_SUBSTATE_H

#include <string>

enum class TUNING_SUBSTATE {
    NONE = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    PITCH = 3,
    ROLL = 4,
    ANGLE = 5,
    POSITION = 6,
    THRUST = 7,
    PITCH_AND_THRUST = 8,
    VERTICAL = 9,
    HORIZONTAL = 10
};

TUNING_SUBSTATE tuning_substate_from_str(const std::string& label);

#endif // TUNING_SUBSTATE_H

