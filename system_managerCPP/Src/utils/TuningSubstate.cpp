#include "utils/TuningSubstate.h"
#include <algorithm>
#include <cctype>

TUNING_SUBSTATE tuning_substate_from_str(const std::string& label) {
    std::string upper_label = label;
    std::transform(upper_label.begin(), upper_label.end(), upper_label.begin(), ::toupper);
    
    if (upper_label == "NONE") {
        return TUNING_SUBSTATE::NONE;
    } else if (upper_label == "VELOCITY" || upper_label == "VEL") {
        return TUNING_SUBSTATE::VELOCITY;
    } else if (upper_label == "ACCELERATION" || upper_label == "ACCEL") {
        return TUNING_SUBSTATE::ACCELERATION;
    } else if (upper_label == "PITCH") {
        return TUNING_SUBSTATE::PITCH;
    } else if (upper_label == "ROLL") {
        return TUNING_SUBSTATE::ROLL;
    } else if (upper_label == "ANGLE") {
        return TUNING_SUBSTATE::ANGLE;
    } else if (upper_label == "POSITION" || upper_label == "POS") {
        return TUNING_SUBSTATE::POSITION;
    } else if (upper_label == "HORIZONTAL" || upper_label == "HORIZ") {
        return TUNING_SUBSTATE::HORIZONTAL;
    } else if (upper_label == "VERTICAL" || upper_label == "VERT") {
        return TUNING_SUBSTATE::VERTICAL;
    } else {
        // printf("string %s not recognized as TUNING_SUBSTATE\n", label.c_str());
        return TUNING_SUBSTATE::NONE;
    }
}

