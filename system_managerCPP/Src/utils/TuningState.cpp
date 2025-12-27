#include "utils/TuningState.h"
#include <algorithm>
#include <cctype>

TUNING_STATE tuning_state_from_str(const std::string& label) {
    std::string upper_label = label;
    std::transform(upper_label.begin(), upper_label.end(), upper_label.begin(), ::toupper);
    
    if (upper_label == "NONE") {
        return TUNING_STATE::NONE;
    } else if (upper_label == "HOVER") {
        return TUNING_STATE::HOVER;
    } else if (upper_label == "PITCH") {
        return TUNING_STATE::PITCH;
    } else if (upper_label == "ROLL") {
        return TUNING_STATE::ROLL;
    } else if (upper_label == "YAW") {
        return TUNING_STATE::YAW;
    } else if (upper_label == "THRUST") {
        return TUNING_STATE::THRUST;
    } else if (upper_label == "GUIDANCE") {
        return TUNING_STATE::GUIDANCE;
    } else {
        // printf("string %s not recognized as TUNING_STATE\n", label.c_str());
        return TUNING_STATE::NONE;
    }
}

