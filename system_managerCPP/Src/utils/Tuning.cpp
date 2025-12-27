#include "utils/Tuning.h"
#include <sstream>

Tuning::Tuning(TUNING_STATE state, TUNING_SUBSTATE substate, double value, double secondary_value)
    : state(state), substate(substate), value(value), secondary_value(secondary_value) {
}

std::string Tuning::to_string() const {
    std::ostringstream oss;
    oss << "Tuning state: " << static_cast<int>(state) 
        << " substate: " << static_cast<int>(substate) 
        << " value: " << value;
    return oss.str();
}

