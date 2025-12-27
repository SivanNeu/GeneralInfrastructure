#ifndef TUNING_H
#define TUNING_H
#include "general.h"

#include "utils/TuningState.h"
#include "utils/TuningSubstate.h"
#include <string>

class Tuning {
public:
    TUNING_STATE state;
    TUNING_SUBSTATE substate;
    double value;
    double secondary_value;

    Tuning(TUNING_STATE state, TUNING_SUBSTATE substate, double value, double secondary_value = 0.0);
    std::string to_string() const;
};

#endif // TUNING_H

