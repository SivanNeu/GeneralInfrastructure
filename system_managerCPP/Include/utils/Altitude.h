#ifndef ALTITUDE_H
#define ALTITUDE_H
#include "general.h"

#include <cstdint>

class Altitude {
public:
    int64_t timestamp;
    double amsl;
    double relative;
    double vertical_speed_estimate;

    Altitude(double altitude_amsl = 0.0, double altitude_relative = 0.0, int64_t timestamp = 0);
};

#endif // ALTITUDE_H

