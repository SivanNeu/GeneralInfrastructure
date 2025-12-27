#include "utils/Altitude.h"

Altitude::Altitude(double altitude_amsl, double altitude_relative, int64_t timestamp)
    : timestamp(timestamp), amsl(altitude_amsl), relative(altitude_relative), vertical_speed_estimate(0.0) {
}

