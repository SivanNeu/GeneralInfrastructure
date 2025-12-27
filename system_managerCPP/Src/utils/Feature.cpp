#include "utils/Feature.h"
#include <sstream>

Feature::Feature() : x(0.0), y(0.0), id(-1), counter(0), global_pos(nullptr) {
}

Feature::Feature(double x, double y, int id, NED* global_position)
    : x(x), y(y), id(id), counter(0), global_pos(global_position) {
}

std::string Feature::to_string() const {
    std::ostringstream oss;
    oss << x << "," << y << "," << id;
    return oss.str();
}

