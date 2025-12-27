#ifndef FEATURE_H
#define FEATURE_H

#include "utils/NED.h"
#include <string>

class NED;

class Feature {
public:
    double x, y;
    int id;
    int counter;
    NED* global_pos;

    Feature();  // Default constructor
    Feature(double x, double y, int id = -1, NED* global_position = nullptr);
    std::string to_string() const;
};

#endif // FEATURE_H

