#include "utils/LLA.h"
#include "general.h"
#include <sstream>

LLA::LLA(int64_t timestamp, const Vector3d& lla, 
         Vector3d* lla_vel, double* relative_alt)
    : timestamp(timestamp), lla(lla), lla_vel(lla_vel), relative_alt(relative_alt) {
}

std::string LLA::to_string() const {
    std::ostringstream oss;
    oss << "lat: " << lla[0] << " lon: " << lla[1] << " alt: " << lla[2];
    return oss.str();
}

