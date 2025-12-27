#include "utils/NED.h"
#include <sstream>
#include <iomanip>
#include <cmath>

NED::NED(const Eigen::Vector3d& ned, const Eigen::Vector3d& vel_ned, int64_t timestamp)
    : timestamp(timestamp), ned(ned), vel_ned(vel_ned) {
}

void NED::clear() {
    timestamp = 0;
    ned = Eigen::Vector3d::Zero();
    // vel_ned is not cleared
}

std::string NED::to_string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(ROUND_VAL);
    oss << "e: " << ned[0] << " n: " << ned[1] << " u: " << ned[2];
    return oss.str();
}

