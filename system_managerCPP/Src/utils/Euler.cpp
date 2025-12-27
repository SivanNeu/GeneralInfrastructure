#include "utils/Euler.h"
#include "utils/Quaternion.h"
#include <sstream>
#include <cmath>

Euler::Euler(const Eigen::Vector3d& rpy) : rpy(rpy) {
}

void Euler::set(const Eigen::Vector3d& rpy) {
    this->rpy = rpy;
}

Euler Euler::diff(const Euler& e1, const Euler& e2) {
    return Euler(e1.rpy - e2.rpy);
}

std::string Euler::to_string() const {
    std::ostringstream oss;
    const double rad_to_deg = 180.0 / M_PI;
    oss << "roll: " << rpy[0] * rad_to_deg 
        << " pitch: " << rpy[1] * rad_to_deg 
        << " yaw: " << rpy[2] * rad_to_deg;
    return oss.str();
}

Quaternion Euler::to_quat() const {
    double cy = std::cos(rpy[2] * 0.5);
    double sy = std::sin(rpy[2] * 0.5);
    double cr = std::cos(rpy[0] * 0.5);
    double sr = std::sin(rpy[0] * 0.5);
    double cp = std::cos(rpy[1] * 0.5);
    double sp = std::sin(rpy[1] * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return Quaternion::normalize(q);
}

