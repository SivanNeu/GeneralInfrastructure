#ifndef LLA_H
#define LLA_H

#include <Eigen/Dense>
#include <cstdint>

class LLA {
public:
    int64_t timestamp;
    Eigen::Vector3d lla;
    Eigen::Vector3d* lla_vel;
    double* relative_alt;

    LLA(int64_t timestamp = 0, const Eigen::Vector3d& lla = Eigen::Vector3d::Zero(), 
        Eigen::Vector3d* lla_vel = nullptr, double* relative_alt = nullptr);
    std::string to_string() const;
};

#endif // LLA_H

