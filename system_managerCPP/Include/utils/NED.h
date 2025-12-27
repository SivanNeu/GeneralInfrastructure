#ifndef NED_H
#define NED_H

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include "utils/CommonConstants.h"

class NED {
public:
    int64_t timestamp;
    Eigen::Vector3d ned;
    Eigen::Vector3d vel_ned;

    NED(const Eigen::Vector3d& ned = Eigen::Vector3d::Zero(), 
        const Eigen::Vector3d& vel_ned = Eigen::Vector3d::Zero(), 
        int64_t timestamp = 0);
    void clear();
    std::string to_string() const;
};

#endif // NED_H

