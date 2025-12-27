#ifndef NED_H
#define NED_H
#include "general.h"

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include "utils/CommonConstants.h"

class NED {
public:
    int64_t timestamp;
    Vector3d ned;
    Vector3d vel_ned;

    NED(const Vector3d& ned = Vector3d::Zero(), 
        const Vector3d& vel_ned = Vector3d::Zero(), 
        int64_t timestamp = 0);
    void clear();
    std::string to_string() const;
};

#endif // NED_H

