#ifndef IMU_H
#define IMU_H
#include "general.h"

#include <Eigen/Dense>
#include <cstdint>

class Imu {
public:
    int64_t timestamp;
    Vector3d accel;
    Vector3d gyro;

    Imu(int64_t timestamp = 0, const Vector3d& accel = Vector3d::Zero(), 
        const Vector3d& gyro = Vector3d::Zero());
};

#endif // IMU_H

