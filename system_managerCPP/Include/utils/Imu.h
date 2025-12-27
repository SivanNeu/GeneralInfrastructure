#ifndef IMU_H
#define IMU_H

#include <Eigen/Dense>
#include <cstdint>

class Imu {
public:
    int64_t timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;

    Imu(int64_t timestamp = 0, const Eigen::Vector3d& accel = Eigen::Vector3d::Zero(), 
        const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero());
};

#endif // IMU_H

