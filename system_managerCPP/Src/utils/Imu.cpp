#include "utils/Imu.h"

Imu::Imu(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro)
    : timestamp(timestamp), accel(accel), gyro(gyro) {
}

