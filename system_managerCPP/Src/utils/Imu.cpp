#include "utils/Imu.h"
#include "general.h"

Imu::Imu(int64_t timestamp, const Vector3d& accel, const Vector3d& gyro)
    : timestamp(timestamp), accel(accel), gyro(gyro) {
}

