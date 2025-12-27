#ifndef EULER_H
#define EULER_H

#include <Eigen/Dense>
#include <string>

class Quaternion;

class Euler {
public:
    Eigen::Vector3d rpy;  // roll, pitch, yaw

    Euler(const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());
    void set(const Eigen::Vector3d& rpy);
    static Euler diff(const Euler& e1, const Euler& e2);
    std::string to_string() const;
    Quaternion to_quat() const;
};

#endif // EULER_H

