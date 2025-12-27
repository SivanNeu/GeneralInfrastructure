#ifndef INTEGRAL_UTILS_H
#define INTEGRAL_UTILS_H

#include <Eigen/Dense>

class IntegralErrorVec3 {
public:
    Eigen::Vector3d error;
    Eigen::Vector3d integrand;

    IntegralErrorVec3();
    void integrate(const Eigen::Vector3d& current_integrand, double dt);
    void set_zero();
};

class IntegralError {
public:
    double error;
    double integrand;

    IntegralError();
    void integrate(double current_integrand, double dt);
    void set_zero();
};

#endif // INTEGRAL_UTILS_H

