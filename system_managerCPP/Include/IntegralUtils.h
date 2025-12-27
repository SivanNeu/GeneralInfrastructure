#ifndef INTEGRAL_UTILS_H
#define INTEGRAL_UTILS_H
#include "general.h"

class IntegralErrorVec3 {
public:
    Vector3d error;
    Vector3d integrand;

    IntegralErrorVec3();
    void integrate(const Vector3d& current_integrand, double dt);
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

