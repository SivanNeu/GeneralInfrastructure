#include "IntegralUtils.h"

IntegralErrorVec3::IntegralErrorVec3() : error(Eigen::Vector3d::Zero()), integrand(Eigen::Vector3d::Zero()) {
}

void IntegralErrorVec3::integrate(const Eigen::Vector3d& current_integrand, double dt) {
    error += (integrand + current_integrand) * dt / 2.0;
    integrand = current_integrand;
}

void IntegralErrorVec3::set_zero() {
    error = Eigen::Vector3d::Zero();
    integrand = Eigen::Vector3d::Zero();
}

IntegralError::IntegralError() : error(0.0), integrand(0.0) {
}

void IntegralError::integrate(double current_integrand, double dt) {
    error += (integrand + current_integrand) * dt / 2.0;
    integrand = current_integrand;
}

void IntegralError::set_zero() {
    error = 0.0;
    integrand = 0.0;
}

