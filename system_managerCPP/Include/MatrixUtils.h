#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H
#include "general.h"

#include <Eigen/Dense>
#include <stdexcept>

// Hat map: converts 3x1 vector to 3x3 skew-symmetric matrix
Matrix3d hat(const Vector3d& x);

// Vee map: converts 3x3 skew-symmetric matrix to 3x1 vector
Vector3d vee(const Matrix3d& x);

// Derivative of unit vector
struct UnitVectorDerivatives {
    Vector3d q;
    Vector3d q_dot;
    Vector3d q_2dot;
};

UnitVectorDerivatives deriv_unit_vector(const Vector3d& A, 
                                        const Vector3d& A_dot, 
                                        const Vector3d& A_2dot);

// Saturate vector between min and max values
Eigen::VectorXd saturate(const Eigen::VectorXd& x, double x_min, double x_max);
Vector3d saturate(const Vector3d& x, double x_min, double x_max);

#endif // MATRIX_UTILS_H

