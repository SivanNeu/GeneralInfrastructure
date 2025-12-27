#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <Eigen/Dense>
#include <stdexcept>

// Hat map: converts 3x1 vector to 3x3 skew-symmetric matrix
Eigen::Matrix3d hat(const Eigen::Vector3d& x);

// Vee map: converts 3x3 skew-symmetric matrix to 3x1 vector
Eigen::Vector3d vee(const Eigen::Matrix3d& x);

// Derivative of unit vector
struct UnitVectorDerivatives {
    Eigen::Vector3d q;
    Eigen::Vector3d q_dot;
    Eigen::Vector3d q_2dot;
};

UnitVectorDerivatives deriv_unit_vector(const Eigen::Vector3d& A, 
                                        const Eigen::Vector3d& A_dot, 
                                        const Eigen::Vector3d& A_2dot);

// Saturate vector between min and max values
Eigen::VectorXd saturate(const Eigen::VectorXd& x, double x_min, double x_max);
Eigen::Vector3d saturate(const Eigen::Vector3d& x, double x_min, double x_max);

#endif // MATRIX_UTILS_H

