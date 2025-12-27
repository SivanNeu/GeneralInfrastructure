#include "MatrixUtils.h"
#include <cmath>

Eigen::Matrix3d hat(const Eigen::Vector3d& x) {
    Eigen::Matrix3d x_hat;
    x_hat << 0.0, -x[2], x[1],
             x[2], 0.0, -x[0],
             -x[1], x[0], 0.0;
    return x_hat;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& x) {
    // Make skew-symmetric if needed
    Eigen::Matrix3d x_skew = (x - x.transpose()) / 2.0;
    Eigen::Vector3d result;
    result << x_skew(2,1), x_skew(0,2), x_skew(1,0);
    return result;
}

UnitVectorDerivatives deriv_unit_vector(const Eigen::Vector3d& A, 
                                        const Eigen::Vector3d& A_dot, 
                                        const Eigen::Vector3d& A_2dot) {
    double nA = A.norm();
    
    if (std::abs(nA) < 1.0e-9) {
        throw std::runtime_error("The 2-norm of A should not be zero");
    }
    
    double nA3 = nA * nA * nA;
    double nA5 = nA3 * nA * nA;
    
    double A_A_dot = A.dot(A_dot);
    
    UnitVectorDerivatives result;
    result.q = A / nA;
    result.q_dot = A_dot / nA - A * (A_A_dot / nA3);
    
    double A_dot_dot = A_dot.dot(A_dot);
    double A_A_2dot = A.dot(A_2dot);
    result.q_2dot = A_2dot / nA
        - A_dot * (2.0 * A_A_dot / nA3)
        - A * ((A_dot_dot + A_A_2dot) / nA3)
        + A * (3.0 * A_A_dot * A_A_dot / nA5);
    
    return result;
}

Eigen::VectorXd saturate(const Eigen::VectorXd& x, double x_min, double x_max) {
    Eigen::VectorXd result = x;
    for (int i = 0; i < result.size(); i++) {
        if (result[i] > x_max) {
            result[i] = x_max;
        } else if (result[i] < x_min) {
            result[i] = x_min;
        }
    }
    return result;
}

Eigen::Vector3d saturate(const Eigen::Vector3d& x, double x_min, double x_max) {
    Eigen::Vector3d result = x;
    for (int i = 0; i < 3; i++) {
        if (result[i] > x_max) {
            result[i] = x_max;
        } else if (result[i] < x_min) {
            result[i] = x_min;
        }
    }
    return result;
}

