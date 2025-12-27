#ifndef QUATERNION_H
#define QUATERNION_H
#include "general.h"

#include <Eigen/Dense>
#include <cstdint>
#include <string>

class Euler;

class Quaternion {
public:
    int64_t timestamp;
    double x, y, z, w;

    Quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0, int64_t timestamp = 0);
    
    Quaternion operator-() const;
    Quaternion operator*(const Quaternion& other) const;
    Quaternion operator/(double scalar) const;
    
    void set(double x, double y, double z, double w, int64_t timestamp = 0);
    Quaternion conjugate() const;
    Quaternion inv() const;
    double dot(const Quaternion& quat) const;
    
    Vector3d rotate_vec(const Vector3d& vec) const;
    Vector3d passive_rotate_vector(double x, double y, double z) const;
    Vector3d active_rotate_vector(double x, double y, double z) const;
    Matrix3d to_rotation_matrix() const;
    Euler to_euler() const;
    
    static Quaternion from_matrix(const Matrix3d& mat);
    static Quaternion from_axis_angle(const Vector3d& axis, double angle);
    static Quaternion from_euler(double roll, double pitch, double yaw);
    static Quaternion euler_to_quat(const Euler& euler);
    static Quaternion euler_to_quat(double roll, double pitch, double yaw);
    static double quat_dot(const Quaternion& quat1, const Quaternion& quat2);
    static Quaternion normalize(const Quaternion& quat);
    static Quaternion quad_diff(const Quaternion& quat_original, const Quaternion& quat_base);
    static Quaternion quat_mult(const Quaternion& quat_1, const Quaternion& quat_2);
    static Quaternion quat_conj(const Quaternion& quat);
    static double magnitude(const Quaternion& quat);
    static Quaternion inverse(const Quaternion& quat);
    static Euler quat_to_euler(const Quaternion& q);
    
    std::string to_string() const;
};

#endif // QUATERNION_H

