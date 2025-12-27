#include "utils/Quaternion.h"
#include "utils/Euler.h"
#include <cmath>
#include <sstream>
#include <algorithm>

Quaternion::Quaternion(double x, double y, double z, double w, int64_t timestamp)
    : timestamp(timestamp), x(x), y(y), z(z), w(w) {
}

Quaternion Quaternion::operator-() const {
    return Quaternion(-x, -y, -z, -w);
}

Quaternion Quaternion::operator*(const Quaternion& other) const {
    double w_res = w * other.w - x * other.x - y * other.y - z * other.z;
    double x_res = w * other.x + x * other.w + y * other.z - z * other.y;
    double y_res = w * other.y - x * other.z + y * other.w + z * other.x;
    double z_res = w * other.z + x * other.y - y * other.x + z * other.w;
    return Quaternion(x_res, y_res, z_res, w_res);
}

Quaternion Quaternion::operator/(double scalar) const {
    return Quaternion(x/scalar, y/scalar, z/scalar, w/scalar);
}

void Quaternion::set(double x, double y, double z, double w, int64_t timestamp) {
    this->timestamp = timestamp;
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::inv() const {
    return Quaternion(-x, -y, -z, w);
}

double Quaternion::dot(const Quaternion& quat) const {
    return x*quat.x + y*quat.y + z*quat.z + w*quat.w;
}

Eigen::Vector3d Quaternion::rotate_vec(const Eigen::Vector3d& vec) const {
    if (vec.norm() == 0) {
        return Eigen::Vector3d::Zero();
    }
    Quaternion v_quat(vec[0], vec[1], vec[2], 0);
    Quaternion res = (*this) * v_quat * conjugate();
    return Eigen::Vector3d(res.x, res.y, res.z);
}

Eigen::Vector3d Quaternion::passive_rotate_vector(double x, double y, double z) const {
    Quaternion v_quat(x, y, z, 0);
    Quaternion q_conjugate = conjugate();
    Quaternion rotated_v = (*this) * v_quat * q_conjugate;
    return Eigen::Vector3d(rotated_v.x, rotated_v.y, rotated_v.z);
}

Eigen::Vector3d Quaternion::active_rotate_vector(double x, double y, double z) const {
    Quaternion v_quat(x, y, z, 0);
    Quaternion q_conjugate = conjugate();
    Quaternion rotated_v = q_conjugate * v_quat * (*this);
    return Eigen::Vector3d(rotated_v.x, rotated_v.y, rotated_v.z);
}

Eigen::Matrix3d Quaternion::to_rotation_matrix() const {
    double q0 = w;
    double q1 = x;
    double q2 = y;
    double q3 = z;
    
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
    double r01 = 2 * (q1 * q2 - q0 * q3);
    double r02 = 2 * (q1 * q3 + q0 * q2);
    
    double r10 = 2 * (q1 * q2 + q0 * q3);
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
    double r12 = 2 * (q2 * q3 - q0 * q1);
    
    double r20 = 2 * (q1 * q3 - q0 * q2);
    double r21 = 2 * (q2 * q3 + q0 * q1);
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1;
    
    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                  r10, r11, r12,
                  r20, r21, r22;
    
    return rot_matrix;
}

Euler Quaternion::to_euler() const {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = std::atan2(t0, t1);
    
    double t2 = +2.0 * (w * y - z * x);
    t2 = std::max(-1.0, std::min(1.0, t2));
    double pitch_y = std::asin(t2);
    
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = std::atan2(t3, t4);
    
    return Euler(Eigen::Vector3d(roll_x, pitch_y, yaw_z));
}

Quaternion Quaternion::from_matrix(const Eigen::Matrix3d& mat) {
    double trace = mat(0,0) + mat(1,1) + mat(2,2);
    
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        double w = 0.25 / s;
        double x = (mat(2,1) - mat(1,2)) * s;
        double y = (mat(0,2) - mat(2,0)) * s;
        double z = (mat(1,0) - mat(0,1)) * s;
        return Quaternion(x, y, z, w);
    } else {
        if (mat(0,0) > mat(1,1) && mat(0,0) > mat(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + mat(0,0) - mat(1,1) - mat(2,2));
            double w = (mat(2,1) - mat(1,2)) / s;
            double x = 0.25 * s;
            double y = (mat(0,1) + mat(1,0)) / s;
            double z = (mat(0,2) + mat(2,0)) / s;
            return Quaternion(x, y, z, w);
        } else if (mat(1,1) > mat(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + mat(1,1) - mat(0,0) - mat(2,2));
            double w = (mat(0,2) - mat(2,0)) / s;
            double x = (mat(0,1) + mat(1,0)) / s;
            double y = 0.25 * s;
            double z = (mat(1,2) + mat(2,1)) / s;
            return Quaternion(x, y, z, w);
        } else {
            double s = 2.0 * std::sqrt(1.0 + mat(2,2) - mat(0,0) - mat(1,1));
            double w = (mat(1,0) - mat(0,1)) / s;
            double x = (mat(0,2) + mat(2,0)) / s;
            double y = (mat(1,2) + mat(2,1)) / s;
            double z = 0.25 * s;
            return Quaternion(x, y, z, w);
        }
    }
}

Quaternion Quaternion::from_axis_angle(const Eigen::Vector3d& axis, double angle) {
    if (axis.norm() == 0) {
        return Quaternion(0, 0, 0, 1);
    }
    
    Eigen::Vector3d axis_norm = axis / axis.norm();
    double w = std::cos(angle/2);
    double x = axis_norm[0] * std::sin(angle/2);
    double y = axis_norm[1] * std::sin(angle/2);
    double z = axis_norm[2] * std::sin(angle/2);
    return Quaternion(x, y, z, w);
}

Quaternion Quaternion::from_euler(double roll, double pitch, double yaw) {
    return euler_to_quat(roll, pitch, yaw);
}

Quaternion Quaternion::euler_to_quat(const Euler& euler) {
    double cy = std::cos(euler.rpy[2] * 0.5);
    double sy = std::sin(euler.rpy[2] * 0.5);
    double cr = std::cos(euler.rpy[0] * 0.5);
    double sr = std::sin(euler.rpy[0] * 0.5);
    double cp = std::cos(euler.rpy[1] * 0.5);
    double sp = std::sin(euler.rpy[1] * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return normalize(q);
}

Quaternion Quaternion::euler_to_quat(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

double Quaternion::quat_dot(const Quaternion& quat1, const Quaternion& quat2) {
    return quat1.x*quat2.x + quat1.y*quat2.y + quat1.z*quat2.z + quat1.w*quat2.w;
}

Quaternion Quaternion::normalize(const Quaternion& quat) {
    double mag = magnitude(quat);
    return Quaternion(quat.x/mag, quat.y/mag, quat.z/mag, quat.w/mag);
}

Quaternion Quaternion::quad_diff(const Quaternion& quat_original, const Quaternion& quat_base) {
    Quaternion conj = quat_conj(quat_original);
    return quat_mult(quat_base, conj);
}

Quaternion Quaternion::quat_mult(const Quaternion& quat_1, const Quaternion& quat_2) {
    double t0 = (quat_1.w*quat_2.w - quat_1.x*quat_2.x - quat_1.y*quat_2.y - quat_1.z*quat_2.z);
    double t1 = (quat_1.w*quat_2.x + quat_1.x*quat_2.w + quat_1.y*quat_2.z - quat_1.z*quat_2.y);
    double t2 = (quat_1.w*quat_2.y - quat_1.x*quat_2.z + quat_1.y*quat_2.w + quat_1.z*quat_2.x);
    double t3 = (quat_1.w*quat_2.z + quat_1.x*quat_2.y - quat_1.y*quat_2.x + quat_1.z*quat_2.w);
    return Quaternion(t1, t2, t3, t0);
}

Quaternion Quaternion::quat_conj(const Quaternion& quat) {
    return Quaternion(-quat.x, -quat.y, -quat.z, quat.w);
}

double Quaternion::magnitude(const Quaternion& quat) {
    return std::sqrt(quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w);
}

Quaternion Quaternion::inverse(const Quaternion& quat) {
    double mag = magnitude(quat);
    Quaternion conj = quat_conj(quat);
    return Quaternion(conj.x/mag, conj.y/mag, conj.z/mag, conj.w/mag);
}

Euler Quaternion::quat_to_euler(const Quaternion& q) {
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double roll_x = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    double pitch_y = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw_z = std::atan2(siny_cosp, cosy_cosp);
    
    return Euler(Eigen::Vector3d(roll_x, pitch_y, yaw_z));
}

std::string Quaternion::to_string() const {
    std::ostringstream oss;
    oss << "x: " << x << " y: " << y << " z: " << z << " w: " << w << " ts: " << timestamp;
    return oss.str();
}

