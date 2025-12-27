#include "common.h"
#include <stdlib.h>
#include <string.h>

// Vector operations
void vec3_zero(vec3_t* v) {
    v->data[0] = 0.0;
    v->data[1] = 0.0;
    v->data[2] = 0.0;
}

void vec3_set(vec3_t* v, double x, double y, double z) {
    v->data[0] = x;
    v->data[1] = y;
    v->data[2] = z;
}

void vec3_copy(vec3_t* dst, const vec3_t* src) {
    memcpy(dst->data, src->data, 3 * sizeof(double));
}

double vec3_norm(const vec3_t* v) {
    return sqrt(v->data[0] * v->data[0] + 
                v->data[1] * v->data[1] + 
                v->data[2] * v->data[2]);
}

// Quaternion operations
void quaternion_init(quaternion_t* q, double x, double y, double z, double w) {
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = w;
    q->timestamp = 0.0;
}

void quaternion_copy(quaternion_t* dst, const quaternion_t* src) {
    dst->x = src->x;
    dst->y = src->y;
    dst->z = src->z;
    dst->w = src->w;
    dst->timestamp = src->timestamp;
}

quaternion_t quaternion_multiply(const quaternion_t* q1, const quaternion_t* q2) {
    quaternion_t result;
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    result.timestamp = 0.0;
    return result;
}

quaternion_t quaternion_conjugate(const quaternion_t* q) {
    quaternion_t result;
    result.x = -q->x;
    result.y = -q->y;
    result.z = -q->z;
    result.w = q->w;
    result.timestamp = q->timestamp;
    return result;
}

quaternion_t quaternion_normalize(const quaternion_t* q) {
    double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    quaternion_t result;
    if (norm > 1e-10) {
        result.w = q->w / norm;
        result.x = q->x / norm;
        result.y = q->y / norm;
        result.z = q->z / norm;
    } else {
        result.w = 1.0;
        result.x = 0.0;
        result.y = 0.0;
        result.z = 0.0;
    }
    result.timestamp = q->timestamp;
    return result;
}

vec3_t quaternion_rotate_vec(const quaternion_t* q, const vec3_t* v) {
    vec3_t result;
    double v_norm = vec3_norm(v);
    if (v_norm < 1e-10) {
        vec3_zero(&result);
        return result;
    }
    
    // Create quaternion from vector: q_vec = (v, 0)
    quaternion_t q_vec;
    quaternion_init(&q_vec, v->data[0], v->data[1], v->data[2], 0.0);
    
    // Rotate: q * q_vec * q_conjugate
    quaternion_t q_conj = quaternion_conjugate(q);
    quaternion_t temp = quaternion_multiply(q, &q_vec);
    quaternion_t rotated = quaternion_multiply(&temp, &q_conj);
    
    result.data[0] = rotated.x;
    result.data[1] = rotated.y;
    result.data[2] = rotated.z;
    
    return result;
}

quaternion_t quaternion_from_euler(double roll, double pitch, double yaw) {
    quaternion_t q;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.timestamp = 0.0;
    
    return quaternion_normalize(&q);
}

// NED operations
void ned_init(ned_t* ned) {
    vec3_zero(&ned->ned);
    vec3_zero(&ned->vel_ned);
    ned->timestamp = 0.0;
}

void ned_clear(ned_t* ned) {
    vec3_zero(&ned->ned);
    ned->timestamp = 0.0;
}

// LLA operations
void lla_init(lla_t* lla) {
    vec3_zero(&lla->lla);
    lla->lla_vel = NULL;
    lla->relative_alt = NULL;
    lla->timestamp = 0.0;
}

// Altitude operations
void altitude_init(altitude_t* alt, double amsl, double relative) {
    alt->amsl = amsl;
    alt->relative = relative;
    alt->vertical_speed_estimate = 0.0;
    alt->timestamp = 0.0;
}

// IMU operations
void imu_init(imu_t* imu) {
    vec3_zero(&imu->accel);
    vec3_zero(&imu->gyro);
    imu->timestamp = 0.0;
}

// Rate command operations
void rate_cmd_init(rate_cmd_t* cmd, const vec3_t* rpydot) {
    vec3_copy(&cmd->rpydot, rpydot);
}

// Flight data operations
void flight_data_init(flight_data_t* fd) {
    memset(fd, 0, sizeof(flight_data_t));
    quaternion_init(&fd->quat_ned_bodyfrd, 0.0, 0.0, 0.0, 1.0);
    altitude_init(&fd->altitude_m, 0.0, 0.0);
    imu_init(&fd->imu_raw_frd);
    imu_init(&fd->imu_ned);
    ned_init(&fd->pos_ned_m);
    lla_init(&fd->raw_pos_lla_deg);
    lla_init(&fd->filt_pos_lla_deg);
    vec3_zero(&fd->rpy_rates);
    vec3_zero(&fd->rpy);
    fd->mode = FLIGHT_MODE_UNKNOWN;
    fd->custom_mode_id = 1;
}

void flight_data_cleanup(flight_data_t* fd) {
    if (fd->custom_mode_name) {
        free(fd->custom_mode_name);
        fd->custom_mode_name = NULL;
    }
    if (fd->status_text) {
        free(fd->status_text);
        fd->status_text = NULL;
    }
    if (fd->flight_mode) {
        free(fd->flight_mode);
        fd->flight_mode = NULL;
    }
    if (fd->landing_state) {
        free(fd->landing_state);
        fd->landing_state = NULL;
    }
}

// Utility functions
double angle_diff_rad(double a1, double a2) {
    double diff = a1 - a2;
    while (diff > PI) diff -= 2.0 * PI;
    while (diff < -PI) diff += 2.0 * PI;
    return diff;
}

