#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / PI)

// Enums
typedef enum {
    PX4_FLIGHT_STATE_OFFBOARD = 393216,
    PX4_FLIGHT_STATE_HOLD = 50593792,
    PX4_FLIGHT_STATE_RETURN = 84148224,
    PX4_FLIGHT_STATE_TAKEOFF = 33816576,
    PX4_FLIGHT_STATE_LAND = 100925440,
    PX4_FLIGHT_STATE_GENERAL = 65535,
    PX4_FLIGHT_STATE_AUTO = 262144,
    PX4_FLIGHT_STATE_ACRO = 327680,
    PX4_FLIGHT_STATE_RATTITUDE = 524288,
    PX4_FLIGHT_STATE_ALTITUDE = 131072,
    PX4_FLIGHT_STATE_POSITION = 196608,
    PX4_FLIGHT_STATE_LOITER = 262147,
    PX4_FLIGHT_STATE_MISSION = 262148,
    PX4_FLIGHT_STATE_MANUAL = 65536,
    PX4_FLIGHT_STATE_STABILIZED = 458752,
    PX4_FLIGHT_STATE_POSITION_SLOW = 33751040,
    PX4_FLIGHT_STATE_SAFE_RECOVERY = 84148224,
    PX4_FLIGHT_STATE_FOLLOW_TARGET = 134479872,
    PX4_FLIGHT_STATE_PRECISION_LAND = 151257088
} px4_flight_state_t;

typedef enum {
    FLIGHT_MODE_UNKNOWN = 0,
    FLIGHT_MODE_OFFBOARD = 1,
    FLIGHT_MODE_STABILIZED = 2,
    FLIGHT_MODE_MANUAL = 3,
    FLIGHT_MODE_ACRO = 4,
    FLIGHT_MODE_RATTITUDE = 5,
    FLIGHT_MODE_ALTCTL = 6,
    FLIGHT_MODE_POSCTL = 7,
    FLIGHT_MODE_LOITER = 8,
    FLIGHT_MODE_MISSION = 9,
    FLIGHT_MODE_RTL = 10,
    FLIGHT_MODE_LAND = 11,
    FLIGHT_MODE_RTGS = 12,
    FLIGHT_MODE_TAKEOFF = 13,
    FLIGHT_MODE_FOLLOWME = 14
} flight_mode_t;

// Vector and matrix types
typedef struct {
    double data[3];
} vec3_t;

typedef struct {
    double data[3][3];
} mat3_t;

// Quaternion structure
typedef struct {
    double x, y, z, w;
    double timestamp;
} quaternion_t;

// NED (North-East-Down) structure
typedef struct {
    vec3_t ned;
    vec3_t vel_ned;
    double timestamp;
} ned_t;

// LLA (Latitude-Longitude-Altitude) structure
typedef struct {
    vec3_t lla;
    vec3_t* lla_vel;  // Optional
    double* relative_alt;  // Optional
    double timestamp;
} lla_t;

// Altitude structure
typedef struct {
    double amsl;
    double relative;
    double vertical_speed_estimate;
    double timestamp;
} altitude_t;

// IMU structure
typedef struct {
    vec3_t accel;
    vec3_t gyro;
    double timestamp;
} imu_t;

// Rate command structure
typedef struct {
    vec3_t rpydot;
} rate_cmd_t;

// Gathered flags structure
typedef struct {
    bool euler_ned_bodyfrd;
    bool quat_ned_bodyfrd;
    bool pos_ned_m;
    bool vel_ned_m;
    bool imu_ned;
    bool tracker_px;
    bool rpy;
    bool rpy_rates;
    bool custom_mode_id;
    bool mode;
    bool relative_m;
    bool amsl_m;
    bool local_m;
    bool monotonic_m;
    bool terrain_m;
    bool bottom_clearance_m;
    bool absolute_press_hpa;
    bool differential_press_hpa;
    bool pressure;
    bool temperature;
} gathered_flags_t;

// Flight Data structure
typedef struct {
    uint32_t message_count;
    double quat_ts;
    quaternion_t quat_ned_bodyfrd;
    altitude_t altitude_m;
    bool is_armed;
    flight_mode_t mode;
    double imu_ts;
    imu_t imu_raw_frd;
    imu_t imu_ned;
    ned_t pos_ned_m;
    lla_t raw_pos_lla_deg;
    lla_t filt_pos_lla_deg;
    vec3_t rpy_rates;
    double current_thrust;
    vec3_t rpy;
    uint32_t custom_mode_id;
    char* custom_mode_name;
    double throttle;
    double heading;
    double groundspeed;
    bool offboardMode;
    double timestamp;
    double local_ts;
    double temperature;
    double amsl_m;
    double local_m;
    double monotonic_m;
    double relative_m;
    double terrain_m;
    double bottom_clearance_m;
    char* status_text;
    double pressure;
    double absolute_press_hpa;
    double differential_press_hpa;
    bool is_available;
    double signal_strength_percent;
    bool was_available_once;
    bool is_gyrometer_calibration_ok;
    bool is_accelerometer_calibration_ok;
    bool is_magnetometer_calibration_ok;
    char* flight_mode;
    bool in_air;
    char* landing_state;
    gathered_flags_t gathered;
} flight_data_t;

// Function declarations
void vec3_zero(vec3_t* v);
void vec3_set(vec3_t* v, double x, double y, double z);
void vec3_copy(vec3_t* dst, const vec3_t* src);
double vec3_norm(const vec3_t* v);

void quaternion_init(quaternion_t* q, double x, double y, double z, double w);
void quaternion_copy(quaternion_t* dst, const quaternion_t* src);
quaternion_t quaternion_multiply(const quaternion_t* q1, const quaternion_t* q2);
quaternion_t quaternion_conjugate(const quaternion_t* q);
quaternion_t quaternion_normalize(const quaternion_t* q);
vec3_t quaternion_rotate_vec(const quaternion_t* q, const vec3_t* v);
quaternion_t quaternion_from_euler(double roll, double pitch, double yaw);

void ned_init(ned_t* ned);
void ned_clear(ned_t* ned);

void lla_init(lla_t* lla);

void altitude_init(altitude_t* alt, double amsl, double relative);

void imu_init(imu_t* imu);

void rate_cmd_init(rate_cmd_t* cmd, const vec3_t* rpydot);

void flight_data_init(flight_data_t* fd);
void flight_data_cleanup(flight_data_t* fd);

// Utility functions
double angle_diff_rad(double a1, double a2);

#endif // COMMON_H

