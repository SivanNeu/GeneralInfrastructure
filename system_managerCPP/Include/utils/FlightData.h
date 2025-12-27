#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include "utils/Quaternion.h"
#include "utils/Altitude.h"
#include "utils/Imu.h"
#include "utils/NED.h"
#include "utils/LLA.h"
#include "utils/FlightMode.h"
#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <map>

struct GatheredData {
    bool euler_ned_bodyfrd = false;
    bool quat_ned_bodyfrd = false;
    bool pos_ned_m = false;
    bool imu_ned = false;
    bool tracker_px = false;
};

class Flight_Data {
public:
    int message_count;
    int64_t quat_ts;
    Quaternion quat_ned_bodyfrd;
    Altitude altitude_m;
    bool is_armed;
    FLIGHT_MODE mode;
    int64_t imu_ts;
    Imu imu_raw_frd;
    Imu imu_ned;
    NED pos_ned_m;
    LLA raw_pos_lla_deg;
    LLA filt_pos_lla_deg;
    Eigen::Vector3d rpy_rates;
    double current_thrust;
    Eigen::Vector3d rpy;
    int custom_mode_id;
    std::string custom_mode_name;
    double throttle;
    double heading;
    double groundspeed;
    bool offboardMode;
    int64_t timestamp;
    int64_t local_ts;
    double temperature;
    double amsl_m;
    double local_m;
    double monotonic_m;
    double relative_m;
    double terrain_m;
    double bottom_clearance_m;
    std::string status_text;
    double pressure;
    double absolute_press_hpa;
    double differential_press_hpa;
    bool is_available;
    double signal_strength_percent;
    bool was_available_once;
    bool is_gyrometer_calibration_ok;
    bool is_accelerometer_calibration_ok;
    bool is_magnetometer_calibration_ok;
    FLIGHT_MODE flight_mode;
    bool in_air;
    std::string landing_state;
    GatheredData gathered;

    Flight_Data();
    std::string to_string() const;
    void set_orientation(double x, double y, double z, double w);
    void set_accelerometer(double x, double y, double z);
};

#endif // FLIGHT_DATA_H

