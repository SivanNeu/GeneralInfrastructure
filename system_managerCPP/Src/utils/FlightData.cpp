#include "utils/FlightData.h"
#include "general.h"
#include <sstream>

Flight_Data::Flight_Data()
    : message_count(0), quat_ts(0), quat_ned_bodyfrd(0, 0, 0, 1), altitude_m(0, 0),
      is_armed(false), mode(FLIGHT_MODE::UNKNOWN), imu_ts(0),
      imu_raw_frd(0), imu_ned(0), pos_ned_m(), raw_pos_lla_deg(0),
      filt_pos_lla_deg(0), rpy_rates(Vector3d::Zero()),
      current_thrust(0), rpy(Vector3d::Zero()), custom_mode_id(1),
      throttle(0), heading(0), groundspeed(0), offboardMode(false),
      timestamp(0), local_ts(0), temperature(0), amsl_m(0), local_m(0),
      monotonic_m(0), relative_m(0), terrain_m(0), bottom_clearance_m(0),
      pressure(0), absolute_press_hpa(0), differential_press_hpa(0),
      is_available(false), signal_strength_percent(0), was_available_once(false),
      is_gyrometer_calibration_ok(false), is_accelerometer_calibration_ok(false),
      is_magnetometer_calibration_ok(false), flight_mode(FLIGHT_MODE::UNKNOWN),
      in_air(false) {
}

std::string Flight_Data::to_string() const {
    std::ostringstream oss;
    oss << "Flight Data: state: " << static_cast<int>(mode)
        << " armed: " << is_armed
        << " pos: e: " << pos_ned_m.ned[0]
        << " n: " << pos_ned_m.ned[1]
        << " u: " << pos_ned_m.ned[2];
    return oss.str();
}

void Flight_Data::set_orientation(double x, double y, double z, double w) {
    // Empty implementation
}

void Flight_Data::set_accelerometer(double x, double y, double z) {
    // Empty implementation
}

