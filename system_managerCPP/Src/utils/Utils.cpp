#include "utils/Utils.h"
#include "utils/TimeUtils.h"
#include "utils/CommonConstants.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef __has_include
    #if __has_include(<filesystem>)
        #include <filesystem>
        namespace fs = std::filesystem;
    #else
        #include <experimental/filesystem>
        namespace fs = std::experimental::filesystem;
    #endif
#else
    #include <filesystem>
    namespace fs = std::filesystem;
#endif

std::pair<bool, std::string> Utils::get_usb_path_for_device(const std::string& device_str, const std::string& device_substr) {
    if (device_str.empty() && device_substr.empty()) {
        return std::make_pair(false, "");
    }

    // Note: This is a simplified implementation. Full implementation would require
    // executing bash script and parsing output
    return std::make_pair(false, "");
}

std::tuple<double, double, double> Utils::elev_az_to_vector(double elev, double az) {
    double x = std::tan(elev);
    double y = std::tan(az);
    double z = 1.0;
    return std::make_tuple(x, y, z);
}

std::pair<double, double> Utils::vector_to_elev_az(double x, double y, double z) {
    double az = std::atan2(y, z);
    double elev = std::atan2(x, z);
    return std::make_pair(az, elev);
}

std::pair<double, double> Utils::px_to_spherical(double px_i, double px_j) {
    double r = std::sqrt(1.0 - (px_j * px_j));
    double d = std::sqrt((r * r) - (px_i * px_i));
    double phi = std::atan2(px_j, r);
    double theta = std::atan2(d, px_i);
    return std::make_pair(theta, phi);
}

std::pair<double, double> Utils::spherical_to_px(double theta, double phi) {
    double px_j = std::sin(phi);
    double px_i = std::cos(theta) * std::sqrt(1.0 - (px_j * px_j));
    return std::make_pair(px_i, px_j);
}

std::tuple<double, double, double> Utils::spherical_to_cartesian(double p, double theta, double phi) {
    double x = p * std::cos(phi) * std::cos(theta);
    double y = p * std::cos(phi) * std::sin(theta);
    double z = p * std::sin(phi);
    return std::make_tuple(x, y, z);
}

std::tuple<double, double, double> Utils::cartesian_to_spherical(double x, double y, double z) {
    double p = std::sqrt(x*x + y*y + z*z);
    double theta = std::atan2(y, x);
    double phi = std::acos(z/p);
    return std::make_tuple(p, theta, phi);
}

std::string Utils::get_log_dir() {
    // Simplified implementation - would need filesystem operations
    std::string day = TimeUtils::get_current_day_month_year_str();
    std::string curr_time = TimeUtils::get_current_time_hour_minute_sec_str();
    std::string parent = "../";
    std::string log_dir = parent + "logs/" + day + "/" + curr_time;
    
    // Create directory if it doesn't exist
    fs::create_directories(log_dir);
    
    return log_dir;
}

std::pair<double, double> Utils::rotate(double origin_x, double origin_y, double px, double py, double angle_rad) {
    double qx = origin_x + std::cos(angle_rad) * (px - origin_x) - std::sin(angle_rad) * (py - origin_y);
    double qy = origin_y + std::sin(angle_rad) * (px - origin_x) + std::cos(angle_rad) * (py - origin_y);
    return std::make_pair(qx, qy);
}

double Utils::angle_diff_deg(double x_deg, double y_deg) {
    double x = x_deg * M_PI / 180.0;
    double y = y_deg * M_PI / 180.0;
    return std::atan2(std::sin(x-y), std::cos(x-y)) * 180.0 / M_PI;
}

double Utils::angle_diff_rad(double x, double y) {
    return std::atan2(std::sin(x-y), std::cos(x-y));
}

double Utils::rolling_avg(double old_data, double new_data, double window_size) {
    double alpha = 1.0 / window_size;
    return (old_data * (1.0 - alpha)) + (new_data * alpha);
}

double Utils::calc_2d_dist(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt((dx*dx) + (dy*dy));
}

double Utils::normalize(double val, double upper_limit, double lower_limit, bool clip) {
    if (clip) {
        val = Utils::clip(val, lower_limit, upper_limit);
    }
    return val * ((upper_limit - lower_limit) + lower_limit);
}

double Utils::clip(double val, double lower_limit, double upper_limit) {
    if (val > upper_limit) {
        return upper_limit;
    }
    if (val < lower_limit) {
        return lower_limit;
    }
    return val;
}

std::string Utils::make_exception_msg(const std::string& msg) {
    return "Exception: " + msg;
}

void Utils::print_red(const std::string& msg) {
    std::cerr << msg << std::endl;
}

