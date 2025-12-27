#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <string>
#include <tuple>
#include <cmath>

class Utils {
public:
    static std::pair<bool, std::string> get_usb_path_for_device(const std::string& device_str = "", const std::string& device_substr = "");
    static std::tuple<double, double, double> elev_az_to_vector(double elev, double az);
    static std::pair<double, double> vector_to_elev_az(double x, double y, double z);
    static std::pair<double, double> px_to_spherical(double px_i, double px_j);
    static std::pair<double, double> spherical_to_px(double theta, double phi);
    static std::tuple<double, double, double> spherical_to_cartesian(double p, double theta, double phi);
    static std::tuple<double, double, double> cartesian_to_spherical(double x, double y, double z);
    static std::string get_log_dir();
    static std::pair<double, double> rotate(double origin_x, double origin_y, double px, double py, double angle_rad);
    static double angle_diff_deg(double x_deg, double y_deg);
    static double angle_diff_rad(double x, double y);
    static double rolling_avg(double old_data, double new_data, double window_size);
    static double calc_2d_dist(double x1, double y1, double x2, double y2);
    static double normalize(double val, double upper_limit, double lower_limit, bool clip = true);
    static double clip(double val, double lower_limit, double upper_limit);
    static std::string make_exception_msg(const std::string& msg);
    static void print_red(const std::string& msg);
};

#endif // UTILS_H

