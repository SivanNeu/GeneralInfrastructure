#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <string>
#include <chrono>

class TimeUtils {
public:
    static void sleep_until(double goal_time, double start_time = 0.0);
    static void sleep(double sleep_time);
    static double now();
    static double get_current_time_sec();
    static std::string get_current_day_month_year_str();
    static std::string get_current_time_hour_minute_sec_str();
    static std::string get_utc_time_str();
    static std::string get_unique_datetime_str();
};

#endif // TIME_UTILS_H

