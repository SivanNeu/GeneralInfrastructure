#include "utils/TimeUtils.h"
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

void TimeUtils::sleep_until(double goal_time, double start_time) {
    if (start_time == 0.0) {
        sleep(goal_time);
        return;
    }
    double elapsed_time = now() - start_time;
    if (elapsed_time < goal_time) {
        sleep(goal_time - elapsed_time);
    }
}

void TimeUtils::sleep(double sleep_time) {
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
}

double TimeUtils::now() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

double TimeUtils::get_current_time_sec() {
    return now();
}

std::string TimeUtils::get_current_day_month_year_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&time_t);
    std::ostringstream oss;
    oss << std::put_time(tm, "%Y-%m-%d");
    return oss.str();
}

std::string TimeUtils::get_current_time_hour_minute_sec_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&time_t);
    std::ostringstream oss;
    oss << std::put_time(tm, "%H-%M-%S");
    return oss.str();
}

std::string TimeUtils::get_utc_time_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::gmtime(&time_t);
    std::ostringstream oss;
    oss << std::put_time(tm, "%H:%M:%S");
    return oss.str();
}

std::string TimeUtils::get_unique_datetime_str() {
    std::string day = get_current_day_month_year_str();
    std::string curr_time = get_current_time_hour_minute_sec_str();
    return day + "_" + curr_time;
}

