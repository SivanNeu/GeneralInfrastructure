#ifndef COMMON_CONSTANTS_H
#define COMMON_CONSTANTS_H

#include <string>

constexpr int ROUND_VAL = 4;
constexpr double QUEUE_READ_TIMEOUT = 0.1;
constexpr double MUTEX_TIMEOUT_SEC = 0.5;
constexpr int COUNT_BETWEEN_FLUSH_BIN = 10;

constexpr const char* BIN = "BIN";
constexpr const char* TEXT = "";
constexpr const char* PKL = "PKL";
constexpr const char* CSV = "CSV";

constexpr const char* USB_DISK_NAME_LIST[] = {"SanDisk"};

#endif // COMMON_CONSTANTS_H

