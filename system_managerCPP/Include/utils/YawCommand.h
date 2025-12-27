#ifndef YAW_COMMAND_H
#define YAW_COMMAND_H
#include "general.h"

enum class YAW_COMMAND {
    NO_CONTROL = 0,      // no yaw control
    DEFINED_DIR = 1,     // yaw control corresponding to defined direction
    HOLD_CUR_DIR = 2,    // hold current direction
    VELOCITY_DIR = 3,    // direction of drone corresponding to velocity direction
    CAMERA_DIR = 4       // direction of drone corresponding to camera direction
};

#endif // YAW_COMMAND_H

