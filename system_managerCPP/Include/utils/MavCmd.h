#ifndef MAV_CMD_H
#define MAV_CMD_H

enum class MAV_CMD {
    ARM = 0,
    SET_TO_OFFBOARD = 1,
    SET_ATTITUDE_TARGET = 2,
    POS_SETPOINT_DEBUG = 3,
    TAKEOFF = 4
};

#endif // MAV_CMD_H

