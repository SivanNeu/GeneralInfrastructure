#ifndef CAMERA_TYPE_H
#define CAMERA_TYPE_H

#include <string>

enum class CAMERA_TYPE {
    GAZEBO = 0,
    AIRSIM = 1,
    UNKNOWN = 2,
    USB_H264 = 3,
    REPLAY = 4
};

CAMERA_TYPE camera_type_from_str(const std::string& label);

#endif // CAMERA_TYPE_H

