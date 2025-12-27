#include "utils/CameraType.h"
#include <algorithm>
#include <cctype>

CAMERA_TYPE camera_type_from_str(const std::string& label) {
    std::string lower_label = label;
    std::transform(lower_label.begin(), lower_label.end(), lower_label.begin(), ::tolower);
    
    if (lower_label == "gazebo") {
        return CAMERA_TYPE::GAZEBO;
    } else if (lower_label == "airsim") {
        return CAMERA_TYPE::AIRSIM;
    } else if (lower_label == "usb_h264") {
        return CAMERA_TYPE::USB_H264;
    } else if (lower_label == "replay") {
        return CAMERA_TYPE::REPLAY;
    } else {
        return CAMERA_TYPE::UNKNOWN;
    }
}

