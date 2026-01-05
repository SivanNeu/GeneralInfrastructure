#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H
#include "general.h"

#include "Control.h"
#include "VelocityPIDController.h"
#include "VelocityRLController.h"
#include "utils/FlightData.h"
#include "utils/Quaternion.h"
#include "utils/Logger.h"
#include "utils/TimeUtils.h"
#include "utils/GeneralFuncs.h"  // Includes TrajectoryResult
#include "utils/ControllerType.h"
#include "utils/YawCommand.h"
#include "utils/PX4FlightState.h"
#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <optional>
#include <map>
#include <chrono>
#include <zmq.hpp>

// ZMQ Topic constants (from zmqTopics.py)
// Python: topicMavlinkPort = 7790, topicMavlinkFlightData = b'FLIGHT_DATA'
constexpr int TOPIC_MAVLINK_PORT = 7790;
constexpr const char* TOPIC_MAVLINK_FLIGHT_DATA = "FLIGHT_DATA";
// Python: topicGuidenceCmdPort = 7793
constexpr int TOPIC_GUIDANCE_CMD_PORT = 7793;
// Python: topicGuidenceCmdVelNed = b'quadVelNedCmd'
constexpr const char* TOPIC_GUIDANCE_CMD_VEL_NED = "quadVelNedCmd";
// Python: topicGuidenceCmdAttitude = b'quadAttitudeCmd'
constexpr const char* TOPIC_GUIDANCE_CMD_ATTITUDE = "quadAttitudeCmd";
// Python: topicGuidenceCmdAcc = b'quadAccCmd'
constexpr const char* TOPIC_GUIDANCE_CMD_ACC = "quadAccCmd";

// Constants
constexpr double GOAL_LOOP_FREQ_HZ = 50.0;
constexpr double GPS_SEARCH_TIMEOUT_SEC = 3.0;
constexpr bool REAL_TIME = true;

enum class MISSION_TYPE {
    NONE = 0,
    WAYPOINT = 1,
    VELOCITY = 2,
    CIRCLE = 3,
    LISSAJOUS = 4,
    TRACKER = 5,
    SECTION = 6,
    SPINNING = 7
};

// TrajectoryResult is defined in GeneralFuncs.h

// Command message structure
struct CommandMessage {
    double ts;
    Vector3d velCmd;
    double yawCmd;
    double yawRateCmd;
    int message_count;
    double message_ts;
    
    // For attitude commands
    double thrustCmd;
    Vector3d rpyRateCmd;
    std::vector<double> quatNedDesBodyFrdCmd;
    bool isRate;
    
    // For acceleration commands
    Vector3d accCmd;
};

class SystemManager {
private:
    std::string _log_dir;
    double _overall_start;
    Vector3d _prev_los_ned_dir;
    Vector3d _prev_pos_ned;
    int64_t _prev_imu_ts;
    double _prev_ts;
    
    int message_count;
    std::unique_ptr<Logger> _input_logger;
    double dronemass;
    std::optional<Quaternion> prev_quat_ned_desbodyfrd_cmd;
    
    std::optional<double> destHeight;
    Vector3d tar_measurement_ned;
    Vector3d heading_dir_ned;
    
    std::optional<Vector3d> holdonHeading;
    std::optional<Vector3d> holdonPos_ned;
    std::optional<double> holdonTime;
    std::optional<Vector3d> yawDefinedDir_ned;
    HOMING_STAGE homingStage;
    
    int pointIndex;
    bool offboardEntry;
    
    // Scenario definitions
    Vector3d referencePoint;
    std::vector<Vector3d> waypointList;  // List of waypoints for WAYPOINT mission type
    double waypointReachThreshold;  // Distance threshold to consider waypoint reached (meters)
    Vector3d desiredHeadingDir_ned;
    MISSION_TYPE missionType;
    YAW_COMMAND yawControlType;
    double yawCommandFactor;
    double maximalVelocity;
    double descentVelocity;
    double targetVelocity;
    Vector3d originOffset_frd;
    bool terminalHomingAlowed;
    double circleRadius;
    CONTROLLER_TYPE controllerType;
    YAW_COMMAND_TYPE yawCommandType;
    
    std::unique_ptr<Control> _controlMain;
    std::unique_ptr<Control> _controlAux;
    bool rateControlEnabled;
    
    // ZMQ sockets
    zmq::context_t zmq_context;
    zmq::socket_t subsSock;
    zmq::socket_t pubSock;
    
    Flight_Data _currentData;
    LLA _current_pos_lla;
    
    double tic;
    std::optional<Vector3d> dest_pos_ned;
    
    // Warning timers
    double _last_missing_data_warning;
    double _last_gather_error_time;
    double _last_no_data_warning_time;
    double _last_none_cmd_warning;
    double _last_invalid_cmd_warning;
    double _last_cmd_send_debug_time;
    int _cmd_send_count;
    
    // Trajectory functions are now in GeneralFuncs.h
    
    // Helper methods
    std::string _get_vehicle_config_file(const std::string& config_dir);
    void _log_input_data(double current_ts, int64_t imu_ts,
                        const Vector3d& command, const Vector3d& rpy_rate_cmd,
                        const Quaternion& quat_ned_desbodyfrd_cmd, double step_dt, int counter,
                        const Vector3d& destination_ned, int current_mode);
    Vector3d _get_los_ned_dir(const Eigen::Vector2d& track_pos_px,
                                     const Quaternion& quat_ned_bodyfrd,
                                     const Eigen::Vector2d& img_size_px,
                                     const Eigen::Vector2d& camera_fov_vec,
                                     const Quaternion& quat_bodyfrd_cam);
    std::tuple<std::vector<bool>, std::tuple<Vector3d, Vector3d, Vector3d>,
              Vector3d, Quaternion, double> setDesiredState(double curTime);
    std::optional<std::tuple<Vector3d, Vector3d, Quaternion, double, double>> 
        generateCommand(const std::tuple<Vector3d, Vector3d, Vector3d>& trajDest,
                       const std::vector<bool>& controlType, double current_ts, int counter,
                       const Quaternion& quat_ned_bodyfrd, bool log_data = true);
    CommandMessage publishCommand(const Vector3d& command, double yawCmd, double yawCmdRate,
                                 const Vector3d& rpyRate_cmd, const Quaternion& quat_ned_desbodyfrd_cmd);
    Quaternion getQuadBodyfrdCamera(double nedYangle);
    std::vector<uint8_t> _serialize_vel_cmd(const Vector3d& vel_cmd, double yaw_cmd, double yaw_rate_cmd);
    std::vector<uint8_t> _serialize_attitude_cmd(double thrust_cmd, const Vector3d& rpy_rate_cmd,
                                                 const Quaternion& quat_cmd, bool is_rate);
    Flight_Data _deserialize_flight_data(const std::vector<uint8_t>& data_bytes);
    void gatherData();

public:
    SystemManager(const std::string& log_dir,
                 double currentTime = -1.0,
                 double dronemass = 0.55,
                 const Vector3d& heading_dir_ned = Vector3d(-1, 1, 0),
                 const Vector3d& desiredHeadingDir_ned = Vector3d(-1, 1, 0),
                 MISSION_TYPE missionType = MISSION_TYPE::WAYPOINT,
                 YAW_COMMAND yawControlType = YAW_COMMAND::DEFINED_DIR,
                 double yawCommandFactor = 1.0,
                 double maximalVelocity = 0.75,
                 double descentVelocity = 10.0,
                 double targetVelocity = 7.5,
                 const Vector3d& originOffset_frd = Vector3d::Zero(),
                 bool terminalHomingAlowed = true,
                 double circleRadius = 5.0,
                 const std::vector<Vector3d>& waypointList = {},
                 double waypointReachThreshold = 0.5,
                 CONTROLLER_TYPE controllerType = CONTROLLER_TYPE::VELOCITYRL,
                 CONTROLLER_TYPE primaryControllerType = CONTROLLER_TYPE::VELOCITYRL,
                 const std::string& primaryControllerParamsFile = "",
                 CONTROLLER_TYPE secondaryControllerType = CONTROLLER_TYPE::VELOCITYPID,
                 const std::string& secondaryControllerParamsFile = "",
                 YAW_COMMAND_TYPE yawCommandType = YAW_COMMAND_TYPE::RATE,
                 bool rateControlEnabled = false);
    ~SystemManager();
    
    CommandMessage sys_manager_step(int counter = -1, bool log_data = true,
                                   Flight_Data* flight_Data = nullptr, double curTime = -1.0);
};

#endif // SYSTEM_MANAGER_H

