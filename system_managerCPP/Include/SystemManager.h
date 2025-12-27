#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

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
constexpr int TOPIC_MAVLINK_PORT = 7790;
constexpr const char* TOPIC_MAVLINK_FLIGHT_DATA = "FLIGHT_DATA";
constexpr int TOPIC_GUIDANCE_CMD_PORT = 7793;
constexpr const char* TOPIC_GUIDANCE_CMD_VEL_NED = "quadVelNedCmd";
constexpr const char* TOPIC_GUIDANCE_CMD_ATTITUDE = "quadAttitudeCmd";
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
    Eigen::Vector3d velCmd;
    double yawCmd;
    double yawRateCmd;
    int message_count;
    double message_ts;
    
    // For attitude commands
    double thrustCmd;
    Eigen::Vector3d rpyRateCmd;
    std::vector<double> quatNedDesBodyFrdCmd;
    bool isRate;
    
    // For acceleration commands
    Eigen::Vector3d accCmd;
};

class SystemManager {
private:
    std::string _config_dir;
    std::string _log_dir;
    double _overall_start;
    Eigen::Vector3d _prev_los_ned_dir;
    Eigen::Vector3d _prev_pos_ned;
    int64_t _prev_imu_ts;
    double _prev_ts;
    
    int message_count;
    std::unique_ptr<Logger> _input_logger;
    double dronemass;
    std::optional<Quaternion> prev_quat_ned_desbodyfrd_cmd;
    
    std::optional<double> destHeight;
    Eigen::Vector3d tar_measurement_ned;
    Eigen::Vector3d heading_dir_ned;
    
    std::optional<Eigen::Vector3d> holdonHeading;
    std::optional<Eigen::Vector3d> holdonPos_ned;
    std::optional<double> holdonTime;
    std::optional<Eigen::Vector3d> yawDefinedDir_ned;
    HOMING_STAGE homingStage;
    
    int pointIndex;
    bool offboardEntry;
    
    // Scenario definitions
    Eigen::Vector3d referencePoint;
    Eigen::Vector3d desiredHeadingDir_ned;
    MISSION_TYPE missionType;
    YAW_COMMAND yawControlType;
    double yawCommandFactor;
    double maximalVelocity;
    double descentVelocity;
    double targetVelocity;
    Eigen::Vector3d originOffset_frd;
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
    std::optional<Eigen::Vector3d> dest_pos_ned;
    
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
                        const Eigen::Vector3d& command, const Eigen::Vector3d& rpy_rate_cmd,
                        const Quaternion& quat_ned_desbodyfrd_cmd, double step_dt, int counter,
                        const Eigen::Vector3d& destination_ned, int current_mode);
    Eigen::Vector3d _get_los_ned_dir(const Eigen::Vector2d& track_pos_px,
                                     const Quaternion& quat_ned_bodyfrd,
                                     const Eigen::Vector2d& img_size_px,
                                     const Eigen::Vector2d& camera_fov_vec,
                                     const Quaternion& quat_bodyfrd_cam);
    std::tuple<std::vector<bool>, std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>,
              Eigen::Vector3d, Quaternion, double> setDesiredState(double curTime);
    std::optional<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Quaternion, double, double>> 
        generateCommand(const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& trajDest,
                       const std::vector<bool>& controlType, double current_ts, int counter,
                       const Quaternion& quat_ned_bodyfrd);
    CommandMessage publishCommand(const Eigen::Vector3d& command, double yawCmd, double yawCmdRate,
                                 const Eigen::Vector3d& rpyRate_cmd, const Quaternion& quat_ned_desbodyfrd_cmd);
    Quaternion getQuadBodyfrdCamera(double nedYangle);
    std::vector<uint8_t> _serialize_vel_cmd(const Eigen::Vector3d& vel_cmd, double yaw_cmd, double yaw_rate_cmd);
    std::vector<uint8_t> _serialize_attitude_cmd(double thrust_cmd, const Eigen::Vector3d& rpy_rate_cmd,
                                                 const Quaternion& quat_cmd, bool is_rate);
    Flight_Data _deserialize_flight_data(const std::vector<uint8_t>& data_bytes);
    void gatherData();

public:
    SystemManager(const std::string& config_dir, const std::string& log_dir,
                 double currentTime = -1.0);
    ~SystemManager();
    
    CommandMessage sys_manager_step(int counter = -1, bool log_data = true,
                                   Flight_Data* flight_Data = nullptr, double curTime = -1.0);
};

#endif // SYSTEM_MANAGER_H

