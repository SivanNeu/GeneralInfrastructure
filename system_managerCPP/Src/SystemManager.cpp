#include "SystemManager.h"
#include "general.h"
#include "utils/Euler.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <cmath>
#include <cstring>
#include <algorithm>

SystemManager::SystemManager(const std::string& log_dir, double currentTime,
                             double dronemass,
                             const Vector3d& heading_dir_ned_init,
                             const Vector3d& desiredHeadingDir_ned,
                             MISSION_TYPE missionType,
                             YAW_COMMAND yawControlType,
                             double yawCommandFactor,
                             double maximalVelocity,
                             double descentVelocity,
                             double targetVelocity,
                             const Vector3d& originOffset_frd,
                             bool terminalHomingAlowed,
                             double circleRadius,
                             const std::vector<Vector3d>& waypointList,
                             double waypointReachThreshold,
                             CONTROLLER_TYPE controllerType,
                             CONTROLLER_TYPE primaryControllerType,
                             const std::string& primaryControllerParamsFile,
                             CONTROLLER_TYPE secondaryControllerType,
                             const std::string& secondaryControllerParamsFile,
                             YAW_COMMAND_TYPE yawCommandType,
                             bool rateControlEnabled)
    : _log_dir(log_dir),
      _overall_start(currentTime < 0 ? TimeUtils::now() : currentTime),
      _prev_los_ned_dir(Vector3d::Zero()),
      _prev_pos_ned(Vector3d::Zero()),
      _prev_imu_ts(0), _prev_ts(_overall_start),
      message_count(0), dronemass(dronemass),
      destHeight(std::nullopt), tar_measurement_ned(Vector3d::Zero()),
      heading_dir_ned(heading_dir_ned_init.normalized()),
      homingStage(HOMING_STAGE::NONE), pointIndex(0), offboardEntry(false),
      referencePoint(Vector3d::Zero()),
      waypointList(waypointList), waypointReachThreshold(waypointReachThreshold),
      desiredHeadingDir_ned(desiredHeadingDir_ned),
      missionType(missionType),
      yawControlType(yawControlType),
      yawCommandFactor(yawCommandFactor), maximalVelocity(maximalVelocity), descentVelocity(descentVelocity),
      targetVelocity(targetVelocity), originOffset_frd(originOffset_frd),
      terminalHomingAlowed(terminalHomingAlowed), circleRadius(circleRadius),
      controllerType(controllerType),
      yawCommandType(yawCommandType),
      rateControlEnabled(rateControlEnabled),
      zmq_context(1), subsSock(zmq_context, ZMQ_SUB), pubSock(zmq_context, ZMQ_PUB),
      tic(TimeUtils::now()),
      _last_missing_data_warning(0), _last_gather_error_time(0),
      _last_no_data_warning_time(0), _last_none_cmd_warning(0),
      _last_invalid_cmd_warning(0), _last_cmd_send_debug_time(0), _cmd_send_count(0) {
    
    std::cout << "System_Manager: Initializing..." << std::endl;
    
    // Initialize waypoint list if provided
    if (!waypointList.empty()) {
        std::cout << "System_Manager: Loaded " << waypointList.size() << " waypoints" << std::endl;      
        // Set initial waypoint
        if (pointIndex < static_cast<int>(waypointList.size())) {
            referencePoint = waypointList[pointIndex];
        }
    } 
    
    // Initialize controllers with parameters from JSON files
    // Use primaryControllerType (which falls back to controllerType in main.cpp if not specified)
    
    // Initialize main controller
    if (primaryControllerType == CONTROLLER_TYPE::VELOCITYRL) {
        VelocityRLControllerParameters rlParams(dronemass);
        if (!primaryControllerParamsFile.empty()) {
            rlParams = VelocityRLControllerParameters::loadFromJSON(primaryControllerParamsFile);
        }
        auto mainController = std::make_unique<VelocityRLController>(rlParams, maximalVelocity, _overall_start);
        _controlMain = std::make_unique<Control>(_log_dir, mainController.release(), maximalVelocity);
    } else if (primaryControllerType == CONTROLLER_TYPE::VELOCITYPID) {
        VelocityPIDControllerParameters pidParams(dronemass);
        if (!primaryControllerParamsFile.empty()) {
            pidParams = VelocityPIDControllerParameters::loadFromJSON(primaryControllerParamsFile);
        }
        auto mainController = std::make_unique<VelocityPIDController>(pidParams, _overall_start, yawCommandType);
        _controlMain = std::make_unique<Control>(_log_dir, mainController.release(), maximalVelocity);
    }
    
    // Initialize secondary/auxiliary controller
    // If RL is primary, always create PID auxiliary (backward compatibility)
    // Otherwise, create secondary controller only if params file is specified
    if (primaryControllerType == CONTROLLER_TYPE::VELOCITYRL) {
        // Default: use PID as auxiliary when RL is main
        VelocityPIDControllerParameters auxParams(dronemass);
        if (!secondaryControllerParamsFile.empty()) {
            auxParams = VelocityPIDControllerParameters::loadFromJSON(secondaryControllerParamsFile);
        }
        auto auxController = std::make_unique<VelocityPIDController>(auxParams, _overall_start, yawCommandType);
        _controlAux = std::make_unique<Control>(_log_dir, auxController.release(), maximalVelocity);
    } else if (!secondaryControllerParamsFile.empty()) {
        // Secondary controller explicitly specified via params file
        if (secondaryControllerType == CONTROLLER_TYPE::VELOCITYPID) {
            VelocityPIDControllerParameters auxParams(dronemass);
            auxParams = VelocityPIDControllerParameters::loadFromJSON(secondaryControllerParamsFile);
            auto auxController = std::make_unique<VelocityPIDController>(auxParams, _overall_start, yawCommandType);
            _controlAux = std::make_unique<Control>(_log_dir, auxController.release(), maximalVelocity);
        } else if (secondaryControllerType == CONTROLLER_TYPE::VELOCITYRL) {
            VelocityRLControllerParameters auxParams(dronemass);
            auxParams = VelocityRLControllerParameters::loadFromJSON(secondaryControllerParamsFile);
            auto auxController = std::make_unique<VelocityRLController>(auxParams, maximalVelocity, _overall_start);
            _controlAux = std::make_unique<Control>(_log_dir, auxController.release(), maximalVelocity);
        }
    }
    
    // Setup ZMQ subscriber (matches zmqWrapper.py subscribe() function)
    // Python: zmqSub.setsockopt(zmq.CONFLATE, 1)
    subsSock.set(zmq::sockopt::conflate, 1);
    subsSock.set(zmq::sockopt::rcvtimeo, 100);
    // Python: zmqSub.connect("tcp://127.0.0.1:%d" % port)
    std::string sub_addr = "tcp://127.0.0.1:" + std::to_string(TOPIC_MAVLINK_PORT);
    subsSock.connect(sub_addr);
    // Python: zmqSub.setsockopt(zmq.SUBSCRIBE, topic) where topic = b'FLIGHT_DATA'
    subsSock.set(zmq::sockopt::subscribe, TOPIC_MAVLINK_FLIGHT_DATA);
    std::cout << "System_Manager: Subscribed to topic: " << TOPIC_MAVLINK_FLIGHT_DATA 
              << " on port: " << TOPIC_MAVLINK_PORT << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Setup ZMQ publisher (matches zmqWrapper.py publisher() function)
    // Python: zmqPub.bind("tcp://*:%d" % port)
    std::string pub_addr = "tcp://*:" + std::to_string(TOPIC_GUIDANCE_CMD_PORT);
    pubSock.bind(pub_addr);
    std::cout << "System_Manager: Publishing commands on port: " << TOPIC_GUIDANCE_CMD_PORT << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    _current_pos_lla = LLA(0, Vector3d::Zero());
}

SystemManager::~SystemManager() {
    // ZMQ sockets will be closed automatically
}

CommandMessage SystemManager::sys_manager_step(int counter, bool log_data, Flight_Data* flight_Data, double curTime) {
    if (curTime < 0) {
        curTime = TimeUtils::now();
    }
    
    if (flight_Data == nullptr) {
        gatherData();
        
        if (!_currentData.gathered.quat_ned_bodyfrd ||
            !_currentData.gathered.pos_ned_m ||
            !_currentData.gathered.imu_ned) {
            double now = TimeUtils::now();
            if (now - _last_missing_data_warning > 2.0) {
                std::cout << "System_Manager: -return-0- (Missing flight data)" << std::endl;
                std::cout << "  System_Manager: Cannot send commands without flight data" << std::endl;
                _last_missing_data_warning = now;
            }
            return CommandMessage();
        }
    } else {
        _currentData = *flight_Data;
    }
    
    // Set desired state based on mission type
    auto [controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts] = setDesiredState(curTime);
    
    // Determine if we should log based on OFFBOARD mode (logger exists)
    bool should_log = log_data && (_input_logger != nullptr);
    
    // Generate command from controller
    auto cmd_result = generateCommand(trajDest, controlType, current_ts, counter, quat_ned_bodyfrd, should_log);
    if (!cmd_result.has_value()) {
        return CommandMessage();
    }
    
    auto [command, rpyRate_cmd, quat_ned_desbodyfrd_cmd, yawCmd, yawCmdRate] = cmd_result.value();
    
    // Increment message count
    message_count++;
    
    // Publish command
    CommandMessage msg = publishCommand(command, yawCmd, yawCmdRate, rpyRate_cmd, quat_ned_desbodyfrd_cmd);
    
    // Monitoring output
    double monitorTime = 1.0;
    if (TimeUtils::now() - tic >= monitorTime) {
        tic = TimeUtils::now();
        Vector3d missionPoint = (holdonPos_ned.value_or(Vector3d::Zero()) + referencePoint);
        Vector3d deltaPos_ned = missionPoint;
        Vector3d deltaPos_frd = Vector3d::Zero();
        if (quat_ned_bodyfrd.w != 0 || quat_ned_bodyfrd.x != 0 || 
            quat_ned_bodyfrd.y != 0 || quat_ned_bodyfrd.z != 0) {
            deltaPos_ned = missionPoint - pos_ned;
            deltaPos_frd = quat_ned_bodyfrd.inv().rotate_vec(deltaPos_ned);
        }
        
        std::cout << "timestamp: " << _currentData.timestamp 
                  << " referencePoint: " << referencePoint.transpose()
                  << " pos_ned: " << pos_ned.transpose()
                  << " Command: " << command.transpose()
                  << " yawCmd: " << yawCmd
                  << " yawRateCmd: " << rpyRate_cmd[2]
                  << " deltaPos_frd: " << deltaPos_frd.transpose() << std::endl;
    }
    
    if (log_data && _input_logger) {
        Vector3d trajDest_pos_ned = std::get<0>(trajDest);
        _log_input_data(current_ts, _currentData.imu_ts, command, rpyRate_cmd,
                       quat_ned_desbodyfrd_cmd, current_ts - _prev_ts, counter,
                       trajDest_pos_ned, _currentData.custom_mode_id);
    }
    
    _prev_imu_ts = _currentData.imu_ts;
    _prev_ts = current_ts;
    _prev_pos_ned = _currentData.pos_ned_m.ned;
    prev_quat_ned_desbodyfrd_cmd = quat_ned_desbodyfrd_cmd;
    
    return msg;
}

std::tuple<std::vector<bool>, std::tuple<Vector3d, Vector3d, Vector3d>,
          Vector3d, Quaternion, double> SystemManager::setDesiredState(double curTime) {
    
    if (!holdonHeading.has_value()) {
        Vector3d forward(1, 0, 0);
        holdonHeading = _currentData.quat_ned_bodyfrd.rotate_vec(forward);
    }
    if (!holdonPos_ned.has_value()) {
        holdonPos_ned = _currentData.pos_ned_m.ned;
    }
    if (!holdonTime.has_value()) {
        holdonTime = curTime;
    }
    
    Vector3d pos_ned = _currentData.pos_ned_m.ned;
    Quaternion quat_ned_bodyfrd = _currentData.quat_ned_bodyfrd;
    double current_ts = curTime;
    
    std::vector<bool> controlType;
    TrajectoryResult desired_trajectory;
    
    heading_dir_ned = holdonHeading.value();
    if (yawControlType == YAW_COMMAND::HOLD_CUR_DIR && yawDefinedDir_ned.has_value()) {
        heading_dir_ned = yawDefinedDir_ned.value();
    } else if (yawControlType == YAW_COMMAND::VELOCITY_DIR) {
        Eigen::Vector2d vel_2d = _currentData.pos_ned_m.vel_ned.head<2>();
        if (vel_2d.norm() > 0.1) {
            heading_dir_ned = _currentData.pos_ned_m.vel_ned;
            heading_dir_ned[2] = 0;
            heading_dir_ned.normalize();
        }
    } else if (yawControlType == YAW_COMMAND::DEFINED_DIR) {
        heading_dir_ned = desiredHeadingDir_ned;
    }
    
    // Check if current waypoint is reached (for WAYPOINT mission type)
    if (missionType == MISSION_TYPE::WAYPOINT && !waypointList.empty()) {
        Vector3d pos_ned = _currentData.pos_ned_m.ned;
        Vector3d currentWaypoint = holdonPos_ned.value() + referencePoint;
        Vector3d deltaPos = currentWaypoint - pos_ned;
        double distanceToWaypoint = deltaPos.norm();
        
        // If waypoint is reached, advance to next waypoint
        if (distanceToWaypoint < waypointReachThreshold) {
            if (waypointList.size() > 1) {
                pointIndex = (pointIndex + 1) % waypointList.size();
                referencePoint = waypointList[pointIndex];
            }
        }
    }
    
    Vector3d missionPoint = holdonPos_ned.value() + referencePoint;
    
    if (missionType == MISSION_TYPE::WAYPOINT) {
        desired_trajectory = ::pos_point({missionPoint}, heading_dir_ned, _overall_start);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
        heading_dir_ned = desired_trajectory.b1[0];
    } else if (missionType == MISSION_TYPE::VELOCITY) {
        Vector3d missionVelocity = (missionPoint - holdonPos_ned.value()).normalized() * maximalVelocity;
        desired_trajectory = ::vel_point(missionVelocity, heading_dir_ned, _overall_start);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
        heading_dir_ned = desired_trajectory.b1[0];
    } else if (missionType == MISSION_TYPE::CIRCLE) {
        desired_trajectory = ::horz_circle(missionPoint, circleRadius, heading_dir_ned, targetVelocity, _overall_start);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
        heading_dir_ned = desired_trajectory.b1[0];
    } else if (missionType == MISSION_TYPE::LISSAJOUS) {
        desired_trajectory = ::command_Lissajous(-1.0, _overall_start);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
        heading_dir_ned = desired_trajectory.b1[0];
    } else if (missionType == MISSION_TYPE::SECTION) {
        Vector3d startPoint = holdonPos_ned.value();
        Vector3d endPoint = missionPoint;
        desired_trajectory = ::lineConstVel(startPoint, endPoint, maximalVelocity, holdonTime.value(), heading_dir_ned);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
    }
       
    Vector3d trajDest_pos_ned = dest_pos_ned.value_or(Vector3d::Zero());
    trajDest_pos_ned[2] = destHeight.has_value() ? destHeight.value() : _currentData.pos_ned_m.ned[2];
    
    Vector3d trajDest_vel_ned = desired_trajectory.x.size() > 1 ? desired_trajectory.x[1] : Vector3d::Zero();
    Vector3d trajDest_acc_ned = desired_trajectory.x.size() > 2 ? desired_trajectory.x[2] : Vector3d::Zero();
    
    auto trajDest = std::make_tuple(trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned);
    
    return std::make_tuple(controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts);
}

std::optional<std::tuple<Vector3d, Vector3d, Quaternion, double, double>>
SystemManager::generateCommand(const std::tuple<Vector3d, Vector3d, Vector3d>& trajDest,
                              const std::vector<bool>& controlType, double current_ts, int counter,
                              const Quaternion& quat_ned_bodyfrd, bool log_data) {
    
    auto [command, rpyRate_cmd, quat_ned_desbodyfrd_cmd] = _controlMain->get_cmd(
        _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
        _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
        quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
        trajDest, _currentData, std::make_tuple(heading_dir_ned, Vector3d::Zero(), Vector3d::Zero()),
        controlType, log_data, homingStage);
    
    if (_controlAux) {
        auto [commandAux, rpyRate_cmdAux, quat_ned_desbodyfrd_cmdAux] = _controlAux->get_cmd(
            _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
            _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
            quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
            trajDest, _currentData, std::make_tuple(heading_dir_ned, Vector3d::Zero(), Vector3d::Zero()),
            controlType, log_data, homingStage);
        
        Vector3d commandBody = commandAux;
        // Check controller type - would need to get from controller
        // For now, assume VELOCITYRL if _controlAux exists
        if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
            commandBody = quat_ned_bodyfrd.inv().rotate_vec(commandAux);
        }
        command[2] = commandBody[2];
    }
    
    // Limit roll/pitch
    double rollpitchlimit = 20.0 * M_PI / 180.0;
    Euler rpy = quat_ned_desbodyfrd_cmd.to_euler();
    rpy.rpy[0] = std::max(-rollpitchlimit, std::min(rollpitchlimit, rpy.rpy[0]));
    rpy.rpy[1] = std::max(-rollpitchlimit, std::min(rollpitchlimit, rpy.rpy[1]));
    quat_ned_desbodyfrd_cmd = Quaternion::from_euler(rpy.rpy[0], rpy.rpy[1], rpy.rpy[2]);
    
    if (prev_quat_ned_desbodyfrd_cmd.has_value() && 
        prev_quat_ned_desbodyfrd_cmd.value().dot(quat_ned_desbodyfrd_cmd) < 0) {
        quat_ned_desbodyfrd_cmd = -quat_ned_desbodyfrd_cmd;
    }
    
    Vector3d forward_dir_frd = quat_ned_bodyfrd.rotate_vec(Vector3d(1, 0, 0));
    
    double yawCmd = std::numeric_limits<double>::quiet_NaN();
    double yawCmdRate = std::numeric_limits<double>::quiet_NaN();
    
    if (yawCommandType == YAW_COMMAND_TYPE::ANGLE) {
        yawCmd = std::atan2(forward_dir_frd[1], forward_dir_frd[0]);
        if (yawControlType == YAW_COMMAND::NO_CONTROL) {
            yawCmd = _currentData.heading;
        } else {
            yawCmd = std::atan2(heading_dir_ned[1], heading_dir_ned[0]);
        }
    } else if (yawCommandType == YAW_COMMAND_TYPE::RATE) {
        yawCmdRate = rpyRate_cmd[2];
    }
    
    // Validate command
    if (command.norm() == 0 && command != Vector3d::Zero()) {
        // Command is invalid
        double now = TimeUtils::now();
        if (now - _last_none_cmd_warning > 2.0) {
            std::cout << "Warning: command is invalid, skipping send" << std::endl;
            _last_none_cmd_warning = now;
        }
        return std::nullopt;
    }
    
    return std::make_tuple(command, rpyRate_cmd, quat_ned_desbodyfrd_cmd, yawCmd, yawCmdRate);
}

CommandMessage SystemManager::publishCommand(const Vector3d& command, double yawCmd, double yawCmdRate,
                                            const Vector3d& rpyRate_cmd, const Quaternion& quat_ned_desbodyfrd_cmd) {
    CommandMessage msg;
    msg.ts = TimeUtils::now();
    msg.message_count = message_count;
    msg.message_ts = TimeUtils::now();
    
    try {
        if (controllerType == CONTROLLER_TYPE::VELOCITYPID || controllerType == CONTROLLER_TYPE::VELOCITYRL) {
            std::vector<uint8_t> cmd_data = _serialize_vel_cmd(command, yawCmd, yawCmdRate * (controllerType == CONTROLLER_TYPE::VELOCITYRL ? yawCommandFactor : 1.0));
            
            std::string topic = TOPIC_GUIDANCE_CMD_VEL_NED;
            zmq::message_t topic_msg(topic.size());
            std::memcpy(topic_msg.data(), topic.c_str(), topic.size());
            
            zmq::message_t data_msg(cmd_data.size());
            std::memcpy(data_msg.data(), cmd_data.data(), cmd_data.size());
            
            // Send multi-part message: topic (with sndmore) then data (final part)
            // Note: final part should not use dontwait to ensure multi-part message completes atomically
            pubSock.send(topic_msg, zmq::send_flags::sndmore);
            pubSock.send(data_msg);  // Remove dontwait to ensure multi-part message completes
            
            _cmd_send_count++;
            msg.velCmd = command;
            msg.yawCmd = yawCmd;
            msg.yawRateCmd = yawCmdRate * (controllerType == CONTROLLER_TYPE::VELOCITYRL ? yawCommandFactor : 1.0);
        }
    } catch (const std::exception& e) {
        std::cerr << "System_Manager: ERROR sending command: " << e.what() << std::endl;
    }
    
    // Debug output
    double now = TimeUtils::now();
    if (now - _last_cmd_send_debug_time > 2.0) {
        std::string controller_type_str = (controllerType == CONTROLLER_TYPE::VELOCITYPID) ? "VELOCITYPID" :
                                          (controllerType == CONTROLLER_TYPE::VELOCITYRL) ? "VELOCITYRL" : "OTHER";
        std::cout << "System_manager: Sent " << _cmd_send_count << " commands in last 2s. Controller: " 
                  << controller_type_str << ", message_count: " << message_count << std::endl;
        _cmd_send_count = 0;
        _last_cmd_send_debug_time = now;
    }
    
    return msg;
}

// Implement missing methods

std::vector<uint8_t> SystemManager::_serialize_vel_cmd(const Vector3d& vel_cmd, double yaw_cmd, double yaw_rate_cmd) {
    // Magic number: "VELC" = 0x56454C43
    const uint32_t magic = 0x56454C43;
    const uint32_t version = 1;
    
    // Check if yaw/yaw_rate are valid (not NaN)
    uint8_t yaw_valid = (std::isnan(yaw_cmd) ? 0 : 1);
    uint8_t yaw_rate_valid = (std::isnan(yaw_rate_cmd) ? 0 : 1);
    
    // Use NaN-safe values
    double yaw_val = std::isnan(yaw_cmd) ? 0.0 : yaw_cmd;
    double yaw_rate_val = std::isnan(yaw_rate_cmd) ? 0.0 : yaw_rate_cmd;
    
    std::vector<uint8_t> result;
    result.reserve(52);  // 4 + 4 + 24 + 8 + 8 + 1 + 1 + 2 = 52 bytes
    
    // Pack as little-endian binary: '<II' (magic, version), '<3d' (vel), '<2d' (yaw, yaw_rate), '<2B' (flags), padding
    // Magic and version (8 bytes)
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&magic), 
                  reinterpret_cast<const uint8_t*>(&magic) + sizeof(uint32_t));
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&version), 
                  reinterpret_cast<const uint8_t*>(&version) + sizeof(uint32_t));
    
    // Velocity (24 bytes - 3 doubles)
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&vel_cmd[0]), 
                  reinterpret_cast<const uint8_t*>(&vel_cmd[0]) + sizeof(double));
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&vel_cmd[1]), 
                  reinterpret_cast<const uint8_t*>(&vel_cmd[1]) + sizeof(double));
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&vel_cmd[2]), 
                  reinterpret_cast<const uint8_t*>(&vel_cmd[2]) + sizeof(double));
    
    // Yaw and yaw_rate (16 bytes - 2 doubles)
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&yaw_val), 
                  reinterpret_cast<const uint8_t*>(&yaw_val) + sizeof(double));
    result.insert(result.end(), reinterpret_cast<const uint8_t*>(&yaw_rate_val), 
                  reinterpret_cast<const uint8_t*>(&yaw_rate_val) + sizeof(double));
    
    // Flags (2 bytes)
    result.push_back(yaw_valid);
    result.push_back(yaw_rate_valid);
    
    // Padding (2 bytes)
    result.push_back(0x00);
    result.push_back(0x00);
    
    return result;
}

Flight_Data SystemManager::_deserialize_flight_data(const std::vector<uint8_t>& data_bytes) {
    Flight_Data data;
    
    // Check minimum size
    if (data_bytes.size() < 8) {
        throw std::runtime_error("Data too short for deserialization");
    }
    
    size_t offset = 0;
    
    // Read magic and version (8 bytes)
    uint32_t magic, version;
    std::memcpy(&magic, data_bytes.data() + offset, 4);
    offset += 4;
    std::memcpy(&version, data_bytes.data() + offset, 4);
    offset += 4;
    
    // Validate magic number (0x464C4947 = "FLIG")
    if (magic != 0x464C4947) {
        throw std::runtime_error("Invalid magic number: " + std::to_string(magic));
    }
    if (version != 1) {
        throw std::runtime_error("Unsupported version: " + std::to_string(version));
    }
    
    // Read message_count (4 bytes, uint32_t)
    uint32_t message_count_u32;
    std::memcpy(&message_count_u32, data_bytes.data() + offset, 4);
    data.message_count = static_cast<int>(message_count_u32);
    offset += 4;
    
    // Read 63 doubles (504 bytes)
    if (data_bytes.size() < offset + 504) {
        throw std::runtime_error("Data too short for doubles section");
    }
    
    double doubles[63];
    std::memcpy(doubles, data_bytes.data() + offset, 504);
    offset += 504;
    
    int idx = 0;
    
    // Scalar doubles (19 total)
    data.quat_ts = static_cast<int64_t>(doubles[idx++]);
    data.imu_ts = static_cast<int64_t>(doubles[idx++]);
    data.timestamp = static_cast<int64_t>(doubles[idx++]);
    data.local_ts = static_cast<int64_t>(doubles[idx++]);
    data.temperature = doubles[idx++];
    data.amsl_m = doubles[idx++];
    data.local_m = doubles[idx++];
    data.monotonic_m = doubles[idx++];
    data.relative_m = doubles[idx++];
    data.terrain_m = doubles[idx++];
    data.bottom_clearance_m = doubles[idx++];
    data.pressure = doubles[idx++];
    data.absolute_press_hpa = doubles[idx++];
    data.differential_press_hpa = doubles[idx++];
    data.signal_strength_percent = doubles[idx++];
    data.throttle = doubles[idx++];
    data.heading = doubles[idx++];
    data.groundspeed = doubles[idx++];
    data.current_thrust = doubles[idx++];
    
    // Quaternion (5 doubles: x, y, z, w, timestamp)
    data.quat_ned_bodyfrd.x = doubles[idx++];
    data.quat_ned_bodyfrd.y = doubles[idx++];
    data.quat_ned_bodyfrd.z = doubles[idx++];
    data.quat_ned_bodyfrd.w = doubles[idx++];
    data.quat_ned_bodyfrd.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // Altitude (4 doubles: amsl, relative, vertical_speed_estimate, timestamp)
    data.altitude_m.amsl = doubles[idx++];
    data.altitude_m.relative = doubles[idx++];
    data.altitude_m.vertical_speed_estimate = doubles[idx++];
    data.altitude_m.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // IMU raw FRD (7 doubles: accel[3], gyro[3], timestamp)
    data.imu_raw_frd.accel[0] = doubles[idx++];
    data.imu_raw_frd.accel[1] = doubles[idx++];
    data.imu_raw_frd.accel[2] = doubles[idx++];
    data.imu_raw_frd.gyro[0] = doubles[idx++];
    data.imu_raw_frd.gyro[1] = doubles[idx++];
    data.imu_raw_frd.gyro[2] = doubles[idx++];
    data.imu_raw_frd.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // IMU NED (7 doubles: accel[3], gyro[3], timestamp)
    data.imu_ned.accel[0] = doubles[idx++];
    data.imu_ned.accel[1] = doubles[idx++];
    data.imu_ned.accel[2] = doubles[idx++];
    data.imu_ned.gyro[0] = doubles[idx++];
    data.imu_ned.gyro[1] = doubles[idx++];
    data.imu_ned.gyro[2] = doubles[idx++];
    data.imu_ned.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // Position NED (7 doubles: ned[3], vel_ned[3], timestamp)
    data.pos_ned_m.ned[0] = doubles[idx++];
    data.pos_ned_m.ned[1] = doubles[idx++];
    data.pos_ned_m.ned[2] = doubles[idx++];
    data.pos_ned_m.vel_ned[0] = doubles[idx++];
    data.pos_ned_m.vel_ned[1] = doubles[idx++];
    data.pos_ned_m.vel_ned[2] = doubles[idx++];
    data.pos_ned_m.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // Raw position LLA (4 doubles: lla[3], timestamp)
    data.raw_pos_lla_deg.lla[0] = doubles[idx++];
    data.raw_pos_lla_deg.lla[1] = doubles[idx++];
    data.raw_pos_lla_deg.lla[2] = doubles[idx++];
    data.raw_pos_lla_deg.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // Filtered position LLA (4 doubles: lla[3], timestamp)
    data.filt_pos_lla_deg.lla[0] = doubles[idx++];
    data.filt_pos_lla_deg.lla[1] = doubles[idx++];
    data.filt_pos_lla_deg.lla[2] = doubles[idx++];
    data.filt_pos_lla_deg.timestamp = static_cast<int64_t>(doubles[idx++]);
    
    // RPY rates and RPY (6 doubles: rpy_rates[3], rpy[3])
    data.rpy_rates[0] = doubles[idx++];
    data.rpy_rates[1] = doubles[idx++];
    data.rpy_rates[2] = doubles[idx++];
    data.rpy[0] = doubles[idx++];
    data.rpy[1] = doubles[idx++];
    data.rpy[2] = doubles[idx++];
    
    // Read uint32_t fields
    if (data_bytes.size() < offset + 4) {
        throw std::runtime_error("Data too short for custom_mode_id");
    }
    uint32_t custom_mode_id_u32;
    std::memcpy(&custom_mode_id_u32, data_bytes.data() + offset, 4);
    data.custom_mode_id = static_cast<int>(custom_mode_id_u32);
    offset += 4;
    
    if (data_bytes.size() < offset + 4) {
        throw std::runtime_error("Data too short for mode");
    }
    uint32_t mode_val;
    std::memcpy(&mode_val, data_bytes.data() + offset, 4);
    data.mode = static_cast<FLIGHT_MODE>(mode_val);
    offset += 4;
    
    // Read bools (packed as uint8_t)
    if (data_bytes.size() < offset + 1) {
        throw std::runtime_error("Data too short for bools");
    }
    uint8_t bools = data_bytes[offset++];
    data.is_armed = (bools & 1) != 0;
    data.offboardMode = (bools & 2) != 0;
    data.is_available = (bools & 4) != 0;
    data.was_available_once = (bools & 8) != 0;
    data.is_gyrometer_calibration_ok = (bools & 16) != 0;
    data.is_accelerometer_calibration_ok = (bools & 32) != 0;
    data.is_magnetometer_calibration_ok = (bools & 64) != 0;
    data.in_air = (bools & 128) != 0;
    
    // Read gathered flags (uint32_t bitfield)
    if (data_bytes.size() < offset + 4) {
        throw std::runtime_error("Data too short for gathered flags");
    }
    uint32_t gathered_flags;
    std::memcpy(&gathered_flags, data_bytes.data() + offset, 4);
    offset += 4;
    
    // Debug: Log gathered flags (first few times)
    static int gather_debug_count = 0;
    if (gather_debug_count < 3) {
        std::cout << "System_Manager: Raw gathered_flags value: 0x" << std::hex << gathered_flags 
                  << std::dec << " (" << gathered_flags << ")" << std::endl;
        std::cout << "System_Manager: Offset when reading gathered_flags: " << (offset - 4) 
                  << ", total data size: " << data_bytes.size() << std::endl;
        gather_debug_count++;
    }
    
    // Set gathered flags (only those that exist in C++ struct)
    // First, use the flags from the binary data
    data.gathered.euler_ned_bodyfrd = (gathered_flags & 1) != 0;
    data.gathered.quat_ned_bodyfrd = (gathered_flags & 2) != 0;
    data.gathered.pos_ned_m = (gathered_flags & 4) != 0;
    data.gathered.imu_ned = (gathered_flags & 16) != 0;
    data.gathered.tracker_px = (gathered_flags & 32) != 0;
    
    // If flags are 0 but we have valid-looking data, set flags based on data validity
    // This handles the case where hardware_adapter hasn't set flags yet but has data
    if (gathered_flags == 0) {
        // Check if quaternion is valid (not all zeros, w should be close to 1 for identity)
        if (data.quat_ned_bodyfrd.w != 0.0 || data.quat_ned_bodyfrd.x != 0.0 || 
            data.quat_ned_bodyfrd.y != 0.0 || data.quat_ned_bodyfrd.z != 0.0) {
            data.gathered.quat_ned_bodyfrd = true;
        }
        
        // For position and IMU, be more lenient - if we got a message, assume data is valid
        // Even if values are zero (e.g., at origin or stationary), the data structure is valid
        // Check if timestamp is non-zero as a sign that data was actually received
        if (data.pos_ned_m.timestamp != 0 || data.timestamp != 0) {
            data.gathered.pos_ned_m = true;
        }
        
        // Check if IMU timestamp is non-zero
        if (data.imu_ned.timestamp != 0 || data.imu_ts != 0) {
            data.gathered.imu_ned = true;
        }
        
        // Debug: Log when we're inferring flags from data
        static int inferred_flags_count = 0;
        if (inferred_flags_count < 3) {
            std::cout << "System_Manager: Inferred gathered flags from data validity. "
                      << "quat=" << data.gathered.quat_ned_bodyfrd
                      << ", pos=" << data.gathered.pos_ned_m
                      << " (pos_ts=" << data.pos_ned_m.timestamp << ", ts=" << data.timestamp << ")"
                      << ", imu=" << data.gathered.imu_ned
                      << " (imu_ts=" << data.imu_ned.timestamp << ", imu_ts=" << data.imu_ts << ")"
                      << std::endl;
            inferred_flags_count++;
        }
    }
    
    return data;
}

void SystemManager::gatherData() {
    // Use the persistent socket created in __init__
    // With CONFLATE enabled, we only get the latest message
    // Receive single-part message with topic prefix
    zmq::message_t message;
    bool received = false;
    
    // Try multiple times to catch messages (with CONFLATE, we only get the latest anyway)
    for (int attempt = 0; attempt < 3; attempt++) {
        try {
            zmq::recv_result_t result = subsSock.recv(message, zmq::recv_flags::dontwait);
            if (result.has_value() && result.value() > 0) {
                received = true;
                break;  // Got a message, exit loop
            }
        } catch (const zmq::error_t& e) {
            if (e.num() == EAGAIN) {
                // No message available - try again with small delay
                if (attempt < 2) {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));  // 0.0001 seconds
                    continue;
                }
            } else {
                // Other error
                double now = TimeUtils::now();
                if (now - _last_gather_error_time > 5.0) {
                    std::cerr << "Error in gatherData: " << e.what() << std::endl;
                    _last_gather_error_time = now;
                }
                break;
            }
        } catch (const std::exception& e) {
            double now = TimeUtils::now();
            if (now - _last_gather_error_time > 5.0) {
                std::cerr << "Error in gatherData: " << e.what() << std::endl;
                _last_gather_error_time = now;
            }
            break;
        }
    }
    
    // Process the received message if we got one
    if (received) {
        try {
            // Check if message starts with topic prefix (matches Python: ret.startswith(zmqTopics.topicMavlinkFlightData))
            std::string topic_prefix(TOPIC_MAVLINK_FLIGHT_DATA);
            std::string msg_str(static_cast<const char*>(message.data()), message.size());
            
            // Debug: Log message info occasionally
            static int msg_count = 0;
            static double last_debug_time = 0;
            double now = TimeUtils::now();
            if (now - last_debug_time > 5.0) {
                std::cout << "System_Manager: Received message, size: " << message.size() 
                          << " bytes, starts with: " << (msg_str.size() > 0 ? std::string(1, msg_str[0]) : "empty")
                          << ", topic prefix: " << topic_prefix << std::endl;
                last_debug_time = now;
            }
            
            if (msg_str.find(topic_prefix) == 0) {
                // Strip topic prefix before deserializing (matches Python: data_bytes = ret[len(zmqTopics.topicMavlinkFlightData):])
                std::vector<uint8_t> data_bytes(
                    static_cast<const uint8_t*>(message.data()) + topic_prefix.size(),
                    static_cast<const uint8_t*>(message.data()) + message.size()
                );
                
                // Debug: Log deserialization attempt
                if (msg_count < 3) {
                    std::cout << "System_Manager: Attempting to deserialize " << data_bytes.size() 
                              << " bytes of flight data" << std::endl;
                    // Check magic number
                    if (data_bytes.size() >= 4) {
                        uint32_t magic_check = 0;
                        std::memcpy(&magic_check, data_bytes.data(), 4);
                        std::cout << "System_Manager: Magic number: 0x" << std::hex << magic_check 
                                  << std::dec << " (expected: 0x464C4947)" << std::endl;
                    }
                    msg_count++;
                }
                
                // Deserialize flight data
                Flight_Data data = _deserialize_flight_data(data_bytes);
                
                double curTime = TimeUtils::now();
                _currentData = data;
                               
                // Handle flight mode logic
                if (_currentData.custom_mode_id != static_cast<int>(PX4_FLIGHT_STATE::OFFBOARD)) {
                    // HOLD ON state or POSITION state - close loggers when leaving OFFBOARD mode
                    if (_input_logger) {
                        _input_logger->close();
                        _input_logger.reset();
                    }
                    // Control loggers will be closed automatically when log_data=false is passed to get_cmd
                    double heading_rad = _currentData.heading;
                    yawDefinedDir_ned = Vector3d(std::cos(heading_rad), std::sin(heading_rad), 0);
                    holdonHeading = yawDefinedDir_ned;
                    holdonPos_ned = _currentData.pos_ned_m.ned;
                    holdonTime = TimeUtils::now();
                    
                    if (_currentData.throttle > 5) {
                        // TODO: Implement throttle-based logic when Control methods are available
                        // if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
                        //     // Handle VELOCITYRL case
                        // } else {
                        //     // Handle other controller types
                        // }
                    }
                    
                    homingStage = (missionType == MISSION_TYPE::TRACKER) ? HOMING_STAGE::SECTION : HOMING_STAGE::NONE;
                    _currentData.offboardMode = false;
                    offboardEntry = true;
                } else if (_currentData.custom_mode_id == static_cast<int>(PX4_FLIGHT_STATE::OFFBOARD)) {
                    // OFFBOARD mode
                    if (!_input_logger) {
                        std::string log_name = TimeUtils::get_unique_datetime_str() + "_system_manager";
                        _input_logger = std::make_unique<Logger>(log_name, _log_dir, true, false, "CSV");
                    }
                    
                    _currentData.offboardMode = true;
                    // TODO: Handle integral term logic when controller methods are available
                    
                    if (offboardEntry && missionType == MISSION_TYPE::WAYPOINT) {
                        // When entering offboard mode, set initial waypoint if waypoint list is available
                        if (!waypointList.empty()) {
                            pointIndex = 0;  // Start from first waypoint
                            referencePoint = waypointList[pointIndex];
                            std::cout << "System_Manager: Offboard entry - Setting initial waypoint " << pointIndex 
                                      << " [" << referencePoint.transpose() << "]" << std::endl;
                        }
                        offboardEntry = false;
                    }
                }
            } else {
                // Unexpected message format
                double now = TimeUtils::now();
                static double last_warn_time = 0;
                if (now - last_warn_time > 5.0) {
                    std::cout << "System_Manager: Warning - Received message without expected topic prefix. "
                              << "Message size: " << message.size() << " bytes, "
                              << "First 20 chars: ";
                    size_t preview_len = std::min(message.size(), size_t(20));
                    for (size_t i = 0; i < preview_len; i++) {
                        char c = msg_str[i];
                        if (c >= 32 && c < 127) {
                            std::cout << c;
                        } else {
                            std::cout << "\\x" << std::hex << (unsigned char)c << std::dec;
                        }
                    }
                    std::cout << std::endl;
                    last_warn_time = now;
                }
                return;
            }
        } catch (const std::exception& e) {
            double now = TimeUtils::now();
            if (now - _last_gather_error_time > 5.0) {
                std::cerr << "System_Manager: Error processing flight data in gatherData: " << e.what() << std::endl;
                _last_gather_error_time = now;
            }
        }
    } else {
        // No message received - warn occasionally
        double now = TimeUtils::now();
        if (now - _last_no_data_warning_time > 5.0) {
            std::cout << "System_Manager: WARNING - No flight data received for 5s. "
                      << "Is hardware_adapter running and publishing on port " 
                      << TOPIC_MAVLINK_PORT << "? "
                      << "Subscribed to topic: " << TOPIC_MAVLINK_FLIGHT_DATA << std::endl;
            _last_no_data_warning_time = now;
        }
    }
}

void SystemManager::_log_input_data(double current_ts, int64_t imu_ts,
                                    const Vector3d& command, const Vector3d& rpy_rate_cmd,
                                    const Quaternion& quat_ned_desbodyfrd_cmd, double step_dt, int counter,
                                    const Vector3d& destination_ned, int current_mode) {
    if (!_input_logger) {
        return;  // Logger not initialized
    }
    
    std::map<std::string, std::string> log_data;
    
    // Add all the fields from Python version
    log_data["comp_time"] = std::to_string(TimeUtils::now());
    log_data["imu_ts"] = std::to_string(_currentData.imu_ned.timestamp);
    log_data["accl_ned/x"] = std::to_string(_currentData.imu_ned.accel[0]);
    log_data["accl_ned/y"] = std::to_string(_currentData.imu_ned.accel[1]);
    log_data["accl_ned/z"] = std::to_string(_currentData.imu_ned.accel[2]);
    log_data["gyro_ned/x"] = std::to_string(_currentData.imu_ned.gyro[0]);
    log_data["gyro_ned/y"] = std::to_string(_currentData.imu_ned.gyro[1]);
    log_data["gyro_ned/z"] = std::to_string(_currentData.imu_ned.gyro[2]);
    log_data["pos_ned_ts"] = std::to_string(_currentData.pos_ned_m.timestamp);
    log_data["pos_ned/x"] = std::to_string(_currentData.pos_ned_m.ned[0]);
    log_data["pos_ned/y"] = std::to_string(_currentData.pos_ned_m.ned[1]);
    log_data["pos_ned/z"] = std::to_string(_currentData.pos_ned_m.ned[2]);
    log_data["vel_ned/x"] = std::to_string(_currentData.pos_ned_m.vel_ned[0]);
    log_data["vel_ned/y"] = std::to_string(_currentData.pos_ned_m.vel_ned[1]);
    log_data["vel_ned/z"] = std::to_string(_currentData.pos_ned_m.vel_ned[2]);
    log_data["quat_ned_bodyfrd_ts"] = std::to_string(_currentData.quat_ned_bodyfrd.timestamp);
    log_data["quat_ned_bodyfrd/x"] = std::to_string(_currentData.quat_ned_bodyfrd.x);
    log_data["quat_ned_bodyfrd/y"] = std::to_string(_currentData.quat_ned_bodyfrd.y);
    log_data["quat_ned_bodyfrd/z"] = std::to_string(_currentData.quat_ned_bodyfrd.z);
    log_data["quat_ned_bodyfrd/w"] = std::to_string(_currentData.quat_ned_bodyfrd.w);
    log_data["lla_ts"] = std::to_string(_currentData.raw_pos_lla_deg.timestamp);
    log_data["lla/lat_deg"] = std::to_string(_currentData.raw_pos_lla_deg.lla[0]);
    log_data["lla/lon_deg"] = std::to_string(_currentData.raw_pos_lla_deg.lla[1]);
    log_data["lla/alt"] = std::to_string(_currentData.raw_pos_lla_deg.lla[2]);
    log_data["current_ts"] = std::to_string(current_ts);
    log_data["step_dt"] = std::to_string(step_dt);
    log_data["imu_ts"] = std::to_string(imu_ts);
    log_data["current_alt"] = std::to_string(_currentData.relative_m);
    log_data["command"] = std::to_string(command[0]) + "," + std::to_string(command[1]) + "," + std::to_string(command[2]);
    log_data["rpy_rate_cmd/x"] = std::to_string(rpy_rate_cmd[0]);
    log_data["rpy_rate_cmd/y"] = std::to_string(rpy_rate_cmd[1]);
    log_data["rpy_rate_cmd/z"] = std::to_string(rpy_rate_cmd[2]);
    log_data["quat_ned_desbodyfrd_cmd/x"] = std::to_string(quat_ned_desbodyfrd_cmd.x);
    log_data["quat_ned_desbodyfrd_cmd/y"] = std::to_string(quat_ned_desbodyfrd_cmd.y);
    log_data["quat_ned_desbodyfrd_cmd/z"] = std::to_string(quat_ned_desbodyfrd_cmd.z);
    log_data["quat_ned_desbodyfrd_cmd/w"] = std::to_string(quat_ned_desbodyfrd_cmd.w);
    log_data["destination_ned/x"] = std::to_string(destination_ned[0]);
    log_data["destination_ned/y"] = std::to_string(destination_ned[1]);
    log_data["destination_ned/z"] = std::to_string(destination_ned[2]);
    log_data["counter"] = std::to_string(counter);
    log_data["current_mode"] = std::to_string(current_mode);
    log_data["timestamp"] = std::to_string(_currentData.timestamp);
    log_data["local_ts"] = std::to_string(_currentData.local_ts);
    log_data["modeID"] = std::to_string(_currentData.custom_mode_id);
    
    _input_logger->log(log_data);
}

