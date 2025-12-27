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

SystemManager::SystemManager(const std::string& config_dir, const std::string& log_dir, double currentTime,
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
                             CONTROLLER_TYPE controllerType,
                             YAW_COMMAND_TYPE yawCommandType,
                             bool rateControlEnabled)
    : _config_dir(config_dir), _log_dir(log_dir),
      _overall_start(currentTime < 0 ? TimeUtils::now() : currentTime),
      _prev_los_ned_dir(Vector3d::Zero()),
      _prev_pos_ned(Vector3d::Zero()),
      _prev_imu_ts(0), _prev_ts(_overall_start),
      message_count(0), dronemass(dronemass),
      destHeight(std::nullopt), tar_measurement_ned(Vector3d::Zero()),
      heading_dir_ned(heading_dir_ned_init.normalized()),
      homingStage(HOMING_STAGE::NONE), pointIndex(0), offboardEntry(false),
      referencePoint(Vector3d::Zero()),
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
    
    // Initialize controllers
    if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
        auto auxController = std::make_unique<VelocityPIDController>(dronemass, _overall_start);
        _controlAux = std::make_unique<Control>(_config_dir, _log_dir, auxController.release(), maximalVelocity);
        
        auto mainController = std::make_unique<VelocityRLController>(dronemass, maximalVelocity, _overall_start);
        _controlMain = std::make_unique<Control>(_config_dir, _log_dir, mainController.release(), maximalVelocity);
    } else if (controllerType == CONTROLLER_TYPE::VELOCITYPID) {
        auto mainController = std::make_unique<VelocityPIDController>(dronemass, _overall_start, yawCommandType);
        _controlMain = std::make_unique<Control>(_config_dir, _log_dir, mainController.release(), maximalVelocity);
    }
    
    // Setup ZMQ subscriber
    subsSock.set(zmq::sockopt::conflate, 1);
    subsSock.set(zmq::sockopt::rcvtimeo, 100);
    std::string sub_addr = "tcp://127.0.0.1:" + std::to_string(TOPIC_MAVLINK_PORT);
    subsSock.connect(sub_addr);
    subsSock.set(zmq::sockopt::subscribe, TOPIC_MAVLINK_FLIGHT_DATA);
    std::cout << "System_Manager: Subscribed to topic: " << TOPIC_MAVLINK_FLIGHT_DATA 
              << " on port: " << TOPIC_MAVLINK_PORT << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Setup ZMQ publisher
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
    
    // Generate command from controller
    auto cmd_result = generateCommand(trajDest, controlType, current_ts, counter, quat_ned_bodyfrd);
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
        if (vel_2d.norm() > 1.0) {
            heading_dir_ned = _currentData.pos_ned_m.vel_ned;
            heading_dir_ned[2] = 0;
            heading_dir_ned.normalize();
        }
    } else if (yawControlType == YAW_COMMAND::DEFINED_DIR) {
        heading_dir_ned = desiredHeadingDir_ned;
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
                              const Quaternion& quat_ned_bodyfrd) {
    
    auto [command, rpyRate_cmd, quat_ned_desbodyfrd_cmd] = _controlMain->get_cmd(
        _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
        _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
        quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
        trajDest, _currentData, std::make_tuple(heading_dir_ned, Vector3d::Zero(), Vector3d::Zero()),
        controlType, true, homingStage);
    
    if (_controlAux) {
        auto [commandAux, rpyRate_cmdAux, quat_ned_desbodyfrd_cmdAux] = _controlAux->get_cmd(
            _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
            _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
            quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
            trajDest, _currentData, std::make_tuple(heading_dir_ned, Vector3d::Zero(), Vector3d::Zero()),
            controlType, true, homingStage);
        
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
            
            pubSock.send(topic_msg, zmq::send_flags::sndmore);
            pubSock.send(data_msg, zmq::send_flags::dontwait);
            
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
            // Check if message starts with topic prefix
            std::string topic_prefix(TOPIC_MAVLINK_FLIGHT_DATA);
            std::string msg_str(static_cast<const char*>(message.data()), message.size());
            
            if (msg_str.find(topic_prefix) == 0) {
                // Strip topic prefix before deserializing
                std::vector<uint8_t> data_bytes(
                    static_cast<const uint8_t*>(message.data()) + topic_prefix.size(),
                    static_cast<const uint8_t*>(message.data()) + message.size()
                );
                
                // Note: _deserialize_flight_data needs to be implemented separately
                // For now, create a placeholder Flight_Data
                // TODO: Implement _deserialize_flight_data based on Python pickle format
                Flight_Data data;
                // data = _deserialize_flight_data(data_bytes);  // Uncomment when implemented
                
                double curTime = TimeUtils::now();
                _currentData = data;
                
                // Set gathered flags to indicate we have received data
                _currentData.gathered.quat_ned_bodyfrd = true;
                _currentData.gathered.pos_ned_m = true;
                _currentData.gathered.imu_ned = true;
                
                // Handle flight mode logic
                if (_currentData.custom_mode_id != static_cast<int>(PX4_FLIGHT_STATE::OFFBOARD)) {
                    // HOLD ON state or POSITION state
                    _input_logger.reset();  // Set to nullptr
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
                        // TODO: Handle pointList when available
                        // pointIndex = (pointIndex + 1) % pointList.size();
                        // referencePoint = pointList[pointIndex];
                        offboardEntry = false;
                    }
                }
            } else {
                // Unexpected message format
                std::cout << "Warning: Received message without expected topic prefix" << std::endl;
                return;
            }
        } catch (const std::exception& e) {
            double now = TimeUtils::now();
            if (now - _last_gather_error_time > 5.0) {
                std::cerr << "Error processing flight data in gatherData: " << e.what() << std::endl;
                _last_gather_error_time = now;
            }
        }
    } else {
        // No message received - warn occasionally
        double now = TimeUtils::now();
        if (now - _last_no_data_warning_time > 5.0) {
            std::cout << "System_manager: WARNING - No flight data received for 5s. Is hardware_adapter running and publishing on port " 
                      << TOPIC_MAVLINK_PORT << "?" << std::endl;
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

