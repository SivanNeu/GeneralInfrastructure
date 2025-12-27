#include "SystemManager.h"
#include "utils/Euler.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <cmath>
#include <cstring>
#include <algorithm>

SystemManager::SystemManager(const std::string& config_dir, const std::string& log_dir, double currentTime)
    : _config_dir(config_dir), _log_dir(log_dir),
      _overall_start(currentTime < 0 ? TimeUtils::now() : currentTime),
      _prev_los_ned_dir(Eigen::Vector3d::Zero()),
      _prev_pos_ned(Eigen::Vector3d::Zero()),
      _prev_imu_ts(0), _prev_ts(_overall_start),
      message_count(0), dronemass(0.55),
      destHeight(std::nullopt), tar_measurement_ned(Eigen::Vector3d::Zero()),
      heading_dir_ned(Eigen::Vector3d(-1, 1, 0).normalized()),
      homingStage(HOMING_STAGE::NONE), pointIndex(0), offboardEntry(false),
      referencePoint(Eigen::Vector3d::Zero()),
      desiredHeadingDir_ned(Eigen::Vector3d(-1, 1, 0)),
      missionType(MISSION_TYPE::WAYPOINT),
      yawControlType(YAW_COMMAND::DEFINED_DIR),
      yawCommandFactor(1.0), maximalVelocity(0.75), descentVelocity(10.0),
      targetVelocity(7.5), originOffset_frd(Eigen::Vector3d::Zero()),
      terminalHomingAlowed(true), circleRadius(5.0),
      controllerType(CONTROLLER_TYPE::VELOCITYRL),
      yawCommandType(YAW_COMMAND_TYPE::RATE),
      rateControlEnabled(false),
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
    
    _current_pos_lla = LLA(0, Eigen::Vector3d::Zero());
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
        Eigen::Vector3d missionPoint = (holdonPos_ned.value_or(Eigen::Vector3d::Zero()) + referencePoint);
        Eigen::Vector3d deltaPos_ned = missionPoint;
        Eigen::Vector3d deltaPos_frd = Eigen::Vector3d::Zero();
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
        Eigen::Vector3d trajDest_pos_ned = std::get<0>(trajDest);
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

std::tuple<std::vector<bool>, std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>,
          Eigen::Vector3d, Quaternion, double> SystemManager::setDesiredState(double curTime) {
    
    if (!holdonHeading.has_value()) {
        Eigen::Vector3d forward(1, 0, 0);
        holdonHeading = _currentData.quat_ned_bodyfrd.rotate_vec(forward);
    }
    if (!holdonPos_ned.has_value()) {
        holdonPos_ned = _currentData.pos_ned_m.ned;
    }
    if (!holdonTime.has_value()) {
        holdonTime = curTime;
    }
    
    Eigen::Vector3d pos_ned = _currentData.pos_ned_m.ned;
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
    
    Eigen::Vector3d missionPoint = holdonPos_ned.value() + referencePoint;
    
    if (missionType == MISSION_TYPE::WAYPOINT) {
        desired_trajectory = ::pos_point({missionPoint}, heading_dir_ned, _overall_start);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
        heading_dir_ned = desired_trajectory.b1[0];
    } else if (missionType == MISSION_TYPE::VELOCITY) {
        Eigen::Vector3d missionVelocity = (missionPoint - holdonPos_ned.value()).normalized() * maximalVelocity;
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
        Eigen::Vector3d startPoint = holdonPos_ned.value();
        Eigen::Vector3d endPoint = missionPoint;
        desired_trajectory = ::lineConstVel(startPoint, endPoint, maximalVelocity, holdonTime.value(), heading_dir_ned);
        controlType = desired_trajectory.pos_control;
        controlType.insert(controlType.end(), desired_trajectory.vel_control.begin(), desired_trajectory.vel_control.end());
        dest_pos_ned = desired_trajectory.x[0];
        destHeight = desired_trajectory.x[0][2];
    }
    
    Eigen::Vector3d trajDest_pos_ned = dest_pos_ned.value_or(Eigen::Vector3d::Zero());
    trajDest_pos_ned[2] = destHeight.has_value() ? destHeight.value() : _currentData.pos_ned_m.ned[2];
    
    Eigen::Vector3d trajDest_vel_ned = desired_trajectory.x.size() > 1 ? desired_trajectory.x[1] : Eigen::Vector3d::Zero();
    Eigen::Vector3d trajDest_acc_ned = desired_trajectory.x.size() > 2 ? desired_trajectory.x[2] : Eigen::Vector3d::Zero();
    
    auto trajDest = std::make_tuple(trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned);
    
    return std::make_tuple(controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts);
}

std::optional<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Quaternion, double, double>>
SystemManager::generateCommand(const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& trajDest,
                              const std::vector<bool>& controlType, double current_ts, int counter,
                              const Quaternion& quat_ned_bodyfrd) {
    
    auto [command, rpyRate_cmd, quat_ned_desbodyfrd_cmd] = _controlMain->get_cmd(
        _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
        _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
        quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
        trajDest, _currentData, std::make_tuple(heading_dir_ned, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
        controlType, true, homingStage);
    
    if (_controlAux) {
        auto [commandAux, rpyRate_cmdAux, quat_ned_desbodyfrd_cmdAux] = _controlAux->get_cmd(
            _currentData.pos_ned_m.ned, _currentData.pos_ned_m.vel_ned,
            _currentData.imu_ned.accel, _currentData.imu_ned.gyro,
            quat_ned_bodyfrd, _currentData.imu_ts, current_ts - _prev_ts, current_ts, counter,
            trajDest, _currentData, std::make_tuple(heading_dir_ned, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
            controlType, true, homingStage);
        
        Eigen::Vector3d commandBody = commandAux;
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
    
    Eigen::Vector3d forward_dir_frd = quat_ned_bodyfrd.rotate_vec(Eigen::Vector3d(1, 0, 0));
    
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
    if (command.norm() == 0 && command != Eigen::Vector3d::Zero()) {
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

CommandMessage SystemManager::publishCommand(const Eigen::Vector3d& command, double yawCmd, double yawCmdRate,
                                            const Eigen::Vector3d& rpyRate_cmd, const Quaternion& quat_ned_desbodyfrd_cmd) {
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

// Continue with trajectory functions and other methods in next part...

