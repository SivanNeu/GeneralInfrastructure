#include "Control.h"
#include "general.h"
#include "VelocityPIDController.h"
#include "VelocityRLController.h"
#include "utils/GeneralFuncs.h"
#include "utils/TimeUtils.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <map>
#include <fstream>
#ifdef __has_include
    #if __has_include(<filesystem>)
        #include <filesystem>
        namespace fs = std::filesystem;
    #else
        #include <experimental/filesystem>
        namespace fs = std::experimental::filesystem;
    #endif
#else
    #include <filesystem>
    namespace fs = std::filesystem;
#endif

// Guidance is now defined in Control.h

Control::Control(const std::string& config_dir, const std::string& log_directory,
                 void* controller, double maximalVelocity)
    : log_directory(log_directory), maximalVelocity(maximalVelocity),
      _min_thrust(0.01), _max_thrust(1.0), MaximalThrust(1.0),
      enableIntegrator(false), controlnode(controller),
      _last_pitch_update(TimeUtils::now()), _finished_stationary_tracking(false),
      _current_pos_ned(Vector3d::Zero()), _current_vel_ned(Vector3d::Zero()) {
    
    // Try to get MaximalThrust from controller
    // This is a simplified version - in practice you'd use a base class interface
    try {
        // Would need proper interface to get mass from controller
        // For now, use default
        MaximalThrust = 1.0;
    } catch (...) {
        MaximalThrust = 1.0;
    }
    
    // Initialize guidance
    double guidance_constant_N = 8.0;
    int guidance_type = 0;  // PURE_PURSUIT - would need enum
    _guidance = std::make_unique<Guidance>(guidance_constant_N, guidance_type);
}

std::tuple<Vector3d, Vector3d, Quaternion> Control::get_cmd(
    const Vector3d& pos_ned, const Vector3d& vel_ned,
    const Vector3d& accel_ned, const Vector3d& gyro_ned,
    const Quaternion& quat_ned_bodyfrd, int64_t imu_ts, double step_dt,
    int64_t current_ts, int counter,
    const std::tuple<Vector3d, Vector3d, Vector3d>& trajDest_ned,
    Flight_Data& currentData,
    const std::optional<std::tuple<Vector3d, Vector3d, Vector3d>>& headingDest,
    const std::optional<std::vector<bool>>& controlType,
    bool log_data,
    const std::optional<HOMING_STAGE>& homingStage) {
    
    Vector3d b1d_dot = Vector3d::Zero();
    Vector3d b1d_ddot = Vector3d::Zero();
    
    std::vector<bool> controlType_use;
    if (controlType.has_value()) {
        controlType_use = controlType.value();
    } else {
        controlType_use = {true, true, true, false, false, false};  // pos_control, vel_control, yaw_control
    }
    
    Vector3d estimated_tar_pos_ned = std::get<0>(trajDest_ned);
    Vector3d pos_des_ned = std::get<0>(trajDest_ned);
    Vector3d vel_des_ned = std::get<1>(trajDest_ned);
    Vector3d accel_des_ned = std::get<2>(trajDest_ned);
    
    Vector3d b1d_ned(1, 0, 0);
    if (headingDest.has_value()) {
        b1d_ned = std::get<0>(headingDest.value());
        b1d_dot = std::get<1>(headingDest.value());
        b1d_ddot = std::get<2>(headingDest.value());
    }
    
    _current_pos_ned = pos_ned;
    _current_vel_ned = vel_ned;
    
    // Prepare body state tuples
    auto currentBodyState = std::make_tuple(pos_ned, vel_ned, accel_ned, gyro_ned, quat_ned_bodyfrd);
    auto desiredBodyState = std::make_tuple(
        std::make_tuple(pos_des_ned, vel_des_ned, accel_des_ned, Vector3d::Zero(), Vector3d::Zero()),
        std::make_tuple(b1d_ned, b1d_dot, b1d_ddot)
    );
    
    // Check if we should start logging (before processing observation to save correct hxs state)
    bool should_start_logging = log_data && !_control_logger;
    std::string log_name;  // Declare outside if block so it's accessible later
    
    // Determine controller type to get name (needed for both hxs saving and logger creation)
    // We'll determine it properly when we call getCommand below, but for now try to get the name
    CONTROLLER_TYPE temp_controllerType = CONTROLLER_TYPE::VELOCITYPID;
    std::string temp_controller_name = "unknown";
    
    // Try to determine controller type without processing observation
    // Note: static_cast doesn't throw, so we check the actual type by calling getControllerType
    VelocityPIDController* pid_controller = static_cast<VelocityPIDController*>(controlnode);
    VelocityRLController* rl_controller = static_cast<VelocityRLController*>(controlnode);
    
    // Try PID first - check if it's actually a PID controller
    if (pid_controller != nullptr) {
        CONTROLLER_TYPE test_type = pid_controller->getControllerType();
        if (test_type == CONTROLLER_TYPE::VELOCITYPID) {
            temp_controllerType = CONTROLLER_TYPE::VELOCITYPID;
            temp_controller_name = pid_controller->getControllerName();
        }
    }
    
    // If not PID, try RL
    if (temp_controller_name == "unknown" && rl_controller != nullptr) {
        CONTROLLER_TYPE test_type = rl_controller->getControllerType();
        if (test_type == CONTROLLER_TYPE::VELOCITYRL) {
            temp_controllerType = CONTROLLER_TYPE::VELOCITYRL;
            temp_controller_name = rl_controller->getControllerName();
        }
    }
    
    if (should_start_logging) {
        log_name = TimeUtils::get_unique_datetime_str() + "_control_logs_" + temp_controller_name;
    }
    
    // Get command from controller - need to cast based on controller type
    Vector3d f_total;
    Matrix3d R_desired;
    Vector3d Omega_desired_frd;
    Eigen::VectorXd obs;
    
    CONTROLLER_TYPE controllerType = CONTROLLER_TYPE::VELOCITYPID;
    std::string controller_name = "unknown";
    
    // Determine controller type and call appropriate method
    // Use getControllerType() to determine the actual controller type
    // (pid_controller and rl_controller are already declared above)
    
    // Check PID controller first
    if (pid_controller != nullptr) {
        CONTROLLER_TYPE test_type = pid_controller->getControllerType();
        if (test_type == CONTROLLER_TYPE::VELOCITYPID) {
            controllerType = CONTROLLER_TYPE::VELOCITYPID;
            controller_name = pid_controller->getControllerName();
            auto result = pid_controller->getCommand(currentBodyState, desiredBodyState, controlType_use, &currentData);
            f_total = std::get<0>(result);
            R_desired = std::get<1>(result);
            Omega_desired_frd = std::get<2>(result);
            obs = std::get<3>(result);
        }
    }
    
    // If not PID, try RL controller
    if (controller_name == "unknown" && rl_controller != nullptr) {
        CONTROLLER_TYPE test_type = rl_controller->getControllerType();
        if (test_type == CONTROLLER_TYPE::VELOCITYRL) {
            controllerType = CONTROLLER_TYPE::VELOCITYRL;
            controller_name = rl_controller->getControllerName();
            
            auto result = rl_controller->getCommand(currentBodyState, desiredBodyState, controlType_use, &currentData);
            f_total = std::get<0>(result);
            R_desired = std::get<1>(result);
            Omega_desired_frd = std::get<2>(result);
            obs = std::get<3>(result);
        }
    }
    
    // Fallback if no controller type was determined
    if (controller_name == "unknown") {
        f_total = Vector3d::Zero();
        R_desired = Matrix3d::Identity();
        Omega_desired_frd = Vector3d::Zero();
        obs = Eigen::VectorXd::Zero(9);
    }
    
    Vector3d command;
    
    // Determine controller type and process command accordingly
    if (controllerType == CONTROLLER_TYPE::VELOCITYPID) {
        Vector3d velCmdDir = Vector3d::Zero();
        if (f_total.norm() > 0) {
            velCmdDir = f_total / f_total.norm();
        }
        double velCmdAbs = f_total.norm();
        double velCmdAbsClipped = std::max(0.0, std::min(velCmdAbs, maximalVelocity));
        command = velCmdDir * velCmdAbsClipped;
    } else if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
        command = f_total;
    } else if (controllerType == CONTROLLER_TYPE::ACCELERATIONPID) {
        Vector3d accCmdDir = f_total / f_total.norm();
        double accCmdAbs = f_total.norm();
        double accCmdAbsClipped = std::max(0.0, std::min(accCmdAbs, MAXIMALACCELERATION));
        command = accCmdDir * accCmdAbsClipped;
    } else {
        Vector3d thrust_cmd = f_total / (MaximalThrust * 0.7);
        command = thrust_cmd.cwiseMin(Vector3d::Constant(_max_thrust))
                         .cwiseMax(Vector3d::Constant(_min_thrust));
    }
    
    Quaternion quat_ned_desbodyfrd = Quaternion::from_matrix(R_desired);
    Vector3d rpyRate_cmd = Omega_desired_frd;
    
    // Handle control logger (depends on log_data flag)
    if (!log_data) {
        // Close control logger when leaving OFFBOARD mode
        if (_control_logger) {
            _control_logger->close();
            _control_logger.reset();
        }
    } else {
        // Create or ensure loggers exist when logging is enabled
        // Use the log_name we already created (or create it if not set)
        if (log_name.empty()) {
            log_name = TimeUtils::get_unique_datetime_str() + "_control_logs_" + controller_name;
        }
        
        // Create control logger if it doesn't exist
        if (!_control_logger) {
            _control_logger = std::make_unique<Logger>(log_name, log_directory, true, false, "CSV");
        }
    }
    
    // Handle RL loggers independently (always create/maintain if RL controller is active)
    if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
        VelocityRLController* rl_controller = static_cast<VelocityRLController*>(controlnode);
        if (rl_controller) {
            // Ensure log_name is set for RL loggers
            if (log_name.empty()) {
                log_name = TimeUtils::get_unique_datetime_str() + "_control_logs_" + controller_name;
            }
            
            auto [vfvr_path, yaw_path] = rl_controller->get_policy_file_paths();
            
            std::string rl_log_name_vfvr = log_name + "_rl_vfvr";
            std::string rl_log_name_yaw = log_name + "_rl_yaw";
            
            // Create vfvr logger if it doesn't exist
            if (!_rl_logger_vfvr) {
                _rl_logger_vfvr = std::make_unique<Logger>(rl_log_name_vfvr, log_directory, true, false, "CSV");
                write_rl_log_metadata(_rl_logger_vfvr.get(), vfvr_path);
            }
            
            // Create yaw logger if it doesn't exist
            if (!_rl_logger_yaw) {
                _rl_logger_yaw = std::make_unique<Logger>(rl_log_name_yaw, log_directory, true, false, "CSV");
                write_rl_log_metadata(_rl_logger_yaw.get(), yaw_path);
            }
        }
    } else {
        // If not RL controller, close RL loggers if they exist
        if (_rl_logger_vfvr) {
            _rl_logger_vfvr->close();
            _rl_logger_vfvr.reset();
        }
        if (_rl_logger_yaw) {
            _rl_logger_yaw->close();
            _rl_logger_yaw.reset();
        }
    }
    
    // Log control data if enabled
    if (log_data && _control_logger) {
        log_control_data(command, rpyRate_cmd, quat_ned_desbodyfrd, Omega_desired_frd,
                        _current_pos_ned, _current_vel_ned, gyro_ned, accel_ned,
                        quat_ned_bodyfrd, imu_ts, step_dt, current_ts, counter,
                        estimated_tar_pos_ned, vel_des_ned, obs, currentData.custom_mode_id,
                        currentData.timestamp / 1000.0);
    }
    
    // Log RL-specific data independently (runs whenever RL loggers exist)
    if (controllerType == CONTROLLER_TYPE::VELOCITYRL && _rl_logger_vfvr && _rl_logger_yaw) {
        VelocityRLController* rl_controller = static_cast<VelocityRLController*>(controlnode);
        if (rl_controller) {
            logRL(rl_controller, f_total, Omega_desired_frd, obs, currentData.timestamp / 1000.0);
        }
    }
    
    return std::make_tuple(command, rpyRate_cmd, quat_ned_desbodyfrd);
}

Quaternion Control::rotate2cameraDirection(const Quaternion& quat_ned_desbodyfrd,
                                          const Quaternion& quat_bodyfrd_cam,
                                          const Vector3d& los_ned_dir,
                                          bool plotFig) {
    Quaternion quat_desbodyfrd_cam = quat_bodyfrd_cam;
    Vector3d rotAxisZfrd_ned = quat_ned_desbodyfrd.rotate_vec(Vector3d(0, 0, 1));
    Vector3d rotAxisZfrd_ned_dir = rotAxisZfrd_ned / rotAxisZfrd_ned.norm();
    Vector3d camZ_ned = (quat_ned_desbodyfrd * quat_desbodyfrd_cam).rotate_vec(Vector3d(0, 0, -1));
    Vector3d camZ_ned_dir = camZ_ned / camZ_ned.norm();
    
    // Find optimal rotation angle
    std::vector<double> los_cam_angle(360);
    double min_angle = std::numeric_limits<double>::max();
    int min_idx = 0;
    
    for (int i = 0; i < 360; i++) {
        double rotAngle = i * M_PI / 180.0;
        Quaternion quat_Z_axis_rot_ned = Quaternion::from_axis_angle(rotAxisZfrd_ned_dir, rotAngle);
        Vector3d camZ_rotated_ned_dir = quat_Z_axis_rot_ned.rotate_vec(camZ_ned_dir);
        Vector3d cross = camZ_rotated_ned_dir.cross(los_ned_dir);
        double dot = camZ_rotated_ned_dir.dot(los_ned_dir);
        los_cam_angle[i] = std::atan2(cross.norm(), dot);
        
        if (los_cam_angle[i] < min_angle) {
            min_angle = los_cam_angle[i];
            min_idx = i;
        }
    }
    
    double rotAngle = min_idx * M_PI / 180.0;
    Quaternion quat_Z_axis_rot_ned = Quaternion::from_axis_angle(rotAxisZfrd_ned_dir, rotAngle);
    
    // Ensure quaternion is in same hemisphere
    if (quat_Z_axis_rot_ned.dot(quat_ned_desbodyfrd) < 0) {
        quat_Z_axis_rot_ned = -quat_Z_axis_rot_ned;
    }
    
    Quaternion quat_ned_desbodyfrd_new = quat_Z_axis_rot_ned * quat_ned_desbodyfrd;
    return quat_ned_desbodyfrd_new;
}

void Control::log_control_data(const Vector3d& command, const Vector3d& rpy_rate_cmd,
                               const Quaternion& quat_ned_desbodyfrd_cmd, const Vector3d& Omega_desired_frd,
                               const Vector3d& current_pos_ned, const Vector3d& cur_vel_ned,
                               const Vector3d& gyro_ned, const Vector3d& accel_ned,
                               const Quaternion& quat_ned_bodyfrd, int64_t imu_ts, double dt,
                               int64_t current_ts, int counter,
                               const Vector3d& est_tar_pos_ned,
                               const Vector3d& vel_des_ned,
                               const Eigen::VectorXd& obs,
                               int modeID, double timestamp) {
    std::map<std::string, std::string> logDict;
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(10);
    
    oss.str("");
    oss << timestamp;
    logDict["timestamp"] = oss.str();
    
    oss.str("");
    oss << TimeUtils::now();
    logDict["comp_time"] = oss.str();
    
    oss.str("");
    oss << command[0];
    logDict["command/[0]"] = oss.str();
    oss.str("");
    oss << command[1];
    logDict["command/[1]"] = oss.str();
    oss.str("");
    oss << command[2];
    logDict["command/[2]"] = oss.str();
    
    oss.str("");
    oss << rpy_rate_cmd[0];
    logDict["rate_cmd/roll"] = oss.str();
    oss.str("");
    oss << rpy_rate_cmd[1];
    logDict["rate_cmd/pitch"] = oss.str();
    oss.str("");
    oss << rpy_rate_cmd[2];
    logDict["rate_cmd/yaw"] = oss.str();
    
    oss.str("");
    oss << quat_ned_desbodyfrd_cmd.x;
    logDict["quat_ned_desbodyfrd_cmd/x"] = oss.str();
    oss.str("");
    oss << quat_ned_desbodyfrd_cmd.y;
    logDict["quat_ned_desbodyfrd_cmd/y"] = oss.str();
    oss.str("");
    oss << quat_ned_desbodyfrd_cmd.z;
    logDict["quat_ned_desbodyfrd_cmd/z"] = oss.str();
    oss.str("");
    oss << quat_ned_desbodyfrd_cmd.w;
    logDict["quat_ned_desbodyfrd_cmd/w"] = oss.str();
    
    oss.str("");
    oss << current_pos_ned[0];
    logDict["current_pos_ned/x"] = oss.str();
    oss.str("");
    oss << current_pos_ned[1];
    logDict["current_pos_ned/y"] = oss.str();
    oss.str("");
    oss << current_pos_ned[2];
    logDict["current_pos_ned/z"] = oss.str();
    
    oss.str("");
    oss << cur_vel_ned[0];
    logDict["cur_vel_ned/x"] = oss.str();
    oss.str("");
    oss << cur_vel_ned[1];
    logDict["cur_vel_ned/y"] = oss.str();
    oss.str("");
    oss << cur_vel_ned[2];
    logDict["cur_vel_ned/z"] = oss.str();
    
    oss.str("");
    oss << gyro_ned[0];
    logDict["gyro_ned/x"] = oss.str();
    oss.str("");
    oss << gyro_ned[1];
    logDict["gyro_ned/y"] = oss.str();
    oss.str("");
    oss << gyro_ned[2];
    logDict["gyro_ned/z"] = oss.str();
    
    oss.str("");
    oss << accel_ned[0];
    logDict["accel_ned/x"] = oss.str();
    oss.str("");
    oss << accel_ned[1];
    logDict["accel_ned/y"] = oss.str();
    oss.str("");
    oss << accel_ned[2];
    logDict["accel_ned/z"] = oss.str();
    
    oss.str("");
    oss << Omega_desired_frd[0];
    logDict["Omega_desired_frd/x"] = oss.str();
    oss.str("");
    oss << Omega_desired_frd[1];
    logDict["Omega_desired_frd/y"] = oss.str();
    oss.str("");
    oss << Omega_desired_frd[2];
    logDict["Omega_desired_frd/z"] = oss.str();
    
    oss.str("");
    oss << quat_ned_bodyfrd.x;
    logDict["quat_ned_bodyfrd/x"] = oss.str();
    oss.str("");
    oss << quat_ned_bodyfrd.y;
    logDict["quat_ned_bodyfrd/y"] = oss.str();
    oss.str("");
    oss << quat_ned_bodyfrd.z;
    logDict["quat_ned_bodyfrd/z"] = oss.str();
    oss.str("");
    oss << quat_ned_bodyfrd.w;
    logDict["quat_ned_bodyfrd/w"] = oss.str();
    
    oss.str("");
    oss << est_tar_pos_ned[0];
    logDict["est_tar_pos_ned/x"] = oss.str();
    oss.str("");
    oss << est_tar_pos_ned[1];
    logDict["est_tar_pos_ned/y"] = oss.str();
    oss.str("");
    oss << est_tar_pos_ned[2];
    logDict["est_tar_pos_ned/z"] = oss.str();
    
    oss.str("");
    oss << vel_des_ned[0];
    logDict["vel_des_ned/x"] = oss.str();
    oss.str("");
    oss << vel_des_ned[1];
    logDict["vel_des_ned/y"] = oss.str();
    oss.str("");
    oss << vel_des_ned[2];
    logDict["vel_des_ned/z"] = oss.str();
    
    oss.str("");
    oss << imu_ts;
    logDict["imu_ts"] = oss.str();
    oss.str("");
    oss << dt;
    logDict["dt"] = oss.str();
    oss.str("");
    oss << current_ts;
    logDict["current_ts"] = oss.str();
    oss.str("");
    oss << counter;
    logDict["counter"] = oss.str();
    oss.str("");
    oss << modeID;
    logDict["modeID"] = oss.str();
    
    if (obs.size() > 0) {
        for (int i = 0; i < obs.size(); i++) {
            oss.str("");
            oss << obs[i];
            logDict["obs/" + std::to_string(i)] = oss.str();
        }
    }
    
    if (_control_logger) {
        _control_logger->log(logDict);
    }
}

void Control::write_rl_log_metadata(Logger* logger, const std::string& policy_file_path) {
    if (!logger) {
        return;
    }
    
    // Write metadata as a comment line (CSV comment convention)
    std::string metadata = "# policy_file: " + policy_file_path + "\n";
    logger->write(metadata);
}

void Control::logRL(VelocityRLController* rl_controller, const Vector3d& f_total, const Vector3d& Omega_desired_frd,
                    const Eigen::VectorXd& obs, double timestamp) {
    if (!rl_controller || !_rl_logger_vfvr || !_rl_logger_yaw) {
        return;
    }
    
    try {
        // Get action distribution (mean and logstd)
        auto [mean_vfvr, logstd_vfvr, mean_yaw, logstd_yaw] = rl_controller->get_action_distribution();
        
        // Get hidden states
        auto [hxs_vfvr, hxs_yaw] = rl_controller->get_hidden_states();
        
        // Extract actions: X (vf), Y (vr), yaw (w)
        double action_x = f_total[0];  // vf
        double action_y = f_total[1];  // vr
        double action_yaw = Omega_desired_frd[2];  // w
        
        // Extract observations: obsXY (4 values) and obsHeading (2 values)
        // obs contains [obsXY[0], obsXY[1], obsXY[2], obsXY[3], obsHeading[0], obsHeading[1]]
        Eigen::Vector4d obsXY = Eigen::Vector4d::Zero();
        Eigen::Vector2d obsHeading = Eigen::Vector2d::Zero();
        
        if (obs.size() >= 6) {
            obsXY << obs[0], obs[1], obs[2], obs[3];
            obsHeading << obs[4], obs[5];
        }
        
        // Build log dictionary
        std::map<std::string, std::string> logDict;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(10);
        
        // Timestamp
        oss.str("");
        oss << timestamp;
        logDict["timestamp"] = oss.str();
        
        // Actions
        oss.str("");
        oss << action_x;
        logDict["action/x"] = oss.str();
        oss.str("");
        oss << action_y;
        logDict["action/y"] = oss.str();
        oss.str("");
        oss << action_yaw;
        logDict["action/yaw"] = oss.str();
        
        // Action distribution for vfvr
        for (int i = 0; i < mean_vfvr.size(); i++) {
            oss.str("");
            oss << mean_vfvr[i];
            logDict["mean_vfvr/" + std::to_string(i)] = oss.str();
        }
        for (int i = 0; i < logstd_vfvr.size(); i++) {
            oss.str("");
            oss << logstd_vfvr[i];
            logDict["logstd_vfvr/" + std::to_string(i)] = oss.str();
        }
        
        // Action distribution for yaw
        for (int i = 0; i < mean_yaw.size(); i++) {
            oss.str("");
            oss << mean_yaw[i];
            logDict["mean_yaw/" + std::to_string(i)] = oss.str();
        }
        for (int i = 0; i < logstd_yaw.size(); i++) {
            oss.str("");
            oss << logstd_yaw[i];
            logDict["logstd_yaw/" + std::to_string(i)] = oss.str();
        }
        
        // Observations for vfvr network (obsXY)
        for (int i = 0; i < obsXY.size(); i++) {
            oss.str("");
            oss << obsXY[i];
            logDict["obsXY/" + std::to_string(i)] = oss.str();
        }
        
        // Observations for yaw network (obsHeading)
        for (int i = 0; i < obsHeading.size(); i++) {
            oss.str("");
            oss << obsHeading[i];
            logDict["obsHeading/" + std::to_string(i)] = oss.str();
        }
        
        // Hidden states for vfvr network
        for (int i = 0; i < hxs_vfvr.size(); i++) {
            oss.str("");
            oss << hxs_vfvr[i];
            logDict["hxs_vfvr/" + std::to_string(i)] = oss.str();
        }
        
        // Build separate log dictionaries for vfvr and yaw
        // Note: We'll add hxs last to ensure it appears at the end of the CSV row
        std::map<std::string, std::string> logDict_vfvr;
        std::map<std::string, std::string> logDict_yaw;
        
        // Common fields
        logDict_vfvr["timestamp"] = logDict["timestamp"];
        logDict_yaw["timestamp"] = logDict["timestamp"];
        
        // Vfvr-specific data (add in order: action, mean, logstd, obs, then hxs last)
        logDict_vfvr["action/x"] = logDict["action/x"];
        logDict_vfvr["action/y"] = logDict["action/y"];
        for (int i = 0; i < mean_vfvr.size(); i++) {
            logDict_vfvr["mean/" + std::to_string(i)] = logDict["mean_vfvr/" + std::to_string(i)];
        }
        for (int i = 0; i < logstd_vfvr.size(); i++) {
            logDict_vfvr["logstd/" + std::to_string(i)] = logDict["logstd_vfvr/" + std::to_string(i)];
        }
        for (int i = 0; i < obsXY.size(); i++) {
            logDict_vfvr["obs/" + std::to_string(i)] = logDict["obsXY/" + std::to_string(i)];
        }
        // Write hxs_vfvr directly from the vector (add last to ensure it's at the end)
        // Use zero-padded indices so alphabetical sorting matches numerical sorting
        // Calculate number of digits needed for zero-padding
        int hxs_vfvr_max_idx = hxs_vfvr.size() > 0 ? static_cast<int>(hxs_vfvr.size() - 1) : 0;
        int hxs_vfvr_digits = hxs_vfvr_max_idx > 0 ? static_cast<int>(std::floor(std::log10(static_cast<double>(hxs_vfvr_max_idx))) + 1) : 1;
        for (int i = 0; i < hxs_vfvr.size(); i++) {
            oss.str("");
            oss << hxs_vfvr[i];
            // Format index with zero-padding for proper numerical sorting
            std::ostringstream idx_oss;
            idx_oss << std::setfill('0') << std::setw(hxs_vfvr_digits) << i;
            logDict_vfvr["z_hxs/" + idx_oss.str()] = oss.str();
        }
        
        // Yaw-specific data (add in order: action, mean, logstd, obs, then hxs last)
        logDict_yaw["action/yaw"] = logDict["action/yaw"];
        for (int i = 0; i < mean_yaw.size(); i++) {
            logDict_yaw["mean/" + std::to_string(i)] = logDict["mean_yaw/" + std::to_string(i)];
        }
        for (int i = 0; i < logstd_yaw.size(); i++) {
            logDict_yaw["logstd/" + std::to_string(i)] = logDict["logstd_yaw/" + std::to_string(i)];
        }
        for (int i = 0; i < obsHeading.size(); i++) {
            logDict_yaw["obs/" + std::to_string(i)] = logDict["obsHeading/" + std::to_string(i)];
        }
        // Write hxs_yaw directly from the vector (add last to ensure it's at the end)
        // Use zero-padded indices so alphabetical sorting matches numerical sorting
        int hxs_yaw_max_idx = hxs_yaw.size() > 0 ? static_cast<int>(hxs_yaw.size() - 1) : 0;
        int hxs_yaw_digits = hxs_yaw_max_idx > 0 ? static_cast<int>(std::floor(std::log10(static_cast<double>(hxs_yaw_max_idx))) + 1) : 1;
        for (int i = 0; i < hxs_yaw.size(); i++) {
            oss.str("");
            oss << hxs_yaw[i];
            // Format index with zero-padding for proper numerical sorting
            std::ostringstream idx_oss;
            idx_oss << std::setfill('0') << std::setw(hxs_yaw_digits) << i;
            logDict_yaw["z_hxs/" + idx_oss.str()] = oss.str();
        }
        
        // Write to separate loggers
        _rl_logger_vfvr->log(logDict_vfvr);
        _rl_logger_yaw->log(logDict_yaw);
        
    } catch (const std::exception& e) {
        std::cerr << "Warning: Exception while logging RL data: " << e.what() << std::endl;
    }
}

