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
    
    // Get command from controller - need to cast based on controller type
    Vector3d f_total;
    Matrix3d R_desired;
    Vector3d Omega_desired_frd;
    Eigen::VectorXd obs;
    
    CONTROLLER_TYPE controllerType = CONTROLLER_TYPE::VELOCITYPID;
    
    // Try to determine controller type and call appropriate method
    // This would be better with a base Controller interface
    try {
        VelocityPIDController* pid_controller = static_cast<VelocityPIDController*>(controlnode);
        controllerType = pid_controller->getControllerType();
        auto result = pid_controller->getCommand(currentBodyState, desiredBodyState, controlType_use, &currentData);
        f_total = std::get<0>(result);
        R_desired = std::get<1>(result);
        Omega_desired_frd = std::get<2>(result);
        obs = std::get<3>(result);
    } catch (...) {
        try {
            VelocityRLController* rl_controller = static_cast<VelocityRLController*>(controlnode);
            controllerType = rl_controller->getControllerType();
            auto result = rl_controller->getCommand(currentBodyState, desiredBodyState, controlType_use, &currentData);
            f_total = std::get<0>(result);
            R_desired = std::get<1>(result);
            Omega_desired_frd = std::get<2>(result);
            obs = std::get<3>(result);
        } catch (...) {
            // Fallback - would need other controller types
            f_total = Vector3d::Zero();
            R_desired = Matrix3d::Identity();
            Omega_desired_frd = Vector3d::Zero();
            obs = Eigen::VectorXd::Zero(9);
        }
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
    
    if (!log_data) {
        _control_logger.reset();
    } else if (log_data && !_control_logger) {
        std::string log_name = TimeUtils::get_unique_datetime_str() + "_control_logs_" + "controller";
        _control_logger = std::make_unique<Logger>(log_name, log_directory, true, false, "CSV");
    }
    
    if (log_data && _control_logger) {
        log_control_data(command, rpyRate_cmd, quat_ned_desbodyfrd, Omega_desired_frd,
                        _current_pos_ned, _current_vel_ned, gyro_ned, accel_ned,
                        quat_ned_bodyfrd, imu_ts, step_dt, current_ts, counter,
                        estimated_tar_pos_ned, vel_des_ned, obs, currentData.custom_mode_id,
                        currentData.timestamp / 1000.0);
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

