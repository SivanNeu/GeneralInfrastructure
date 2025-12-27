#include "VelocityPIDController.h"
#include "general.h"
#include "utils/FlightData.h"
#include <algorithm>
#include <cmath>

VelocityPIDControllerParameters::VelocityPIDControllerParameters(double mass) : mass(mass) {
    double commonFactor = 2.0;
    kX = Matrix3d::Identity() * commonFactor;
    kV = Matrix3d::Identity() * commonFactor * 0.0;
    kIX = Matrix3d::Identity() * 0.0 * commonFactor;
    kIV = Matrix3d::Identity() * 0.0;
    c1 = 1.0;
    sat_sigmaX = 3.0;
    sat_sigmaV = 3.0;
    keYaw = 1.0;
    keYawRate = 0.01;
    keYawIntegral = 1.0 * 0.0;
    sat_sigmaYaw = 3.0;
}

VelocityPIDController::VelocityPIDController(double mass, double currentTime, YAW_COMMAND_TYPE yawCommandType)
    : controllerName("VelocityPID"), controllerType(CONTROLLER_TYPE::VELOCITYPID),
      yawCommandType(yawCommandType), t0(currentTime), t(0.0), t_pre(0.0), dt(1e-9),
      use_integralTerm(true), A(Vector3d::Zero()), param(mass),
      ex(Vector3d::Zero()), ev(Vector3d::Zero()),
      eYaw(0.0), prev_eYaw(std::nullopt), eYawRate(0.0), eYawIntegral(0.0),
      eYawRateFilter(0.2, true, LPF_TYPE::FIRST_ORDER), eDV(Vector3d::Zero()) {
}

void VelocityPIDController::resetIntegralErrorTerms() {
    eIX.set_zero();
    eIV.set_zero();
}

std::tuple<Vector3d, Matrix3d, Vector3d, Eigen::VectorXd> VelocityPIDController::getCommand(
    const std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Quaternion>& currentBodyState,
    const std::tuple<std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>,
                    std::tuple<Vector3d, Vector3d, Vector3d>>& desiredBodyState,
    const std::vector<bool>& controlType,
    Flight_Data* currentData) {
    
    Vector3d pos_control = Vector3d::Ones();
    Vector3d vel_control = Vector3d::Ones();
    
    if (!controlType.empty()) {
        for (int i = 0; i < 3 && i < static_cast<int>(controlType.size()); i++) {
            pos_control[i] = controlType[i] ? 1.0 : 0.0;
        }
        if (controlType.size() > 3) {
            for (int i = 0; i < 3 && (i + 3) < static_cast<int>(controlType.size()); i++) {
                vel_control[i] = controlType[i + 3] ? 1.0 : 0.0;
            }
        }
    }
    
    auto [pos_ned, vel_ned, accel_ned, omega_ned, quat_ned_bodyfrd] = currentBodyState;
    auto [desired_state, desired_heading] = desiredBodyState;
    auto [xd, xd_dot, xd_2dot, xd_3dot, xd_4dot] = desired_state;
    auto [b1d, b1d_dot, b1d_2dot] = desired_heading;
    
    Matrix3d kX = param.kX * pos_control.asDiagonal();
    Matrix3d kIX = param.kIX * pos_control.asDiagonal();
    Matrix3d kV = param.kV * (vel_control.cwiseMax(pos_control)).asDiagonal();
    Matrix3d kIV = param.kIV * vel_control.asDiagonal();
    
    if (currentData != nullptr) {
        update_current_time(currentData->local_ts);
    }
    dt = t - t_pre;
    
    Vector3d eX = (xd - pos_ned).cwiseProduct(pos_control);
    Vector3d eV = xd_dot - vel_ned;
    
    if (use_integralTerm) {
        eIX.integrate(param.c1 * eX + eV, dt);
        eIX.error = saturate(eIX.error, -param.sat_sigmaX, param.sat_sigmaX);
        eIV.integrate(eV, dt);
        eIV.error = saturate(eIV.error, -param.sat_sigmaV, param.sat_sigmaV);
    }
    
    Vector3d velocity_command = kX * eX + kV * eV + kIX * eIX.error + kIV * eIV.error + xd_2dot;
    
    A = velocity_command;
    
    double yaw_command = 0.0;
    Eigen::VectorXd obs(9);
    obs << pos_ned, vel_ned, omega_ned;
    
    if (yawCommandType == YAW_COMMAND_TYPE::RATE) {
        Vector3d b1d_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(b1d);
        eYaw = std::atan2(b1d_bodyfrd[1], b1d_bodyfrd[0]);
        if (prev_eYaw.has_value() && dt > 0.0) {
            eYawRate = eYawRateFilter.step((eYaw - prev_eYaw.value()) / dt);
        } else {
            eYawRate = 0.0;
        }
        prev_eYaw = eYaw;
        eYawIntegral += eYaw * dt;
        eYawIntegral = std::max(-param.sat_sigmaYaw, std::min(param.sat_sigmaYaw, eYawIntegral));
        yaw_command = param.keYaw * eYaw - param.keYawRate * eYawRate + param.keYawIntegral * eYawIntegral;
    }
    
    Vector3d rpyRate_cmd(0.0, 0.0, yaw_command);
    return std::make_tuple(velocity_command, Matrix3d::Identity(), rpyRate_cmd, obs);
}

void VelocityPIDController::update_current_time(double currentTime) {
    t_pre = t;
    t = currentTime - t0;
}

