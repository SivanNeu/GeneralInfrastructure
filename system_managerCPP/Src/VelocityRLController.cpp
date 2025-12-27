#include "VelocityRLController.h"
#include "general.h"
#include "utils/FlightData.h"
#include <algorithm>
#include <cmath>

VelocityRLControllerParameters::VelocityRLControllerParameters(double mass) : mass(mass) {
}

VelocityRLController::VelocityRLController(double mass, double maximalVelocity, double currentTime)
    : controllerName("VelocityRL"), controllerType(CONTROLLER_TYPE::VELOCITYRL),
      lastTime(currentTime), current_time(currentTime),
      pos_self(Vector3d::Zero()), vel_self(Vector3d::Zero()),
      pos_target(Vector3d::Zero()), heading_target(Vector3d::Zero()),
      vel_target(Vector3d::Zero()),
      max_vel(maximalVelocity), max_range(15.0), int_scale(max_range * 20.0),
      max_omega(M_PI / 2.0), ringLen(1), ringV(Eigen::MatrixXd::Zero(2, ringLen)),
      ringIndex(0), ringAverage(Eigen::Vector2d::Zero()), param(mass),
      rl_policyVfVr(nullptr), rl_policyOmegaYaw(nullptr) {
    // RL policy loading - paths would be configurable
    std::string rlFilePathVfVr = "./train_dir/rlcat2_quad/checkpoint_p0/best_000003172_3248128_reward_176.079.pth";
    std::string rlFilePathOmegaYaw = "./train_dir/rlcat2_yawrate/checkpoint_p0/best_000008610_8816640_reward_4791.792.pth";
    
    try {
        std::cout << "Loading RL policy from: " << rlFilePathVfVr << std::endl;
        rl_policyVfVr = RLPolicyClean::load_from_checkpoint(rlFilePathVfVr, "cpu", "relu", false);
        
        std::cout << "Loading RL policy from: " << rlFilePathOmegaYaw << std::endl;
        rl_policyOmegaYaw = RLPolicyClean::load_from_checkpoint(rlFilePathOmegaYaw, "cpu", "relu", false);
    } catch (const std::exception& e) {
        std::cerr << "Warning: Failed to load RL policies: " << e.what() << std::endl;
        std::cerr << "RL controller will use placeholder inference" << std::endl;
    }
}

std::tuple<Vector3d, Matrix3d, Vector3d, Eigen::VectorXd> VelocityRLController::getCommand(
    const std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Quaternion>& currentBodyState,
    const std::tuple<std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>,
                    std::tuple<Vector3d, Vector3d, Vector3d>>& desiredBodyState,
    const std::vector<bool>& controlType,
    Flight_Data* currentData) {
    
    auto [pos_ned, vel_ned, accel_ned, gyro_ned, quat_ned_bodyfrd] = currentBodyState;
    auto [desired_state, desired_heading] = desiredBodyState;
    auto [pos_target_state, vel_target_state, accel_target, omega_target, alpha_target] = desired_state;
    auto [heading_target_state, heading_dot, heading_2dot] = desired_heading;
    
    pos_self = pos_ned;
    vel_self = vel_ned;
    pos_target = pos_target_state;
    vel_target = vel_target_state;
    heading_target = heading_target_state;
    heading_target.normalize();
    
    Vector3d gyro_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(gyro_ned);
    
    // Heading Observation
    Vector3d curHeading = quat_ned_bodyfrd.rotate_vec(Vector3d(1, 0, 0));
    Vector3d cross = curHeading.cross(heading_target);
    double cross_z = cross[2];
    double dot = curHeading.dot(heading_target);
    double theta = std::atan2(cross_z, dot);
    Eigen::Vector2d obsHeading(theta, gyro_bodyfrd[2]);
    
    // Transform into observation space
    Eigen::Vector2d pos_error_ned = (pos_target - pos_self).head<2>();
    Eigen::Vector2d vel_error_ned = vel_self.head<2>();
    
    if (pos_error_ned.norm() > max_range) {
        pos_error_ned = pos_error_ned / pos_error_ned.norm() * max_range;
    }
    if (vel_error_ned.norm() > max_vel) {
        vel_error_ned = vel_error_ned / vel_error_ned.norm() * max_vel;
    }
    
    // Update time
    if (currentData != nullptr) {
        current_time = currentData->local_ts;
    }
    double dt = current_time - lastTime;
    if (dt < 0) {
        dt = 0;
        lastTime = current_time;
    }
    lastTime = current_time;
    
    Eigen::Vector4d obsXY;
    obsXY << pos_error_ned[0], pos_error_ned[1], vel_error_ned[0], vel_error_ned[1];
    
    // Execute policy inference
    auto [vf_vr, w] = rl_inference(obsXY, obsHeading);
    double vf_ = vf_vr[0];
    double vr_ = vf_vr[1];
    
    double vf = std::max(-max_vel, std::min(max_vel, vf_));
    double vr = std::max(-max_vel, std::min(max_vel, vr_));
    double w_clipped = std::max(-max_omega, std::min(max_omega, w));
    
    Vector3d vel_vector(vf, vr, 0);  // in FRD
    Vector3d omega_vector(0, 0, w_clipped);
    
    Eigen::VectorXd obsTotal(6);
    obsTotal << obsXY, obsHeading;
    
    return std::make_tuple(vel_vector, Matrix3d::Identity(), omega_vector, obsTotal);
}

void VelocityRLController::resetIntegralErrorTerms() {
    // No integral terms in RL controller
}

std::pair<Eigen::Vector2d, double> VelocityRLController::rl_inference(const Eigen::Vector4d& obsXY, const Eigen::Vector2d& obsHeading) {
    Eigen::Vector2d vf_vr(0.0, 0.0);
    double w = 0.0;
    
    if (rl_policyVfVr && rl_policyOmegaYaw) {
        try {
            // Convert Eigen to VectorXd for policy input
            Eigen::VectorXd obsXY_vec(4);
            obsXY_vec << obsXY[0], obsXY[1], obsXY[2], obsXY[3];
            
            Eigen::VectorXd obsHeading_vec(2);
            obsHeading_vec << obsHeading[0], obsHeading[1];
            
            // Forward pass through policies
            auto [action_logits_vfvr, hxs_vfvr] = rl_policyVfVr->forward(obsXY_vec, false);
            auto [action_logits_yaw, hxs_yaw] = rl_policyOmegaYaw->forward(obsHeading_vec, false);
            
            // Extract mean and logstd (action_logits contains [mean, logstd] concatenated)
            int action_dim = action_logits_vfvr.size() / 2;
            Eigen::VectorXd mean_vfvr = action_logits_vfvr.head(action_dim);
            Eigen::VectorXd logstd_vfvr = action_logits_vfvr.tail(action_dim);
            
            // Sample or use mean (for deterministic, use mean)
            vf_vr[0] = mean_vfvr[0];
            vf_vr[1] = mean_vfvr[1];
            
            int yaw_action_dim = action_logits_yaw.size() / 2;
            Eigen::VectorXd mean_yaw = action_logits_yaw.head(yaw_action_dim);
            w = mean_yaw[0];
            
        } catch (const std::exception& e) {
            std::cerr << "Error in RL inference: " << e.what() << std::endl;
            // Fallback to zeros
            vf_vr = Eigen::Vector2d::Zero();
            w = 0.0;
        }
    } else {
        // Policies not loaded - return zeros
        vf_vr = Eigen::Vector2d::Zero();
        w = 0.0;
    }
    
    return std::make_pair(vf_vr, w);
}

