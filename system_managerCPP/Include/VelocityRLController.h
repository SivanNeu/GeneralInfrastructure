#ifndef VELOCITY_RL_CONTROLLER_H
#define VELOCITY_RL_CONTROLLER_H

#include "utils/ControllerType.h"
#include "utils/Quaternion.h"
#include "utils/FlightData.h"
#include "RLPolicyClean.h"
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>

// Forward declaration for RL policy (Python-specific, will need implementation)
// class RLPolicyClean;

struct VelocityRLControllerParameters {
    double mass;
    VelocityRLControllerParameters(double mass = 0.5);
};

class VelocityRLController {
private:
    std::string controllerName;
    CONTROLLER_TYPE controllerType;
    double lastTime;
    double current_time;
    
    Eigen::Vector3d pos_self;
    Eigen::Vector3d vel_self;
    Eigen::Vector3d pos_target;
    Eigen::Vector3d heading_target;
    Eigen::Vector3d vel_target;
    
    double max_vel;
    double max_range;
    double int_scale;
    double max_omega;
    
    int ringLen;
    Eigen::MatrixXd ringV;
    int ringIndex;
    Eigen::Vector2d ringAverage;
    
    VelocityRLControllerParameters param;
    
    // RL Policies
    std::shared_ptr<RLPolicyClean> rl_policyVfVr;
    std::shared_ptr<RLPolicyClean> rl_policyOmegaYaw;

public:
    VelocityRLController(double mass = 0.5, double maximalVelocity = 3.0, double currentTime = 0.0);
    
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::VectorXd> getCommand(
        const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Quaternion>& currentBodyState,
        const std::tuple<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>,
                        std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& desiredBodyState,
        const std::vector<bool>& controlType = {},
        Flight_Data* currentData = nullptr);
    
    void resetIntegralErrorTerms();
    
    std::string getControllerName() const { return controllerName; }
    CONTROLLER_TYPE getControllerType() const { return controllerType; }
    
    // RL policy inference
    std::pair<Eigen::Vector2d, double> rl_inference(const Eigen::Vector4d& obsXY, const Eigen::Vector2d& obsHeading);
};

#endif // VELOCITY_RL_CONTROLLER_H

