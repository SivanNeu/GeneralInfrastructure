#ifndef VELOCITY_RL_CONTROLLER_H
#define VELOCITY_RL_CONTROLLER_H
#include "general.h"

#include "utils/ControllerType.h"
#include "utils/Quaternion.h"
#include "utils/FlightData.h"
#include "RLPolicyClean.h"
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <regex>

// Forward declaration for RL policy (Python-specific, will need implementation)
// class RLPolicyClean;

struct VelocityRLControllerParameters {
    double mass;
    double max_vel;
    double max_range;
    double int_scale;
    double max_omega;
    std::string rlFilePathVfVr;
    std::string rlFilePathOmegaYaw;
    
    VelocityRLControllerParameters(double mass = 0.5);
    static VelocityRLControllerParameters loadFromJSON(const std::string& jsonFilePath);
};

class VelocityRLController {
private:
    std::string controllerName;
    CONTROLLER_TYPE controllerType;
    double lastTime;
    double current_time;
    
    Vector3d pos_self;
    Vector3d vel_self;
    Vector3d pos_target;
    Vector3d heading_target;
    Vector3d vel_target;
    
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
    VelocityRLController(const VelocityRLControllerParameters& params, double maximalVelocity = 3.0, double currentTime = 0.0);
    
    std::tuple<Vector3d, Matrix3d, Vector3d, Eigen::VectorXd> getCommand(
        const std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Quaternion>& currentBodyState,
        const std::tuple<std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>,
                        std::tuple<Vector3d, Vector3d, Vector3d>>& desiredBodyState,
        const std::vector<bool>& controlType = {},
        Flight_Data* currentData = nullptr);
    
    void resetIntegralErrorTerms();
    
    std::string getControllerName() const { return controllerName; }
    CONTROLLER_TYPE getControllerType() const { return controllerType; }
    
    // RL policy inference
    std::pair<Eigen::Vector2d, double> rl_inference(const Eigen::Vector4d& obsXY, const Eigen::Vector2d& obsHeading);
};

#endif // VELOCITY_RL_CONTROLLER_H

