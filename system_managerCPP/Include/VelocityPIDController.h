#ifndef VELOCITY_PID_CONTROLLER_H
#define VELOCITY_PID_CONTROLLER_H

#include "utils/ControllerType.h"
#include "utils/YawCommandType.h"
#include "utils/LowPassFilter.h"
#include "IntegralUtils.h"
#include "MatrixUtils.h"
#include "utils/Quaternion.h"
#include "utils/FlightData.h"
#include <Eigen/Dense>
#include <memory>

// Forward declaration
class Flight_Data;

struct VelocityPIDControllerParameters {
    double mass;
    Eigen::Matrix3d kX;
    Eigen::Matrix3d kV;
    Eigen::Matrix3d kIX;
    Eigen::Matrix3d kIV;
    double c1;
    double sat_sigmaX;
    double sat_sigmaV;
    double keYaw;
    double keYawRate;
    double keYawIntegral;
    double sat_sigmaYaw;

    VelocityPIDControllerParameters(double mass = 0.5);
};

class VelocityPIDController {
private:
    std::string controllerName;
    CONTROLLER_TYPE controllerType;
    YAW_COMMAND_TYPE yawCommandType;
    double t0;
    double t;
    double t_pre;
    double dt;
    bool use_integralTerm;
    Eigen::Vector3d A;
    
    VelocityPIDControllerParameters param;
    
    Eigen::Vector3d ex;
    Eigen::Vector3d ev;
    IntegralErrorVec3 eIX;
    IntegralErrorVec3 eIV;
    double eYaw;
    std::optional<double> prev_eYaw;
    double eYawRate;
    double eYawIntegral;
    Low_Pass_Filter eYawRateFilter;
    Eigen::Vector3d eDV;

public:
    VelocityPIDController(double mass, double currentTime = 0.0, YAW_COMMAND_TYPE yawCommandType = YAW_COMMAND_TYPE::NONE);
    
    void resetIntegralErrorTerms();
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::VectorXd> getCommand(
        const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Quaternion>& currentBodyState,
        const std::tuple<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>,
                        std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& desiredBodyState,
        const std::vector<bool>& controlType = {},
        Flight_Data* currentData = nullptr);
    
    void update_current_time(double currentTime);
    
    std::string getControllerName() const { return controllerName; }
    CONTROLLER_TYPE getControllerType() const { return controllerType; }
};

#endif // VELOCITY_PID_CONTROLLER_H

