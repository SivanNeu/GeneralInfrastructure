#ifndef CONTROL_H
#define CONTROL_H
#include "general.h"

#include "utils/ControllerType.h"
#include "utils/YawCommand.h"
#include "utils/Quaternion.h"
#include "utils/FlightData.h"
#include "utils/Logger.h"
#include "utils/TimeUtils.h"
#include "MatrixUtils.h"
#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <optional>
#include <map>

// Forward declarations
class VelocityPIDController;
class VelocityRLController;

// Guidance class definition (needed for unique_ptr)
class Guidance {
public:
    Guidance(double N, int type) {}
};

// Constants
constexpr double GRAVITYACCL = -9.80665;
constexpr bool ADD_NOISE = false;
constexpr double MAX_THRUST_KG = 2.14;
constexpr double PITCHLIMITDEG = 10.0;
constexpr double ROLLLIMITDEG = 10.0;
constexpr double MAXIMALVELOCITY = 10.0;
constexpr double MAXIMALACCELERATION = 100.0;

enum class HOMING_STAGE {
    NONE = 0,
    SECTION = 1,
    VELOCITY = 2,
    PURSUIT = 3,
    VELOCITY_ENDGAME = 4
};

class Control {
private:
    std::string log_directory;
    double maximalVelocity;
    std::optional<double> _start_vertical_los_deg;
    std::optional<Vector3d> _noise_sim;
    
    double _min_thrust;
    double _max_thrust;
    double MaximalThrust;
    bool enableIntegrator;
    
    // Controller node (polymorphic - can be VelocityPIDController, VelocityRLController, etc.)
    void* controlnode;  // Using void* for now, could use base class pointer
    
    Vector3d _accel_cmd_ned;
    std::unique_ptr<Logger> _control_logger;
    
    double _last_pitch_update;
    bool _finished_stationary_tracking;
    Vector3d _current_pos_ned;
    Vector3d _current_vel_ned;
    
    std::optional<Vector3d> prev_los_distance;
    std::optional<Vector3d> prev_los_ned_dir;
    
    std::unique_ptr<Guidance> _guidance;

public:
    Control(const std::string& log_directory, 
            void* controller, double maximalVelocity = MAXIMALVELOCITY);
    
    std::tuple<Vector3d, Vector3d, Quaternion> get_cmd(
        const Vector3d& pos_ned, const Vector3d& vel_ned,
        const Vector3d& accel_ned, const Vector3d& gyro_ned,
        const Quaternion& quat_ned_bodyfrd, int64_t imu_ts, double step_dt,
        int64_t current_ts, int counter,
        const std::tuple<Vector3d, Vector3d, Vector3d>& trajDest_ned,
        Flight_Data& currentData,
        const std::optional<std::tuple<Vector3d, Vector3d, Vector3d>>& headingDest = std::nullopt,
        const std::optional<std::vector<bool>>& controlType = std::nullopt,
        bool log_data = true,
        const std::optional<HOMING_STAGE>& homingStage = std::nullopt);
    
    Quaternion rotate2cameraDirection(const Quaternion& quat_ned_desbodyfrd,
                                      const Quaternion& quat_bodyfrd_cam,
                                      const Vector3d& los_ned_dir,
                                      bool plotFig = false);
    
    void log_control_data(const Vector3d& command, const Vector3d& rpy_rate_cmd,
                          const Quaternion& quat_ned_desbodyfrd_cmd, const Vector3d& Omega_desired_frd,
                          const Vector3d& current_pos_ned, const Vector3d& cur_vel_ned,
                          const Vector3d& gyro_ned, const Vector3d& accel_ned,
                          const Quaternion& quat_ned_bodyfrd, int64_t imu_ts, double dt,
                          int64_t current_ts, int counter,
                          const Vector3d& est_tar_pos_ned = Vector3d::Zero(),
                          const Vector3d& vel_des_ned = Vector3d::Zero(),
                          const Eigen::VectorXd& obs = Eigen::VectorXd(),
                          int modeID = 0, double timestamp = 0.0);
    
    void* getControlNode() const { return controlnode; }
};

#endif // CONTROL_H

