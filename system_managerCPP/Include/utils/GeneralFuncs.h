#ifndef GENERAL_FUNCS_H
#define GENERAL_FUNCS_H
#include "general.h"

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <vector>
#include <limits>
#include <algorithm>
#include "YawCommand.h"

// Rotation matrices
Matrix3d rotX(double angle);
Matrix3d rotY(double angle);
Matrix3d rotZ(double angle);

// Rate conversions
Vector3d rpyRate2omega_frd(const Vector3d& rpyVec, const Vector3d& rpydot);
Vector3d omega_frd2rpyRate(const Vector3d& rpyVec, const Vector3d& omega_frd);

// Thrust limiting
Vector3d limitInclination(double maxAngle, const Vector3d& thrustVector);

// Lissajous curve
struct LissajousResult {
    Vector3d x, x_dot, x_2dot, x_3dot, x_4dot;
    Vector3d b1, b1_dot, b1_2dot;
};

LissajousResult lissajous_func(double t, double A = 1.0, double B = 1.0, double C = 0.2, 
                                double a = 2.0, double b = 3.0, double c = 2.0, 
                                double alt = -1.0, double w = 2.0 * M_PI / 10.0);

// Ray-plane intersection (returns nullptr if no intersection)
// Note: Caller is responsible for deleting the returned pointer
Vector3d* ray_plane_intersection(const Vector3d& ray_origin, 
                                        const Vector3d& ray_direction, 
                                        const Eigen::Vector4d& plane_coeffs);

// Vector utilities
Vector3d unitVec(const Vector3d& vec);

// Math utilities from MathUtils
// MaxNorm functions for error estimation
double MaxNormIntegrate(const Eigen::VectorXd& y0, const Eigen::VectorXd& y1, double epsilon);
double MaxNormODE(double y0, double y1, double epsilon);
double MaxNormODEp1(double y0, double y1, double epsilon);

// Binary search solver
struct FsolveResult {
    double res;
    int err;
    Eigen::Vector2d errors;  // [xerror, ferror]
};

FsolveResult myfsolve(std::function<double(double)> func, double bottomBound, double topBound, 
                      double val = 0.0, double valBottom = std::numeric_limits<double>::quiet_NaN(), 
                      double valTop = std::numeric_limits<double>::quiet_NaN(), 
                      double rel_tol = 1e-10);

// Runge-Kutta 4th order integration
struct RK4Result {
    double tim;
    Eigen::VectorXd y;
    Eigen::VectorXd dy;
    bool hasNaN;
};

RK4Result advanceRK4(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, double, const Eigen::VectorXd&)> func,
                     const Eigen::Vector2d& tspan, const Eigen::VectorXd& f0, double dt_prev,
                     const Eigen::VectorXd& f0_prev = Eigen::VectorXd(), int numOfSteps = 1);

// Adaptive integration with event detection
struct IntegrateResult {
    std::vector<Eigen::VectorXd> stateList;
    std::vector<double> argList;
    std::vector<Eigen::VectorXd> primeList;
    double maxNorm;
    bool finishedByEvent;
};

IntegrateResult Integrate(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&)> integrantFunc,
                          const Eigen::VectorXd& f0, double t0, double t1,
                          double dt = std::numeric_limits<double>::quiet_NaN(), int nSteps = 100,
                          std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> eventFunc = nullptr,
                          double eventTol = 1e-6, int maxCount = 3, bool initOnly = false, bool runTillEvent = false,
                          std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> MaxNorm = nullptr,
                          double maxNormTol = 1e-6);

// Trajectory generation functions (from trajectories.py)
struct TrajectoryResult {
    std::vector<Vector3d> x;  // [x, x_dot, x_2dot, x_3dot, x_4dot]
    std::vector<Vector3d> b1; // [b1, b1_dot, b1_2dot]
    std::vector<bool> pos_control;
    std::vector<bool> vel_control;
    YAW_COMMAND yaw_control;
    bool finished = false;  // For lineConstVel
};

// Horizontal circle trajectory
TrajectoryResult horz_circle(const Vector3d& center = Vector3d::Zero(),
                             double radius = 3.0,
                             const Vector3d& missionAttitudeDirection = Vector3d(-1, 0, 0),
                             double Vel = 1.0,
                             double start_time = -1.0);

// Point position trajectory
TrajectoryResult pos_point(const std::vector<Vector3d>& missionPoint,
                          const Vector3d& missionAttitudeDirection = Vector3d(-1, 0, 0),
                          double start_time = -1.0);

// Velocity point trajectory
TrajectoryResult vel_point(const Vector3d& missionVelocity = Vector3d(1, 0, 0),
                          const Vector3d& missionAttitudeDirection = Vector3d(-1, 0, 0),
                          double start_time = -1.0);

// Constant velocity line trajectory
TrajectoryResult lineConstVel(const Vector3d& startPoint,
                             const Vector3d& endPoint,
                             double speed,
                             double startTime,
                             const Vector3d& missionAttitudeDirection = Vector3d(-1, 0, 0));

// Vertical circle trajectory
TrajectoryResult vert_circle(double start_time = -1.0);

// Lissajous curve trajectory
TrajectoryResult command_Lissajous(double t = -1.0, double start_time = -1.0);

#endif // GENERAL_FUNCS_H

