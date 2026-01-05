#include "utils/GeneralFuncs.h"
#include "general.h"
#include "utils/TimeUtils.h"
#include <cmath>
#include <algorithm>

Matrix3d rotX(double angle) {
    Matrix3d R;
    R << 1, 0, 0,
         0, std::cos(angle), -std::sin(angle),
         0, std::sin(angle), std::cos(angle);
    return R;
}

Matrix3d rotY(double angle) {
    Matrix3d R;
    R << std::cos(angle), 0, std::sin(angle),
         0, 1, 0,
         -std::sin(angle), 0, std::cos(angle);
    return R;
}

Matrix3d rotZ(double angle) {
    Matrix3d R;
    R << std::cos(angle), -std::sin(angle), 0,
         std::sin(angle), std::cos(angle), 0,
         0, 0, 1;
    return R;
}

Vector3d rpyRate2omega_frd(const Vector3d& rpyVec, const Vector3d& rpydot) {
    Matrix3d convMat_ned_omega_rpy;
    convMat_ned_omega_rpy << 1, 0, -std::sin(rpyVec[1]),
                             0, std::cos(rpyVec[0]), std::cos(rpyVec[1]) * std::sin(rpyVec[0]),
                             0, -std::sin(rpyVec[0]), std::cos(rpyVec[1]) * std::cos(rpyVec[0]);
    
    return convMat_ned_omega_rpy * rpydot;
}

Vector3d omega_frd2rpyRate(const Vector3d& rpyVec, const Vector3d& omega_frd) {
    Matrix3d convMat_ned_rpy_omega;
    double tan_pitch = std::tan(rpyVec[1]);
    double cos_pitch = std::cos(rpyVec[1]);
    double sin_roll = std::sin(rpyVec[0]);
    double cos_roll = std::cos(rpyVec[0]);
    
    convMat_ned_rpy_omega << 1, sin_roll * tan_pitch, cos_roll * tan_pitch,
                             0, cos_roll, -sin_roll,
                             0, sin_roll / cos_pitch, cos_roll / cos_pitch;
    
    return convMat_ned_rpy_omega * omega_frd;
}

Vector3d limitInclination(double maxAngle, const Vector3d& thrustVector) {
    Vector3d thrustDir = thrustVector / thrustVector.norm();
    double angle = std::acos(thrustDir[2]);
    
    if (angle > maxAngle) {
        thrustDir[2] = std::sqrt(thrustDir[0]*thrustDir[0] + thrustDir[1]*thrustDir[1]) / std::tan(maxAngle);
        thrustDir = thrustDir / thrustDir.norm();
        double newThrust = thrustDir.dot(thrustVector);
        return thrustDir * newThrust;
    }
    
    return thrustVector;
}

LissajousResult lissajous_func(double t, double A, double B, double C, 
                                double a, double b, double c, double alt, double w) {
    double d = M_PI / 2.0 * 0.0;
    
    LissajousResult result;
    result.x = Vector3d(
        A * std::sin(a * t + d),
        B * std::sin(b * t),
        alt + C * std::cos(c * t)
    );
    result.x_dot = Vector3d(
        A * a * std::cos(a * t + d),
        B * b * std::cos(b * t),
        C * c * -std::sin(c * t)
    );
    result.x_2dot = Vector3d(
        A * a * a * -std::sin(a * t + d),
        B * b * b * -std::sin(b * t),
        C * c * c * -std::cos(c * t)
    );
    result.x_3dot = Vector3d(
        A * a * a * a * -std::cos(a * t + d),
        B * b * b * b * -std::cos(b * t),
        C * c * c * c * std::sin(c * t)
    );
    result.x_4dot = Vector3d(
        A * a * a * a * a * std::sin(a * t + d),
        B * b * b * b * b * std::sin(b * t),
        C * c * c * c * c * std::cos(c * t)
    );
    
    result.b1 = Vector3d(std::cos(w * t), std::sin(w * t), 0);
    result.b1_dot = w * Vector3d(-std::sin(w * t), std::cos(w * t), 0);
    result.b1_2dot = w * w * Vector3d(-std::cos(w * t), -std::sin(w * t), 0);
    
    return result;
}

Vector3d* ray_plane_intersection(const Vector3d& ray_origin, 
                                         const Vector3d& ray_direction, 
                                         const Eigen::Vector4d& plane_coeffs) {
    Vector3d plane_normal = plane_coeffs.head<3>();
    double denominator = plane_normal.dot(ray_direction);
    
    if (std::abs(denominator) < 1e-6) {
        return nullptr;  // Ray is parallel to the plane
    }
    if (denominator > 0) {
        return nullptr;  // Ray is in the same direction with plane normal
    }
    
    double t = -(ray_origin.dot(plane_normal) + plane_coeffs[3]) / denominator;
    Vector3d* intersection = new Vector3d(ray_origin + t * ray_direction);
    return intersection;
}

Vector3d unitVec(const Vector3d& vec) {
    double norm = vec.norm();
    if (norm == 0) {
        return vec;
    }
    return vec / norm;
}

// Math utilities from MathUtils
double MaxNormIntegrate(const Eigen::VectorXd& y0, const Eigen::VectorXd& y1, double epsilon) {
    Eigen::VectorXd diff = y0 - y1;
    double abs_err = diff.norm();
    double normy0 = y0.norm();
    double normy1 = y1.norm();
    double rel_err = abs_err / (normy0 + normy1 + epsilon);
    return std::min(rel_err, abs_err) / epsilon;
}

double MaxNormODE(double y0, double y1, double epsilon) {
    double err = std::abs(y0 - y1) / ((std::abs(y0) + std::abs(y1) + epsilon) * epsilon);
    return err;
}

double MaxNormODEp1(double y0, double y1, double epsilon) {
    double err = std::abs(y0 - y1) / ((std::abs(y0) + std::abs(y1) + 1.0) * epsilon);
    return err;
}

FsolveResult myfsolve(std::function<double(double)> func, double bottomBound, double topBound, 
                      double val, double valBottom, double valTop, double rel_tol) {
    FsolveResult result;
    result.err = 0;
    
    double v0 = bottomBound;
    bool valBottomProvided = !std::isnan(valBottom);
    if (!valBottomProvided) {
        valBottom = func(v0);
    }
    valBottom -= val;
    
    double v1 = topBound;
    bool valTopProvided = !std::isnan(valTop);
    if (!valTopProvided) {
        valTop = func(v1);
    }
    valTop -= val;
    
    if (valBottom * valTop > 0 && valBottom < valTop) {
        v0 = topBound;
        valBottom = func(v0) - val;
        v1 = bottomBound;
        valTop = func(v1) - val;
    }
    
    // Binary search
    double tol = std::log10(rel_tol);
    double vt = 0.0;
    double valc = 0.0;
    
    while (std::log10(std::abs((v0 - v1) / v1)) > tol) {
        if (valBottom * valTop < 0) {
            vt = (v0 + v1) / 2.0;
            valc = func(vt);
            if ((valc - val) * valBottom > 0) {
                valBottom = valc - val;
                v0 = vt;
            } else {
                valTop = valc - val;
                v1 = vt;
            }
        } else {
            vt = v1 - valTop * (v1 - v0) / (valTop - valBottom);
            v0 = v1;
            valBottom = valTop;
            v1 = vt;
            valTop = func(v1) - val;
            valc = valTop;
        }
    }
    
    double xerror = std::log10(std::abs((valc - val) / valc));
    double ferror = std::log10(std::abs((v0 - v1) / v1));
    if (std::abs(val) > 0 && std::log10(std::abs((valc - val) / valc)) > -2) {
        result.err = 1;
    }
    result.res = vt;
    result.errors = Eigen::Vector2d(xerror, ferror);
    return result;
}

RK4Result advanceRK4(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, double, const Eigen::VectorXd&)> func,
                     const Eigen::Vector2d& tspan, const Eigen::VectorXd& f0, double dt_prev,
                     const Eigen::VectorXd& f0_prev, int numOfSteps) {
    RK4Result result;
    
    // Create time vector
    double h = (tspan[1] - tspan[0]) / numOfSteps;
    Eigen::VectorXd y = f0;
    
    // Default f0_prev if empty
    Eigen::VectorXd f0_prev_use = f0_prev;
    if (f0_prev_use.size() == 0) {
        f0_prev_use = Eigen::VectorXd::Zero(f0.size());
    }
    
    Eigen::VectorXd k_1 = func(tspan[0], y, dt_prev, f0_prev_use);
    Eigen::VectorXd k_2 = func(tspan[0] + 0.5 * h, y + 0.5 * h * k_1, 0.5 * h, y);
    Eigen::VectorXd k_3 = func(tspan[0] + 0.5 * h, y + 0.5 * h * k_2, 0.5 * h, y);
    Eigen::VectorXd k_4 = func(tspan[0] + h, y + h * k_3, h, y);
    
    Eigen::VectorXd dy = (1.0 / 6.0) * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4) * h;
    y = y + dy;
    
    result.tim = tspan[1];
    result.y = y;
    result.dy = dy;
    
    // Check for NaN
    result.hasNaN = false;
    for (int i = 0; i < y.size(); i++) {
        if (std::isnan(y[i]) || std::isinf(y[i])) {
            result.hasNaN = true;
            break;
        }
    }
    
    return result;
}

IntegrateResult Integrate(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&)> integrantFunc,
                          const Eigen::VectorXd& f0, double t0, double t1,
                          double dt, int nSteps,
                          std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> eventFunc,
                          double eventTol, int maxCount, bool initOnly, bool runTillEvent,
                          std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> MaxNorm,
                          double maxNormTol) {
    IntegrateResult result;
    result.finishedByEvent = false;
    double maxNorm = 2.0;
    
    std::vector<Eigen::VectorXd> stateList;
    std::vector<Eigen::VectorXd> primeList;
    std::vector<double> argList;
    
    stateList.push_back(f0);
    
    Eigen::VectorXd dt_vec(1);
    dt_vec[0] = 0.0;
    Eigen::VectorXd yprime = integrantFunc(t0, f0, dt_vec, f0);
    primeList.push_back(yprime);
    argList.push_back(t0);
    
    if (std::isnan(dt)) {
        dt = (t1 - t0) / nSteps;
    }
    
    double dir = (dt > 0) ? 1.0 : -1.0;
    Eigen::VectorXd ycur = f0;
    double curt = t0;
    double curdt = dt;
    Eigen::VectorXd err0(1);
    err0[0] = 1.0;
    
    if (eventFunc) {
        err0 = eventFunc(curt, ycur, yprime);
        bool allBelowTol = true;
        for (int i = 0; i < err0.size(); i++) {
            if (err0[i] >= eventTol) {
                allBelowTol = false;
                break;
            }
        }
        if (allBelowTol) {
            result.stateList = stateList;
            result.argList = argList;
            result.primeList = primeList;
            result.maxNorm = maxNorm;
            result.finishedByEvent = false;
            return result;
        }
    }
    
    int count = 0;
    int factorExp = 0;
    
    // Use default MaxNorm if not provided
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> MaxNormFunc = MaxNorm;
    if (!MaxNormFunc) {
        MaxNormFunc = [](const Eigen::VectorXd& y0, const Eigen::VectorXd& y1, double eps) {
            return MaxNormIntegrate(y0, y1, eps);
        };
    }
    
    while ((dir * (t1 - curt)) > 0 || runTillEvent) {
        double newdt = curdt;
        maxNorm = 29.0;
        int nsteps = 1;
        Eigen::VectorXd f0_prev = stateList.back();
        double dt_prev = curdt;
        if (argList.size() > 1) {
            dt_prev = argList.back() - argList[argList.size() - 2];
        }
        
        Eigen::VectorXd delta2Halfs = Eigen::VectorXd::Zero(ycur.size());
        Eigen::VectorXd deltaFull = Eigen::VectorXd::Zero(ycur.size());
        
        while (maxNorm > 1.0 && nsteps < 512) {
            curdt = newdt;
            
            Eigen::Vector2d tspan1(curt, curt + curdt);
            // Wrap integrantFunc to match advanceRK4 signature: (t, y, dt, f0_prev) -> (t, y, dt_vec, f0_prev)
            auto rk4_func = [&integrantFunc](double t, const Eigen::VectorXd& y, double dt, const Eigen::VectorXd& f0_prev) -> Eigen::VectorXd {
                Eigen::VectorXd dt_vec(1);
                dt_vec[0] = dt;
                return integrantFunc(t, y, dt_vec, f0_prev);
            };
            RK4Result rk4_1 = advanceRK4(rk4_func, tspan1, ycur, dt_prev, f0_prev);
            Eigen::VectorXd ytplusFulldt = rk4_1.y;
            deltaFull = rk4_1.dy;
            
            Eigen::Vector2d tspan2(curt, curt + curdt / 2.0);
            RK4Result rk4_2 = advanceRK4(rk4_func, tspan2, ycur, dt_prev, f0_prev);
            Eigen::VectorXd ytplusHalfdt = rk4_2.y;
            Eigen::VectorXd deltaHalf1 = rk4_2.dy;
            
            Eigen::Vector2d tspan3(curt + curdt / 2.0, curt + curdt);
            RK4Result rk4_3 = advanceRK4(rk4_func, tspan3, ytplusHalfdt, curdt, ycur);
            Eigen::VectorXd ytplus2Halfsdt = rk4_3.y;
            Eigen::VectorXd deltaHalf2 = rk4_3.dy;
            
            delta2Halfs = deltaHalf1 + deltaHalf2;
            maxNorm = MaxNormFunc(ytplusFulldt, ytplus2Halfsdt, maxNormTol);
            newdt = curdt / 2.0;
            nsteps = static_cast<int>(std::max((t1 - curt) / newdt, 1.0));
            
            if (count > maxCount) {
                break;
            }
            count++;
            if (maxNorm > 1.0) {
                factorExp = 0;
            }
            f0_prev = ycur;
            dt_prev = curdt;
        }
        
        // Richardson Interpolation
        Eigen::VectorXd delta = delta2Halfs + (1.0 / 15.0) * (delta2Halfs - deltaFull);
        Eigen::VectorXd nexty = ycur + delta;
        double nextt = curt + curdt;
        
        Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(ycur.size());
        yprime = integrantFunc(nextt, nexty, Eigen::VectorXd::Constant(1, curdt), zero_vec);  // dt as VectorXd(1) to match signature
        
        if (eventFunc) {
            Eigen::VectorXd err = eventFunc(nextt, nexty, yprime);
            bool allNegative = true;
            for (int i = 0; i < err.size(); i++) {
                if (err[i] >= 0) {
                    allNegative = false;
                    break;
                }
            }
            
            if (allNegative) {
                double absErr = err.cwiseAbs().maxCoeff();
                if (absErr < (eventTol * err0.cwiseAbs().maxCoeff()) || absErr < eventTol) {
                    result.finishedByEvent = true;
                    curt = nextt;
                    ycur = nexty;
                    break;
                } else {
                    Eigen::VectorXd yprimecur = integrantFunc(curt, ycur, Eigen::VectorXd::Constant(1, 0.0), zero_vec);
                    Eigen::VectorXd errcur = eventFunc(curt, ycur, yprimecur);
                    double errVal = err.cwiseAbs().maxCoeff();
                    double errcurVal = errcur.cwiseAbs().maxCoeff();
                    double new_dt = -curdt * errcurVal / (errVal - errcurVal);
                    curdt = new_dt;
                    continue;
                }
            }
        }
        
        curt = nextt;
        ycur = nexty;
        curdt = curdt * std::pow(std::sqrt(2.0), factorExp);
        factorExp = 1;
        
        argList.push_back(curt);
        stateList.push_back(ycur);
        
        if (initOnly) {
            break;
        }
        primeList.push_back(yprime);
    }
    
    argList.push_back(curt);
    stateList.push_back(ycur);
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(ycur.size());
    yprime = integrantFunc(curt, ycur, Eigen::VectorXd::Constant(1, 0.0), zero_vec);
    primeList.push_back(yprime);
    
    result.stateList = stateList;
    result.argList = argList;
    result.primeList = primeList;
    result.maxNorm = maxNorm;
    
    return result;
}

// Trajectory generation functions (from trajectories.py)

TrajectoryResult horz_circle(const Vector3d& center, double radius,
                             const Vector3d& missionAttitudeDirection,
                             double Vel, double start_time) {
    double t;
    if (start_time < 0) {
        t = TimeUtils::now();
    } else {
        t = TimeUtils::now() - start_time;
    }
    
    double A = radius;
    double B = radius;
    double C = 0.0;
    
    double R = (A + B) / 2.0;
    double L = 2.0 * M_PI * R;
    double w = Vel / L;
    
    double d = M_PI / 2.0 * 0.0;
    double a = 2.0 * M_PI * w;
    double b = 2.0 * M_PI * w;
    double c = 0.0 * 2.0 * M_PI * w * 2.0;
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = Vector3d(
        A * std::sin(a * t + d),
        B * std::cos(b * t),
        C * std::cos(c * t)
    ) + center;
    result.x[1] = Vector3d(
        A * a * std::cos(a * t + d),
        B * b * -std::sin(b * t),
        C * c * -std::sin(c * t)
    );
    result.x[2] = Vector3d(
        A * a * a * -std::sin(a * t + d),
        B * b * b * -std::cos(b * t),
        C * c * c * -std::cos(c * t)
    );
    result.x[3] = Vector3d(
        A * a * a * a * -std::cos(a * t + d),
        B * b * b * b * std::sin(b * t),
        C * c * c * c * std::sin(c * t)
    );
    result.x[4] = Vector3d(
        A * a * a * a * a * std::sin(a * t + d),
        B * b * b * b * b * std::cos(b * t),
        C * c * c * c * c * std::cos(c * t)
    );
    
    result.b1.resize(3);
    // If missionAttitudeDirection is zero (equivalent to None in Python), use rotating b1
    if (missionAttitudeDirection.norm() < 1e-6) {
        result.b1[0] = Vector3d(std::cos(w * t), std::sin(w * t), 0);
    } else {
        result.b1[0] = missionAttitudeDirection;
    }
    result.b1[1] = Vector3d::Zero();
    result.b1[2] = Vector3d::Zero();
    
    result.pos_control = {true, true, true};
    result.vel_control = {true, true, false};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    
    return result;
}

TrajectoryResult pos_point(const std::vector<Vector3d>& missionPoint,
                          const Vector3d& missionAttitudeDirection,
                          double start_time) {
    double t;
    if (start_time < 0) {
        t = TimeUtils::now();
    } else {
        t = TimeUtils::now() - start_time;
    }
    
    Vector3d x = missionPoint[0];
    if (missionPoint.size() > 1) {
        double T = 10.0;
        double w = 2.0 * M_PI / T;
        double state = std::sin(w * t);
        x = missionPoint[0] + (state > 0 ? 1.0 : -1.0) * (missionPoint[1] - missionPoint[0]);
    }
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = x;
    result.x[1] = Vector3d::Zero();
    result.x[2] = Vector3d::Zero();
    result.x[3] = Vector3d::Zero();
    result.x[4] = Vector3d::Zero();
    
    result.b1.resize(3);
    result.b1[0] = missionAttitudeDirection;
    result.b1[1] = Vector3d::Zero();
    result.b1[2] = Vector3d::Zero();
    
    result.pos_control = {true, true, true};
    result.vel_control = {false, false, false};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    
    return result;
}

TrajectoryResult vel_point(const Vector3d& missionVelocity,
                          const Vector3d& missionAttitudeDirection,
                          double start_time) {
    double t;
    if (start_time < 0) {
        t = TimeUtils::now();
    } else {
        t = TimeUtils::now() - start_time;
    }
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = Vector3d::Zero();
    result.x[1] = missionVelocity;
    result.x[2] = Vector3d::Zero();
    result.x[3] = Vector3d::Zero();
    result.x[4] = Vector3d::Zero();
    
    result.b1.resize(3);
    result.b1[0] = missionAttitudeDirection;
    result.b1[1] = Vector3d::Zero();
    result.b1[2] = Vector3d::Zero();
    
    result.pos_control = {false, false, false};
    result.vel_control = {true, true, true};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    
    return result;
}

TrajectoryResult lineConstVel(const Vector3d& startPoint,
                             const Vector3d& endPoint,
                             double speed,
                             double startTime,
                             const Vector3d& missionAttitudeDirection) {
    double t = TimeUtils::now() - startTime;
    
    Vector3d deltaDistance = endPoint - startPoint;
    Vector3d deltaDir = unitVec(deltaDistance);
    double deltaDistanceNorm = deltaDistance.norm();
    Vector3d velocity = deltaDir * speed;
    
    Vector3d x = startPoint + t * velocity;
    Vector3d x_dot = velocity;
    bool finished = false;
    
    if (t * speed > deltaDistanceNorm) {
        x = endPoint;
        x_dot = Vector3d::Zero();
        finished = true;
    }
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = x;
    result.x[1] = x_dot;
    result.x[2] = Vector3d::Zero();
    result.x[3] = Vector3d::Zero();
    result.x[4] = Vector3d::Zero();
    
    result.b1.resize(3);
    result.b1[0] = missionAttitudeDirection;
    result.b1[1] = Vector3d::Zero();
    result.b1[2] = Vector3d::Zero();
    
    result.pos_control = {true, true, true};
    bool in_transit = (t * speed < deltaDistanceNorm);
    result.vel_control = {in_transit, in_transit, in_transit};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    result.finished = finished;
    
    return result;
}

TrajectoryResult vert_circle(double start_time) {
    double t;
    if (start_time < 0) {
        t = TimeUtils::now();
    } else {
        t = TimeUtils::now() - start_time;
    }
    
    double Vel = 3.0;
    double A = 5.0;
    double B = 5.0;
    double C = 0.0;
    
    double R = (A + B) / 2.0;
    double L = 2.0 * M_PI * R;
    double w = Vel / L;
    
    double d = M_PI / 2.0 * 0.0;
    double a = 2.0 * M_PI * w;
    double b = 2.0 * M_PI * w;
    double c = 2.0 * M_PI * w * 2.0;
    double alt = -20.0;
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = Vector3d(
        C * std::cos(c * t),
        A * std::sin(a * t + d),
        B * std::cos(b * t)
    );
    result.x[0][2] = alt + C * std::cos(c * t);  // Overwrite z component
    result.x[1] = Vector3d(
        C * c * -std::sin(c * t),
        A * a * std::cos(a * t + d),
        B * b * -std::sin(b * t)
    );
    result.x[2] = Vector3d(
        C * c * c * -std::cos(c * t),
        A * a * a * -std::sin(a * t + d),
        B * b * b * -std::cos(b * t)
    );
    result.x[3] = Vector3d(
        C * c * c * c * std::sin(c * t),
        A * a * a * a * -std::cos(a * t + d),
        B * b * b * b * std::sin(b * t)
    );
    result.x[4] = Vector3d(
        C * c * c * c * c * std::cos(c * t),
        A * a * a * a * a * std::sin(a * t + d),
        B * b * b * b * b * std::cos(b * t)
    );
    
    result.b1.resize(3);
    result.b1[0] = Vector3d(1, 0, 0);
    result.b1[1] = Vector3d::Zero();
    result.b1[2] = Vector3d::Zero();
    
    result.pos_control = {false, false, false};
    result.vel_control = {true, true, true};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    
    return result;
}

TrajectoryResult command_Lissajous(double t, double start_time) {
    if (t < 0) {
        if (start_time < 0) {
            t = TimeUtils::now();
        } else {
            t = TimeUtils::now() - start_time;
        }
    }
    
    double A = 15.0;   // X amplitude
    double B = 15.0;   // Y amplitude
    double C = 5.0;    // Z amplitude
    double alt = -1.0; // Z offset
    
    // double d = M_PI / 2.0 * 0.0;  // Unused variable
    double a = 0.2 * 1.5;  // X frequency
    double b = 0.3 * 1.5;  // Y frequency
    double c = 0.2;        // Z frequency
    double w = 2.0 * M_PI / 10.0;  // attitude rotation frequency
    
    LissajousResult liss = lissajous_func(t, A, B, C, a, b, c, alt, w);
    
    TrajectoryResult result;
    result.x.resize(5);
    result.x[0] = liss.x;
    result.x[1] = liss.x_dot;
    result.x[2] = liss.x_2dot;
    result.x[3] = liss.x_3dot;
    result.x[4] = liss.x_4dot;
    
    result.b1.resize(3);
    result.b1[0] = liss.b1;
    result.b1[1] = liss.b1_dot;
    result.b1[2] = liss.b1_2dot;
    
    result.pos_control = {false, false, false};
    result.vel_control = {true, true, true};
    result.yaw_control = YAW_COMMAND::DEFINED_DIR;
    
    return result;
}

