# How to Enable C++ Action Logging

To enable full comparison between C++ and Python inference, you need to log action outputs from the C++ code.

## Steps to Enable Action Logging

### 1. Modify `Control.cpp::log_control_data()`

Add action_logits parameter to the function signature:

```cpp
void Control::log_control_data(const Vector3d& command, const Vector3d& rpy_rate_cmd,
                               const Quaternion& quat_ned_desbodyfrd_cmd, const Vector3d& Omega_desired_frd,
                               const Vector3d& current_pos_ned, const Vector3d& cur_vel_ned,
                               const Vector3d& gyro_ned, const Vector3d& accel_ned,
                               const Quaternion& quat_ned_bodyfrd, int64_t imu_ts, double dt,
                               int64_t current_ts, int counter,
                               const Vector3d& est_tar_pos_ned,
                               const Vector3d& vel_des_ned,
                               const Eigen::VectorXd& obs,
                               const Eigen::VectorXd& action_logits,  // ADD THIS
                               int modeID, double timestamp);
```

### 2. Add Action Logging in `log_control_data()`

After the observation logging (around line 373), add:

```cpp
// Log action outputs (mean and logstd)
if (action_logits.size() > 0) {
    int action_dim = action_logits.size() / 2;
    for (int i = 0; i < action_dim; i++) {
        oss.str("");
        oss << action_logits[i];  // mean
        logDict["cpp_action_mean/" + std::to_string(i)] = oss.str();
        oss.str("");
        oss << action_logits[action_dim + i];  // logstd
        logDict["cpp_action_logstd/" + std::to_string(i)] = oss.str();
    }
}
```

### 3. Update `Control.cpp::get_cmd()` to Pass Action Logits

In the `get_cmd()` function, after getting the command from the controller, extract action_logits:

```cpp
// For RL controller, get action_logits
Eigen::VectorXd action_logits;
if (controllerType == CONTROLLER_TYPE::VELOCITYRL) {
    VelocityRLController* rl_controller = static_cast<VelocityRLController*>(controlnode);
    // You'll need to modify VelocityRLController to return action_logits
    // Or extract them from the controller's internal state
    // For now, create a placeholder
    action_logits = Eigen::VectorXd::Zero(4);  // 2 actions * 2 (mean + logstd)
} else {
    action_logits = Eigen::VectorXd::Zero(0);
}
```

Then pass it to `log_control_data()`:

```cpp
log_control_data(command, rpyRate_cmd, quat_ned_desbodyfrd, Omega_desired_frd,
                _current_pos_ned, _current_vel_ned, gyro_ned, accel_ned,
                quat_ned_bodyfrd, imu_ts, step_dt, current_ts, counter,
                estimated_tar_pos_ned, vel_des_ned, obs, action_logits,  // ADD action_logits
                currentData.custom_mode_id, currentData.timestamp / 1000.0);
```

### 4. Modify `VelocityRLController` to Return Action Logits

In `VelocityRLController::getCommand()`, modify the return to include action_logits:

```cpp
// In VelocityRLController.h, change return type:
std::tuple<Vector3d, Matrix3d, Vector3d, Eigen::VectorXd, Eigen::VectorXd> getCommand(...);

// In VelocityRLController.cpp, return action_logits:
return std::make_tuple(vel_vector, Matrix3d::Identity(), omega_vector, obsTotal, action_logits);
```

Then in `Control.cpp`, extract it:

```cpp
auto result = rl_controller->getCommand(...);
action_logits = std::get<4>(result);  // Get action_logits
```

## Alternative: Quick Test Without Modifying C++

If you just want to test Python inference without modifying C++ code, the script will work and output only Python results. The comparison columns will be empty, but you can still verify that Python inference is working correctly.
