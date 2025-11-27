#!/home/$USER/anaconda3/envs/py11/bin/python3
import os
import pandas as pd
import matplotlib.pyplot as plt
import sys
import webbrowser
import numpy as np
from common import *
# from Filter.AllInOneKalman import AllInOneKalman
import pymap3d

# Read input and control log CSV files
inputData = pd.read_csv('/home/valentin/RL/logs/20251113_092619_system_manager.csv')
controlLogData = pd.read_csv('/home/valentin/RL/logs/20251113_092619_control_logs_VelocityRL.csv')

# Reader lines for input_logger.log data (lines 22-40)
# self._input_logger.log({"comp_time":time.monotonic(), "imu_ts":self._currentData.imu_ned.timestamp,
#                         "accl_ned/x": self._currentData.imu_ned.accel[0],"accl_ned/y": self._currentData.imu_ned.accel[1], "accl_ned/z": self._currentData.imu_ned.accel[2],
#                         "gyro_ned/x": self._currentData.imu_ned.gyro[0], "gyro_ned/y": self._currentData.imu_ned.gyro[1], "gyro_ned/z": self._currentData.imu_ned.gyro[2],
#                         "pos_ned_ts": self._currentData.pos_ned_m.timestamp,
#                         "pos_ned/x": self._currentData.pos_ned_m.ned[0], "pos_ned/y": self._currentData.pos_ned_m.ned[1], "pos_ned/z": self._currentData.pos_ned_m.ned[2],
#                         "vel_ned/x": self._currentData.pos_ned_m.vel_ned[0], "vel_ned/y": self._currentData.pos_ned_m.vel_ned[1], "vel_ned/z": self._currentData.pos_ned_m.vel_ned[2],
#                         "quat_ned_bodyfrd_ts": self._currentData.quat_ned_bodyfrd.timestamp,
#                         "quat_ned_bodyfrd/x": self._currentData.quat_ned_bodyfrd.x, "quat_ned_bodyfrd/y": self._currentData.quat_ned_bodyfrd.y,
#                         "quat_ned_bodyfrd/z": self._currentData.quat_ned_bodyfrd.z, "quat_ned_bodyfrd/w": self._currentData.quat_ned_bodyfrd.w,
#                         "lla_ts": self._currentData.raw_pos_lla_deg.timestamp,
#                         "lla/lat_deg": self._currentData.raw_pos_lla_deg.lla[0], "lla/lon_deg": self._currentData.raw_pos_lla_deg.lla[1], "lla/alt": self._currentData.raw_pos_lla_deg.lla[2],
#                         "current_ts":current_ts, "step_dt":step_dt, "imu_ts":imu_ts,
#                         "current_alt": self._currentData.relative_m, "command": command, 
#                         "rpy_rate_cmd/x": rpy_rate_cmd[0], "rpy_rate_cmd/y": rpy_rate_cmd[1], "rpy_rate_cmd/z": rpy_rate_cmd[2], 
#                         "quat_ned_desbodyfrd_cmd/x": quat_ned_desbodyfrd_cmd.x, "quat_ned_desbodyfrd_cmd/y": quat_ned_desbodyfrd_cmd.y,
#                         "quat_ned_desbodyfrd_cmd/z": quat_ned_desbodyfrd_cmd.z, "quat_ned_desbodyfrd_cmd/w": quat_ned_desbodyfrd_cmd.w,
#                         "destination_ned/x": destination_ned[0], "destination_ned/y": destination_ned[1], "destination_ned/z": destination_ned[2],
#                         "counter": counter, "current_mode": current_mode, "timestamp": self._currentData.timestamp, "local_ts":self._currentData.local_ts,
#                         "modeID":self._currentData.custom_mode_id})

comp_time_input = np.array(inputData['comp_time'])
imu_ts = np.array(inputData['imu_ts'])
accl_ned_x = inputData['accl_ned/x']; accl_ned_y = inputData['accl_ned/y']; accl_ned_z = inputData['accl_ned/z']
gyro_ned_x = inputData['gyro_ned/x']; gyro_ned_y = inputData['gyro_ned/y']; gyro_ned_z = inputData['gyro_ned/z']
pos_ned_ts = np.array(inputData['pos_ned_ts'])
pos_ned_x = inputData['pos_ned/x']; pos_ned_y = inputData['pos_ned/y']; pos_ned_z = inputData['pos_ned/z']
vel_ned_x = inputData['vel_ned/x']; vel_ned_y = inputData['vel_ned/y']; vel_ned_z = inputData['vel_ned/z']
quat_ned_bodyfrd_ts = np.array(inputData['quat_ned_bodyfrd_ts'])
quat_ned_bodyfrd_x = inputData['quat_ned_bodyfrd/x']; quat_ned_bodyfrd_y = inputData['quat_ned_bodyfrd/y']; quat_ned_bodyfrd_z = inputData['quat_ned_bodyfrd/z']; quat_ned_bodyfrd_w = inputData['quat_ned_bodyfrd/w']
lla_ts = np.array(inputData['lla_ts'])
lla_lat = inputData['lla/lat_deg']; lla_lon = inputData['lla/lon_deg']; lla_alt = inputData['lla/alt']
current_ts = np.array(inputData['current_ts']); step_dt = inputData['step_dt']
current_alt = inputData['current_alt']
command = inputData['command']
rpy_rate_cmd_x = inputData['rpy_rate_cmd/x']; rpy_rate_cmd_y = inputData['rpy_rate_cmd/y']; rpy_rate_cmd_z = inputData['rpy_rate_cmd/z']
quat_ned_desbodyfrd_cmd_x = inputData['quat_ned_desbodyfrd_cmd/x']; quat_ned_desbodyfrd_cmd_y = inputData['quat_ned_desbodyfrd_cmd/y']; quat_ned_desbodyfrd_cmd_z = inputData['quat_ned_desbodyfrd_cmd/z']; quat_ned_desbodyfrd_cmd_w = inputData['quat_ned_desbodyfrd_cmd/w']
destination_ned_x = inputData['destination_ned/x']; destination_ned_y = inputData['destination_ned/y']; destination_ned_z = inputData['destination_ned/z']
counter = inputData['counter']
current_mode = inputData['current_mode']
timestamp = np.array(inputData['timestamp'])
local_ts = inputData['local_ts']
modeID = np.array(inputData['modeID'])

# Reader lines for control_logger.log data (lines 62-76)
# self._control_logger.log({"comp_time":time.monotonic(), "command/[0]":command[0], "command/[1]":command[1], "command/[2]":command[2], 
#                           "rate_cmd/roll":rpy_rate_cmd[0], "rate_cmd/pitch":rpy_rate_cmd[1], "rate_cmd/yaw":rpy_rate_cmd[2],
#                           "quat_ned_desbodyfrd_cmd/x":quat_ned_desbodyfrd_cmd.x, "quat_ned_desbodyfrd_cmd/y":quat_ned_desbodyfrd_cmd.y,
#                           "quat_ned_desbodyfrd_cmd/z":quat_ned_desbodyfrd_cmd.z, "quat_ned_desbodyfrd_cmd/w":quat_ned_desbodyfrd_cmd.w,
#                           "current_pos_ned/x":current_pos_ned[0], "current_pos_ned/y":current_pos_ned[1], "current_pos_ned/z":current_pos_ned[2],
#                           "cur_vel_ned/x":cur_vel_ned[0], "cur_vel_ned/y":cur_vel_ned[1], "cur_vel_ned/z":cur_vel_ned[2],
#                           "gyro_ned/x":gyro_ned[0], "gyro_ned/y":gyro_ned[1], "gyro_ned/z":gyro_ned[2],
#                           "accel_ned/x":accel_ned[0], "accel_ned/y":accel_ned[1], "accel_ned/z":accel_ned[2],
#                           "Omega_desired_frd/x":Omega_desired_frd[0], "Omega_desired_frd/y":Omega_desired_frd[1], "Omega_desired_frd/z":Omega_desired_frd[2],
#                           "quat_ned_bodyfrd/x":quat_ned_bodyfrd.x, "quat_ned_bodyfrd/y":quat_ned_bodyfrd.y,
#                           "quat_ned_bodyfrd/z":quat_ned_bodyfrd.z, "quat_ned_bodyfrd/w":quat_ned_bodyfrd.w,
#                           "est_tar_pos_ned/x":est_tar_pos_ned[0], "est_tar_pos_ned/y":est_tar_pos_ned[1], "est_tar_pos_ned/z":est_tar_pos_ned[2],
#                           "vel_des_ned/x":vel_des_ned[0], "vel_des_ned/y":vel_des_ned[1], "vel_des_ned/z":vel_des_ned[2],
#                           "imu_ts":imu_ts, "dt":dt, "current_ts":current_ts, "counter":counter, "modeID":modeID, "timestamp":timestamp,
#                           "obs/[0]":obs[0], "obs/[1]":obs[1], "obs/[2]":obs[2]})

comp_time = np.array(controlLogData['comp_time'])
command_0 = controlLogData['command/[0]']; command_1 = controlLogData['command/[1]']; command_2 = controlLogData['command/[2]']
roll_rate_cmd = controlLogData['rate_cmd/roll']; pitch_rate_cmd = controlLogData['rate_cmd/pitch']; yaw_rate_cmd = controlLogData['rate_cmd/yaw']
quat_ned_desbodyfrd_cmd_x = controlLogData['quat_ned_desbodyfrd_cmd/x']; quat_ned_desbodyfrd_cmd_y = controlLogData['quat_ned_desbodyfrd_cmd/y']; quat_ned_desbodyfrd_cmd_z = controlLogData['quat_ned_desbodyfrd_cmd/z']; quat_ned_desbodyfrd_cmd_w = controlLogData['quat_ned_desbodyfrd_cmd/w']
current_pos_ned_x = controlLogData['current_pos_ned/x']; current_pos_ned_y = controlLogData['current_pos_ned/y']; current_pos_ned_z = controlLogData['current_pos_ned/z']
cur_vel_ned_x = controlLogData['cur_vel_ned/x']; cur_vel_ned_y = controlLogData['cur_vel_ned/y']; cur_vel_ned_z = controlLogData['cur_vel_ned/z']
gyro_ned_x = controlLogData['gyro_ned/x']; gyro_ned_y = controlLogData['gyro_ned/y']; gyro_ned_z = controlLogData['gyro_ned/z']
accel_ned_x = controlLogData['accel_ned/x']; accel_ned_y = controlLogData['accel_ned/y']; accel_ned_z = controlLogData['accel_ned/z']
Omega_desired_frd_x = controlLogData['Omega_desired_frd/x']; Omega_desired_frd_y = controlLogData['Omega_desired_frd/y']; Omega_desired_frd_z = controlLogData['Omega_desired_frd/z']
quat_ned_bodyfrd_x = controlLogData['quat_ned_bodyfrd/x']; quat_ned_bodyfrd_y = controlLogData['quat_ned_bodyfrd/y']; quat_ned_bodyfrd_z = controlLogData['quat_ned_bodyfrd/z']; quat_ned_bodyfrd_w = controlLogData['quat_ned_bodyfrd/w']
est_tar_pos_ned_x = controlLogData['est_tar_pos_ned/x']; est_tar_pos_ned_y = controlLogData['est_tar_pos_ned/y']; est_tar_pos_ned_z = controlLogData['est_tar_pos_ned/z']
vel_des_ned_x = controlLogData['vel_des_ned/x']; vel_des_ned_y = controlLogData['vel_des_ned/y']; vel_des_ned_z = controlLogData['vel_des_ned/z']
imu_ts = np.array(controlLogData['imu_ts'])
dt = controlLogData['dt']
current_ts = np.array(controlLogData['current_ts'])
counter = controlLogData['counter']
modeID = np.array(controlLogData['modeID'])
timestamp = np.array(controlLogData['timestamp'])
obs_0 = controlLogData['obs/[0]']; obs_1 = controlLogData['obs/[1]']; obs_2 = controlLogData['obs/[2]']

