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

inputData = pd.read_csv('/home/valentin/RL/logs/20251113_092619_system_manager.csv')
controlLogData = pd.read_csv('/home/valentin/RL/logs/20251113_092619_control_logs_VelocityRL.csv')

# get column pos_ned_x from input.csv
R_rfu_frd = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0, -1]])
quat_rfu_frd = Quaternion.from_matrix(R_rfu_frd)
quat_enu_ned = Quaternion.from_matrix(R_rfu_frd)

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


inds=lla_ts==0; lla_ts=np.array(lla_ts[~inds]); lla_lat=np.array(lla_lat[~inds]); lla_lon=np.array(lla_lon[~inds]); lla_alt=np.array(lla_alt[~inds])
startTime = np.min([imu_ts[0],quat_ts[0], lla_ts[0]])
lla_ts = lla_ts - startTime
imu_ts = imu_ts - startTime
pos_ned_ts = pos_ned_ts - startTime
quat_ts = quat_ts - startTime
current_ts = current_ts - startTime
if 0:    # simulate kalman data
    iekf_pos = np.zeros((len(lla_ts), 4)); tfiekf_pos = np.zeros((len(lla_ts), 4)); se3_R3_EqF_pos = np.zeros((len(lla_ts), 4)); se23_se3_EqF_pos = np.zeros((len(lla_ts), 4)); se23_se23_EqF_pos = np.zeros((len(lla_ts), 4)); mekf_pos = np.zeros((len(lla_ts), 4)); liekf_pos = np.zeros((len(lla_ts), 4))
    iekf_R = np.zeros((len(lla_ts),3,3)); tfiekf_R = np.zeros((len(lla_ts),3,3)); se3_R3_EqF_R = np.zeros((len(lla_ts),3,3)); se23_se3_EqF_R = np.zeros((len(lla_ts),3,3)); se23_se23_EqF_R = np.zeros((len(lla_ts),3,3)); mekf_R = np.zeros((len(lla_ts),3,3)); liekf_R = np.zeros((len(lla_ts),3,3))
    timeline = np.vstack((imu_ts, pos_ned_ts, quat_ts, current_ts))
    allInOneKalman = AllInOneKalman()
    cur_imu = None; cur_pos = None; cur_quat_ned_bodyfrd = None; cur_lla = None
    end_time = np.min([imu_ts[-1], pos_ned_ts[-1], quat_ts[-1], lla_ts[-1]])
    ind_imu = 0; ind_pos = 0; ind_quat = 0; ind_lla = 0
    cur_time = 0
    init_lla = np.array([lla_lat[0], lla_lon[0], lla_alt[0]])
    cur_accl_rfu = None
    while cur_time < end_time:
        cur_time = np.min([imu_ts[ind_imu], quat_ts[ind_quat], lla_ts[ind_lla]])
        imin = np.argmin([imu_ts[ind_imu], quat_ts[ind_quat], lla_ts[ind_lla]])
        if imin == 0:
            imu_time = imu_ts[ind_imu]
            if cur_quat_ned_bodyfrd is not None:
                cur_accl_ned = np.array([accl_ned_x[ind_imu], accl_ned_y[ind_imu], accl_ned_z[ind_imu]])
                cur_gyro_ned = np.array([gyro_ned_x[ind_imu], gyro_ned_y[ind_imu], gyro_ned_z[ind_imu]])
                cur_accl_rfu = (quat_rfu_frd*(cur_quat_ned_bodyfrd.inv())).rotate_vec(cur_accl_ned)
                cur_gyro_rfu = (quat_rfu_frd*(cur_quat_ned_bodyfrd.inv())).rotate_vec(cur_gyro_ned)
                allInOneKalman.predict(t=imu_ts[ind_imu], acc=cur_accl_rfu, omega=cur_gyro_rfu)
            ind_imu += 1
        elif imin == 1:
            quat_time = quat_ts[ind_quat]
            cur_quat_ned_bodyfrd = Quaternion(x=quat_ned_bodyfrd_x[ind_quat], y=quat_ned_bodyfrd_y[ind_quat], z=quat_ned_bodyfrd_z[ind_quat], w=quat_ned_bodyfrd_w[ind_quat])      
            ind_quat += 1
        elif imin == 2:
            lla_time = lla_ts[ind_lla]
            if cur_accl_rfu is not None:
                gps_pos_ned = pymap3d.geodetic2ned(lla_lat[ind_lla], lla_lon[ind_lla], lla_alt[ind_lla], 
                                               init_lla[0], init_lla[1], init_lla[2])
                gps_pos_enu = quat_enu_ned.rotate_vec(np.array(gps_pos_ned)) 
                gps_pos_enu = np.array([[gps_pos_enu[0], gps_pos_enu[1], gps_pos_enu[2]]]).T
                allInOneKalman.update(t=lla_ts[ind_lla], pos_enu=gps_pos_enu)
                estimation = allInOneKalman.getEstimate()#t=pos_time)
                
                # iekf_pos[ind_lla] = np.hstack((np.array([lla_time]), quat_enu_ned.inv().rotate_vec(estimation['iekf'][1].T[0])))
                # tfiekf_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['tfiekf'][1].T[0]))
                # se3_R3_EqF_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['se3_R3_EqF'][1].T[0]))
                # se23_se3_EqF_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['se23_se3_EqF'][1].T[0]))
                se23_se23_EqF_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['se23_se23_EqF'][1].T[0]))
                # mekf_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['mekf'][1].T[0]))
                # liekf_pos[ind_lla] = np.hstack((np.array([lla_time]), estimation['liekf'][1].T[0]))
                
                # iekf_R[ind_lla] = estimation['iekf'][0]
                # tfiekf_R[ind_pos] = estimation['tfiekf'][0]
                # se3_R3_EqF_R[ind_pos] = estimation['se3_R3_EqF'][0]
                # se23_se3_EqF_R[ind_pos] = estimation['se23_se3_EqF'][0]
                se23_se23_EqF_R[ind_pos] = estimation['se23_se23_EqF'][0]
                # mekf_R[ind_pos] = estimation['mekf'][0]
                # liekf_R[ind_pos] = estimation['liekf'][0]
                
            ind_lla += 1
    
    plt.figure(2)   # plot  accl_ned and gyro_ned data
    plt.subplot(211)
    plt.plot(imu_ts, accl_ned_x, label='accl_ned_x')
    plt.plot(imu_ts, accl_ned_y, label='accl_ned_y')
    plt.plot(imu_ts, accl_ned_z, label='accl_ned_z')
    plt.title('Accelerations in NED Frame')
    plt.xlabel('Time')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    
    plt.subplot(212)
    plt.plot(imu_ts, gyro_ned_x, label='gyro_ned_x')
    plt.plot(imu_ts, gyro_ned_y, label='gyro_ned_y')
    plt.plot(imu_ts, gyro_ned_z, label='gyro_ned_z')
    plt.title('Gyro Rates in NED Frame')
    plt.xlabel('Time')
    plt.ylabel('Gyro Rate (rad/s)')
    plt.legend()
    
    plt.tight_layout()
    # plt.show()
    
    plt.figure(1)
    plt.subplot(311)
    # plt.plot(iekf_pos[:, 0], iekf_pos[:, 1], label='iekf_pos_x')
    # plt.plot(tfiekf_pos[:, 0], tfiekf_pos[:, 1], label='tfiekf_pos_x')
    # plt.plot(se3_R3_EqF_pos[:, 0], se3_R3_EqF_pos[:, 1], label='se3_R3_EqF_pos_x')
    # plt.plot(se23_se3_EqF_pos[:, 0], se23_se3_EqF_pos[:, 1], label='se23_se3_EqF_pos_x')
    plt.plot(se23_se23_EqF_pos[:, 0], se23_se23_EqF_pos[:, 1], label='se23_se23_EqF_pos_x')
    # plt.plot(mekf_pos[:, 0], mekf_pos[:, 1], label='mekf_pos_x')
    # plt.plot(liekf_pos[:, 0], liekf_pos[:, 1], label='liekf_pos_x')
    plt.plot(pos_ned_ts, pos_ned_x, label='pos_ned_x')
    plt.grid();    plt.legend();    plt.title('Position X'); plt.xlabel('time'); plt.ylabel('pos_ned_x')
    plt.subplot(312)
    # plt.plot(iekf_pos[:, 0], iekf_pos[:, 2], label='iekf_pos_y')
    # plt.plot(tfiekf_pos[:, 0], tfiekf_pos[:, 2], label='tfiekf_pos_y')
    # plt.plot(se3_R3_EqF_pos[:, 0], se3_R3_EqF_pos[:, 2], label='se3_R3_EqF_pos_y')
    # plt.plot(se23_se3_EqF_pos[:, 0], se23_se3_EqF_pos[:, 2], label='se23_se3_EqF_pos_y')
    plt.plot(se23_se23_EqF_pos[:, 0], se23_se23_EqF_pos[:, 2], label='se23_se23_EqF_pos_y')
    # plt.plot(mekf_pos[:, 2], label='mekf_pos_y')
    # plt.plot(liekf_pos[:, 2], label='liekf_pos_y')
    plt.plot(pos_ned_ts, pos_ned_y, label='pos_ned_y')
    plt.grid();    plt.legend();    plt.title('Position Y'); plt.xlabel('time'); plt.ylabel('pos_ned_y')
    plt.subplot(313)
    # plt.plot(iekf_pos[:, 0], iekf_pos[:, 3], label='iekf_pos_z')
    # plt.plot(tfiekf_pos[:, 0], tfiekf_pos[:, 3], label='tfiekf_pos_z')
    # plt.plot(se3_R3_EqF_pos[:, 0], se3_R3_EqF_pos[:, 3], label='se3_R3_EqF_pos_z')
    # plt.plot(se23_se3_EqF_pos[:, 0], se23_se3_EqF_pos[:, 3], label='se23_se3_EqF_pos_z')
    plt.plot(se23_se23_EqF_pos[:, 0], se23_se23_EqF_pos[:, 3], label='se23_se23_EqF_pos_z')
    # plt.plot(mekf_pos[:, 0], mekf_pos[:, 3], label='mekf_pos_z')
    # plt.plot(liekf_pos[:, 0], liekf_pos[:, 3], label='liekf_pos_z')
    plt.plot(pos_ned_ts, pos_ned_z, label='pos_ned_z')
    plt.grid();    plt.legend();    plt.title('Position Z'); plt.xlabel('time'); plt.ylabel('pos_ned_z')
    plt.show()
    i=1
 
if 1:    # plot summary plots
    tim = current_ts 
    pitch = np.zeros(len(tim))
    roll = np.zeros(len(tim))
    yaw = np.zeros(len(tim))
    for i in range(len(tim)):
        euler = Quaternion(quat_ned_bodyfrd_x[i],quat_ned_bodyfrd_y[i],quat_ned_bodyfrd_z[i],quat_ned_bodyfrd_w[i]).to_euler()
        roll[i] = np.rad2deg(euler.rpy[0])
        pitch[i] = np.rad2deg(euler.rpy[1])
        yaw[i] = np.rad2deg(euler.rpy[2])

    roll_rate = np.gradient(roll, tim)/100
    pitch_rate = np.gradient(pitch, tim)/100
    yaw_rate = np.gradient(yaw, tim)/100
    length = len(tim)

    fig = plt.figure(6)
    fx = fig.add_subplot(411)
    fx.plot(tim, vel_des_ned_x[:length], label='vel_des_ned_x')
    fx.plot(tim, vel_ned_x[:length], label='vel_ned_x')
    fx.set_xlabel('time');            fx.set_ylabel('vel_ned_x')
    fx.grid(True);                    fx.legend(loc='upper right')
    fx = fig.add_subplot(412, sharex=fx)
    fx.plot(tim, vel_des_ned_y[:length], label='vel_des_ned_y')
    fx.plot(tim, vel_ned_y[:length], label='vel_ned_y')
    fx.set_xlabel('time');            fx.set_ylabel('vel_ned_y')
    fx.grid(True);                    fx.legend(loc='upper right')
    fx = fig.add_subplot(413, sharex=fx)
    fx.plot(tim, vel_des_ned_z[:length], label='vel_des_ned_z')
    fx.plot(tim, vel_ned_z[:length], label='vel_ned_z')
    fx.set_xlabel('time');            fx.set_ylabel('vel_ned_z')
    fx.grid(True);                    fx.legend(loc='upper right')
    fx = fig.add_subplot(414, sharex=fx)
    fx.plot(tim, accel_cmd_ned_x[:length], label='accel_cmd_ned_x')
    fx.plot(tim, accl_ned_x[:length], label='accl_ned_x')
    fx.set_xlabel('time');            fx.set_ylabel('accl_ned_x')
    fx.grid(True);                    fx.legend(loc='upper right')
    plt.legend()

    fig = plt.figure(5)
    ex = fig.add_subplot(311)
    ex.plot(tim, target_ned_x[:length], label='target_ned_x')
    # ex.plot(tim, est_tar_pos_ned_x, label='est_tar_pos_ned_x')
    ex.plot(tim, pos_ned_x[:length], label='pos_ned_x')
    ex.set_xlabel('time');            ex.set_ylabel('pos_ned_x')
    ex.grid(True);                    ex.legend(loc='upper right')
    ex = fig.add_subplot(312, sharex=ex)
    ex.plot(tim, target_ned_y[:length], label='target_ned_y')
    # ex.plot(tim, est_tar_pos_ned_y, label='est_tar_pos_ned_y')
    ex.plot(tim, pos_ned_y[:length], label='pos_ned_y')
    ex.set_xlabel('time');            ex.set_ylabel('pos_ned_y')
    ex.grid(True);                    ex.legend(loc='upper right')
    ex = fig.add_subplot(313, sharex=ex)
    ex.plot(tim, target_ned_z[:length], label='target_ned_z')
    # ex.plot(tim, est_tar_pos_ned_z, label='est_tar_pos_ned_z')
    ex.plot(tim, pos_ned_z[:length], label='pos_ned_z')
    ex.set_xlabel('time');            ex.set_ylabel('pos_ned_z')
    ex.grid(True);                    ex.legend(loc='upper right')
    plt.legend()
    # plt.show()

    if 1:
        fig = plt.figure(4)
        dx = fig.add_subplot(411)
        dx.plot(tim, command[:length], label='thrust_cmd')
        dx.set_xlabel('time');            dx.set_ylabel('thrust_cmd')
        dx.grid(True);                    dx.legend(loc='upper right')
        dx = fig.add_subplot(412, sharex=dx)
        dx.plot(tim, roll[:length], label='roll')
        dx.set_xlabel('time');            dx.set_ylabel('roll')
        dx.grid(True);                    dx.legend(loc='upper right')
        dx = fig.add_subplot(413, sharex=dx)
        dx.plot(tim, pitch[:length], label='pitch')
        dx.set_xlabel('time');            dx.set_ylabel('pitch')
        dx.grid(True);                    dx.legend(loc='upper right')
        dx = fig.add_subplot(414, sharex=dx)
        dx.plot(tim, yaw[:length], label='yaw')
        dx.set_xlabel('time');            dx.set_ylabel('yaw')
        dx.grid(True);                    dx.legend(loc='upper right')
        plt.legend()

        fig = plt.figure(3)
        cx = fig.add_subplot(411)
        cx.plot(tim, command[:length], label='thrust_cmd')
        cx.set_xlabel('time');            cx.set_ylabel('thrust_cmd')
        cx.grid(True);                    cx.legend(loc='upper right')
        cx = fig.add_subplot(412, sharex=cx)
        cx.plot(tim, pos_ned_x[:length], label='pos_ned_x')
        cx.set_xlabel('time');            cx.set_ylabel('pos_ned_x')
        cx.grid(True);                    cx.legend(loc='upper right')
        cx = fig.add_subplot(413, sharex=cx)
        cx.plot(tim, pos_ned_y[:length], label='pos_ned_y')
        cx.set_xlabel('time');            cx.set_ylabel('pos_ned_y')
        cx.grid(True);                    cx.legend(loc='upper right')
        cx = fig.add_subplot(414, sharex=cx)
        cx.plot(tim, pos_ned_z[:length], label='pos_ned_z')
        cx.set_xlabel('time');            cx.set_ylabel('pos_ned_z')
        cx.grid(True);                    cx.legend(loc='upper right')
        plt.legend()

    target_pos = np.array([target_ned_x[0], target_ned_y[0], target_ned_z[0]])

    for i in range(1,len(tim)-1):
        z0 = pos_ned_z[i-1]
        z1 = pos_ned_z[i]
        if abs(z1) < .5 :        
            xZ0 = pos_ned_x[i-1]
            yZ0 = pos_ned_y[i-1]
            target_pos = np.array([target_ned_x[i], target_ned_y[i], target_ned_z[i]])
            miss = target_pos - np.array([xZ0, yZ0, z1])
            print("miss", miss, ", dist is ", np.linalg.norm(miss),", target pos %.3f %.3f"%(target_pos[0], target_pos[1]))
        if (z1) > 0:
            break

    fig = plt.figure(2)
    bx = fig.add_subplot(111, projection='3d')
    bx.plot(pos_ned_x, pos_ned_y, pos_ned_z, label='pos_ned')
    bx.plot([target_pos[0]], [target_pos[1]], [target_pos[2]],'x', color='m', label='target', markersize=10)
    bx.set_xlabel('X ');            bx.set_ylabel('Y ');            bx.set_zlabel('Z ')
    bx.grid(True);                  bx.legend(loc='upper right')
    plt.axis('equal')
    plt.show()
    pass

if 0:    # plot 3D trajectory
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    N=100
    for i in range(1,len(tim)-1):
        ax.plot(pos_ned_x[i:i+2], pos_ned_y[i:i+2], pos_ned_z[i:i+2],'k')

        if i%N==0:
            accl_ned = np.array([accl_ned_x[i], accl_ned_y[i], accl_ned_z[i]])
            vel_ned = np.array([vel_ned_x[i], vel_ned_y[i], vel_ned_z[i]])
            pos_ned = np.array([pos_ned_x[i], pos_ned_y[i], pos_ned_z[i]])
            gyro_ned = np.array([gyro_ned_x[i], gyro_ned_y[i], gyro_ned_z[i]])
            quat_bodyfrd_cam = Quaternion(x=quat_bodyfrd_cam_x[i], y=quat_bodyfrd_cam_y[i], z=quat_bodyfrd_cam_z[i], w=quat_bodyfrd_cam_w[i])        
            quat_ned_bodyfrd = Quaternion(x=quat_ned_bodyfrd_x[i], y=quat_ned_bodyfrd_y[i], z=quat_ned_bodyfrd_z[i], w=quat_ned_bodyfrd_w[i])
            pos_lla = np.array([lla_lat[i], lla_lon[i], lla_alt[i]])
            rpy_rate_cmd = np.array([rpy_rate_cmd_x[i], rpy_rate_cmd_y[i], rpy_rate_cmd_z[i]])        
            quat_ned_desbodyfrd_cmd = Quaternion(x=quat_ned_desbodyfrd_cmd_x[i], y=quat_ned_desbodyfrd_cmd_y[i], z=quat_ned_desbodyfrd_cmd_z[i], w=quat_ned_desbodyfrd_cmd_w[i])
            prev_los_ned_dir = np.array([prev_los_ned_dir_x[i], prev_los_ned_dir_y[i], prev_los_ned_dir_z[i]])
            camera_ned = np.array([camera_ned_x[i], camera_ned_y[i], camera_ned_z[i]])
            target_ned = np.array([target_ned_x[i], target_ned_y[i], target_ned_z[i]])
            image_size_px = np.array([image_size_px_x[i], image_size_px_y[i]])
            image_fov_vec = np.array([image_fov_vec_x[i], image_fov_vec_y[i]])
            quat_bodyfrd_cam = Quaternion(x=quat_bodyfrd_cam_x[i], y=quat_bodyfrd_cam_y[i], z=quat_bodyfrd_cam_z[i], w=quat_bodyfrd_cam_w[i])
            


            plot3DBox(pos_ned,quat_ned_bodyfrd, ax=ax, linewidth=2)
            plot3DBox(pos_ned,quat_ned_desbodyfrd_cmd, ax=ax, linewidth=.5)
            # plot3Dcoord(pos_ned, quat_ned_bodyfrd, ax=ax, linewidth=0.5)
            # plot3Dcoord(pos_ned, quat_ned_desbodyfrd_cmd, ax=ax, linewidth=0.5)

            vel_ned_dir = vel_ned/np.linalg.norm(vel_ned)
            ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], vel_ned_dir[0], vel_ned_dir[1], vel_ned_dir[2], color='c', label='cur_vel_ned', linewidth=2)
            accl_ned_dir = accl_ned/np.linalg.norm(accl_ned)/2
            ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], accl_ned_dir[0], accl_ned_dir[1], accl_ned_dir[2], color='r', label='cur_accl_ned', linewidth=4)
            accl_cmd_ned = np.array([accel_cmd_ned_x[i], accel_cmd_ned_y[i], accel_cmd_ned_z[i]])
            desiredAccl_ned_dir = accl_cmd_ned/np.linalg.norm(accl_cmd_ned)/2
            ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], desiredAccl_ned_dir[0], desiredAccl_ned_dir[1], desiredAccl_ned_dir[2], color='m', label='desiredAccl_ned', linewidth=2)
            # ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], accel_cmd_ned_dir[0], accel_cmd_ned_dir[1], accel_cmd_ned_dir[2], color='m', label='accel_cmd_ned', linewidth=2)
            # ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], desiredAccl_ned_dir[0], desiredAccl_ned_dir[1], desiredAccl_ned_dir[2], color='y', label='desiredAccl_ned', linewidth=2)
            # ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], desiredAccl_ned_wgravity[0], desiredAccl_ned_wgravity[1], desiredAccl_ned_wgravity[2], color='g', label='desiredAccl_ned_wgravity', linewidth=2)
            
            los_ned_dir_est = np.array([los_ned_dir_x[i], los_ned_dir_y[i], los_ned_dir_z[i]])
            ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], los_ned_dir_est[0], los_ned_dir_est[1], los_ned_dir_est[2], color='r', label='los_ned_dir', linewidth=2)

            target_pos = np.array([target_ned_x[i], target_ned_y[i], target_ned_z[i]])
            los_ned_true = target_pos - pos_ned
            los_ned_true_dir = los_ned_true/np.linalg.norm(los_ned_true)
            ax.quiver(pos_ned[0], pos_ned[1], pos_ned[2], los_ned_true_dir[0], los_ned_true_dir[1], los_ned_true_dir[2], color='b', label='los_ned_true', linewidth=2)
            target_pos_close = pos_ned + los_ned_true_dir*2

            ax.plot([target_pos_close[0]], [target_pos_close[1]], [target_pos_close[2]],'x', color='m', label='target', markersize=10)          

            quat_ned_cam = quat_ned_bodyfrd*quat_bodyfrd_cam
            plotCamera(pos_ned=pos_ned, fov_vec=image_fov_vec, quat_ned_cam=quat_ned_cam, ax=ax)

            ax.set_xlabel('X Label');            ax.set_ylabel('Y Label');            ax.set_zlabel('Z Label')


        if i%N == 0 and i>0:
            plt.legend()
            plt.axis('equal')
            plt.show()
            pass
        
