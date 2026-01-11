#!/bin/python3
import time
#from hardware_adapter import Hardware_Adapter
#from virtual_tracker import Virtual_Tracker
from common import *
import numpy as np
# from config_parser import Config_Parser
from control import *
# from tracker import Tracker
import os
import sys
import zmq
import zmqWrapper
import zmqTopics
import pickle
import mps

sys.path.append(os.path.join(os.path.dirname(__file__), 'QuadSim'))
from QuadSim.quadFiles.quad import Quadcopter
from QuadSim.trajectory import Trajectory
from QuadSim.ctrl import Control
from QuadSim.utils.windModel import Wind
from QuadSim.utils.display import makeFigures
from QuadSim.utils.animation import sameAxisAnimation

pubTopicsList = [
               [zmqTopics.topicMavlinkFlightData],
            ]
pubSock = zmqWrapper.publisher(zmqTopics.topicMavlinkPort) #TODO: change to pubTopicsList

mpsDict = {}
for topic in pubTopicsList:
    mpsDict[topic[0]] = mps.MPS(topic[0])

subSock = zmqWrapper.subscribe([zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdVelNed,
                                zmqTopics.topicGuidenceCmdVelBody,
                                # zmqTopics.topicGuidenceCmdTakeoff,
                                # zmqTopics.topicGuidenceCmdLand,
                                zmqTopics.topicGuidanceCmdArm,
                                ], zmqTopics.topicGuidenceCmdPort)

#################################################################################################################

def getFlightData(quad):
    flight_Data = Flight_Data()
    success = False
    
    flight_Data.local_ts = time.monotonic()
    flight_Data.timestamp = time.time()
    flight_Data.pos_ned_m.ned = quad.pos
    flight_Data.pos_ned_m.vel_ned = quad.vel
    flight_Data.pos_ned_m.timestamp = time.time()
    flight_Data.altitude_m = quad.pos[2]
    flight_Data.heading = quad.euler[2]
    flight_Data.throttle = quad.thr[2]

    flight_Data.quat_ned_bodyfrd = Quaternion(w=quad.quat[0], x=quad.quat[1], y=quad.quat[2], z=quad.quat[3])
    flight_Data.rpy_rates = omega_frd2rpyRate(quad.euler, quad.omega)

    flight_Data.rpy = quad.euler
    flight_Data.filt_pos_lla_deg.lla = quad.pos
    flight_Data.relative_m = quad.pos[2]

    flight_Data.amsl_m = quad.pos[2]
    flight_Data.local_m = quad.pos[2]
    flight_Data.monotonic_m = quad.pos[2]
    flight_Data.terrain_m = quad.pos[2]
    flight_Data.bottom_clearance_m = quad.pos[2]

    flight_Data.imu_ned.accel = quad.acc
    flight_Data.imu_ned.gyro = quad.omega
    flight_Data.imu_ned.timestamp = flight_Data.timestamp

    flight_Data.gathered['imu_ned'] = True
    flight_Data.gathered['absolute_press_hpa'] = True
    flight_Data.gathered['differential_press_hpa'] = True
    flight_Data.gathered['pressure'] = True
    flight_Data.gathered['temperature'] = True
    flight_Data.gathered['relative_m'] = True
    flight_Data.gathered['amsl_m'] = True
    flight_Data.gathered['local_m'] = True
    flight_Data.gathered['monotonic_m'] = True
    flight_Data.gathered['terrain_m'] = True
    flight_Data.gathered['bottom_clearance_m'] = True
    flight_Data.gathered['pos_ned_m'] = True
    flight_Data.gathered['vel_ned_m'] = True
    flight_Data.gathered['rpy'] = True
    flight_Data.gathered['rpy_rates'] = True
    flight_Data.gathered['quat_ned_bodyfrd'] = True
    flight_Data.gathered['rpy_rates'] = True
    flight_Data.gathered['pos_ned_m'] = True
    flight_Data.gathered['vel_ned_m'] = True            

    return flight_Data

################################################################################################################
def listenerToCommands(traj, quad):
    ret = zmq.select([subSock], [], [], timeout=0.001)
    # ret = ret[0]
    if ret[0] is None or len(ret[0]) == 0:
        return traj
    data = subSock.recv_multipart()
    topic = data[0]
    data = pickle.loads(data[1])
    
    if topic == zmqTopics.topicGuidenceCmdAttitude:
        targetQuat = Quaternion(x=data['quatNedDesBodyFrdCmd'][1], y=data['quatNedDesBodyFrdCmd'][2], z=data['quatNedDesBodyFrdCmd'][3], w=data['quatNedDesBodyFrdCmd'][0])
        rpyRateCmd = Rate_Cmd(roll=data['rpyRateCmd'][0], pitch=data['rpyRateCmd'][1], yaw=data['rpyRateCmd'][2])
        thrustCmd = data['thrustCmd']
        isRate = data['isRate']
        traj.desThr = thrustCmd
        if isRate:
            traj.desEul = rpyRateCmd.to_euler()
        else:
            traj.desEul = targetQuat.to_euler()
        
    elif topic == zmqTopics.topicGuidenceCmdVelNed:     
        yawCmd = data['yawCmd']
        yawRateCmd = data['yawRateCmd'] 
        velCmd = data['velCmd']
        
        traj.desVel = velCmd
        traj.desYawRate = yawRateCmd if not np.isnan(yawRateCmd) else np.nan
        traj.desEul = np.array([np.nan, np.nan, yawCmd]) if not np.isnan(yawCmd) else np.array([np.nan, np.nan, np.nan])
        traj.desPos = np.array([np.nan, np.nan, np.nan])
        
    elif topic == zmqTopics.topicGuidenceCmdVelBody:     
        yawCmd = data['yawCmd'] 
        yawRateCmd = data['yawRateCmd']
        dcm_ned_bodyfrd=quad.quat.to_dcm()
        velCmd = (dcm_ned_bodyfrd.T)@(data['velCmd'])
        # velCmd[2] = 0.0
        traj.desVel = velCmd
        traj.desYawRate = yawRateCmd if not np.isnan(yawRateCmd) else np.nan
        traj.desEul = np.array([np.nan, np.nan, yawCmd]) if not np.isnan(yawCmd) else np.array([np.nan, np.nan, np.nan])
        traj.desPos = np.array([np.nan, np.nan, np.nan])
        
    elif topic == zmqTopics.topicGuidenceCmdAcc:
        yawCmd = data['yawCmd'] 
        yawRateCmd = data['yawRateCmd']
        accCmd = data['accCmd']
        traj.desAcc = accCmd
        traj.desYawRate = yawRateCmd if not np.isnan(yawRateCmd) else np.nan
        traj.desEul = np.array([np.nan, np.nan, yawCmd]) if not np.isnan(yawCmd) else np.array([np.nan, np.nan, np.nan])
        traj.desPos = np.array([np.nan, np.nan, np.nan])
        traj.desVel = np.array([np.nan, np.nan, np.nan])
    return traj
                      

############################################################################################################################
def quad_sim(t, Ts, quad, ctrl, wind, traj):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts

    # Trajectory for Desired States 
    # ---------------------------
    desPos = traj.desPos      # Desired position (x, y, z)
    
    desVel = traj.desVel      # Desired velocity (xdot, ydot, zdot)
       
    desAcc = traj.desAcc      # Desired acceleration (xdotdot, ydotdot, zdotdot)
    
    desThr = traj.desThr      # Desired thrust in N-E-D directions (or E-N-U, if selected)
    desEul = traj.desEul      # Desired orientation in the world frame (phi, theta, psi)
    desPQR = traj.desPQR      # Desired angular velocity in the body frame (p, q, r)
    desYawRate = traj.desYawRate         # Desired yaw speed
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(quad=quad, sDes=sDes, Ts=Ts)

    return t

############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
def main():
    flight_Data = Flight_Data()
    
    start_time = time.time()
 
    # Simulation Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.002
    Tf = 10
    ifsave = 0
 
    # Choose trajectory settings
    # --------------------------- 
    trajSelect = np.zeros(3)

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(Ti)
    traj = Trajectory(quad, "xy_vel_z_pos", trajSelect)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)

    # Trajectory for First Desired States
    # ---------------------------
    sDes = traj.desiredState(0, Ts, quad)        

    # # Generate First Commands
    # # ---------------------------
    # ctrl.controller(traj, quad, sDes, Ts)
    
    # Initialize Result Matrixes
    # ---------------------------
    numTimeStep = int(Tf/Ts+1)

    t_all          = np.zeros(numTimeStep)
    s_all          = np.zeros([numTimeStep, len(quad.state)])
    pos_all        = np.zeros([numTimeStep, len(quad.pos)])
    vel_all        = np.zeros([numTimeStep, len(quad.vel)])
    quat_all       = np.zeros([numTimeStep, len(quad.quat)])
    omega_all      = np.zeros([numTimeStep, len(quad.omega)])
    euler_all      = np.zeros([numTimeStep, len(quad.euler)])
    sDes_traj_all  = np.zeros([numTimeStep, len(traj.sDes)])
    sDes_calc_all  = np.zeros([numTimeStep, len(ctrl.sDesCalc)])
    w_cmd_all      = np.zeros([numTimeStep, len(ctrl.w_cmd)])
    wMotor_all     = np.zeros([numTimeStep, len(quad.wMotor)])
    thr_all        = np.zeros([numTimeStep, len(quad.thr)])
    tor_all        = np.zeros([numTimeStep, len(quad.tor)])

    t_all[0]            = Ti
    s_all[0,:]          = quad.state
    pos_all[0,:]        = quad.pos
    vel_all[0,:]        = quad.vel
    quat_all[0,:]       = quad.quat
    omega_all[0,:]      = quad.omega
    euler_all[0,:]      = quad.euler
    sDes_traj_all[0,:]  = traj.sDes
    sDes_calc_all[0,:]  = ctrl.sDesCalc
    w_cmd_all[0,:]      = ctrl.w_cmd
    wMotor_all[0,:]     = quad.wMotor
    thr_all[0,:]        = quad.thr
    tor_all[0,:]        = quad.tor

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 1
    
    outFreq = 500
    outDT = 1/outFreq
    current_ts = time.monotonic()
    outTime = current_ts+outDT
    logDT = .1
    
    printFreq = 1
    printDt = 1/printFreq
    printTime = current_ts+printDt
    logTime = current_ts+logDT
    globalTime = 0
    while(globalTime<Tf):
        start_time = time.monotonic()
        flight_Data = getFlightData(quad)
        if(time.monotonic() > outTime):
            outTime = time.monotonic()+outDT
            data = pickle.dumps(flight_Data)
            pubSock.send_multipart([zmqTopics.topicMavlinkFlightData, data])
        if(time.monotonic() > printTime):
            printTime = time.monotonic()+printDt
            print("timestamp: ", flight_Data.timestamp)
        traj = listenerToCommands(traj=traj, quad=quad)

        if np.linalg.norm(traj.desVel) > 0.01:
            pass
      # for ind in range(int(control_dt/Ts)):
        quad_sim(t=0, Ts=Ts, quad=quad, ctrl=ctrl, wind=wind, traj=traj)
        
        # print("{:.3f}".format(t))
        if(time.monotonic() > logTime):
            logTime = time.monotonic()+logDT
            t_all[i]             = t
            s_all[i,:]           = quad.state
            pos_all[i,:]         = quad.pos
            vel_all[i,:]         = quad.vel
            quat_all[i,:]        = quad.quat
            omega_all[i,:]       = quad.omega
            euler_all[i,:]       = quad.euler
            sDes_traj_all[i,:]   = traj.sDes
            sDes_calc_all[i,:]   = ctrl.sDesCalc
            w_cmd_all[i,:]       = ctrl.w_cmd
            wMotor_all[i,:]      = quad.wMotor
            thr_all[i,:]         = quad.thr
            tor_all[i,:]         = quad.tor       
            i += 1
        end_time = time.monotonic()
        # print("Time taken: {:.6f}s".format(end_time - start_time))
  
        if Ts-(end_time-start_time) > 0:
            time.sleep(Ts-(end_time-start_time))
        else:
            Ts = end_time-start_time
        globalTime += Ts
  
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))

    # View Results
    # ---------------------------

    # utils.fullprint(sDes_traj_all[:,3:6])
    makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all)
    ani = sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    plt.show()
    
    
    plt.figure(2)
    plt.plot(ulgData['vehicle_local_position_vx']['timestamp'], ulgData['vehicle_local_position_vx']['data'])
    plt.plot(ulgData['vehicle_local_position_vy']['timestamp'], ulgData['vehicle_local_position_vy']['data'])
    plt.legend(['vx ulg', 'vy ulg'])
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    ani = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    # plt.show()
    pass

if __name__=='__main__':
    main()
