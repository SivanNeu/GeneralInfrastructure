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
import pymap3d
# import cv2
import sys
import zmq
import zmqWrapper
import zmqTopics
import pickle
# from Filter.AllInOneKalman import AllInOneKalman
# from virtual_tracker import Tracker

# from controlGeom import GeometricController
# from controlGeomAdaptive import AdaptiveGeometricController
# from controlBrescianini import BrescianiniController
# from controlKooijman import KooijmanController
from controlVelocityPID import VelocityPIDController
from controlVelocityRL import VelocityRLController
# from controlAccelerationPID import AccelerationPIDController
# from controlJaeyoung import JaeyoungController

# sys.path.append('../')
# try:
#     import src.zmqTopics as zmqTopics
#     import src.zmqWrapper as zmqWrapper
#     import src.mps as mps
# except:
#     import zmqTopics
#     import zmqWrapper
#     import mps
REAL_TIME_SIMULATION = False

class MISSION_TYPE(Enum):
    NONE = 0
    WAYPOINT = 1
    VELOCITY = 2
    CIRCLE = 3
    LISSAJOUS = 4
    TRACKER = 5
    SECTION = 6
    SPINNING = 7
#################################################################################################################
GOAL_LOOP_FREQ_HZ = 50
GPS_SEARCH_TIMEOUT_SEC = 3
#################################################################################################################
class System_Manager():
    def __init__(self, config_dir, log_dir, sim_object=None, external_imu=None, use_usb_for_mavlink=False, currentTime=None):
        self._overall_start = time.monotonic() if currentTime is None else currentTime
        self._config_dir = config_dir
        self._log_dir = log_dir
        self._finished = False
        self._target_threshold_m = 5
        self._target_received = False
        self._prev_imu_timestamp = 0
        self._prev_los_ned_dir = np.zeros(3)
        self._prev_pos_ned = np.zeros(3)
        self._prev_imu_ts = 0
        self._prev_ts = self._overall_start
        self._tracker_pos_px = None
        self._init_pos_lla_deg = None
        self._ai1Kestimates = None
        # vehicle_config_path = self._get_vehicle_config_file(self._config_dir)
        self._input_logger = None #Logger("input"+time.strftime("%Y%m%d_%H%M%S"), self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="CSV")
        # vehicle_data_parser = Config_Parser(path=vehicle_config_path)
        self.dronemass = 0.55
        # if(vehicle_data_parser is None):
        #     print("config init failed")    
        self.prev_quat_ned_desbodyfrd_cmd = None

        self.destHeight = None
        self.tar_measurement_ned = np.array([0,0,0])
        self.heading_dir_ned = np.array([-1,1,0])/np.linalg.norm(np.array([-1,1,0])) 
  
        self.holdonHeading = None
        self.holdonPos_ned = None
        self.holdonTime = None       
        self.yawDefinedDir_ned = None   
        self.homingStage = HOMING_STAGE.NONE     

        self.pointIndex = 0
        self.offboardEntry = False
        
        ##############################
        # start scenario definitions #
        ##############################
        factor = 1
        self.referencePoint = np.array([ 10, 0, 0])
        self.desiredHeadingDir_ned = np.array([1, -1, 0])
        # self.pointList = np.array([[0, 10*factor*0, 0], [0, 10*factor, 0]])
        
        self.missionType = MISSION_TYPE.WAYPOINT    # 1 - WAYPOINT, 2 - VELOCITY, 3 - CIRCLE, 4 - LISSAJOUS, 5 - TRACKER, 6 - SECTION, 7 - SPINNING
        self.yawControlType = YAW_COMMAND.DEFINED_DIR   #YAW_COMMAND.CAMERA_DIR   #YAW_COMMAND.VELOCITY_DIR  # YAW_COMMAND.HOLD_CUR_DIR # YAW_COMMAND.DEFINED_DIR
        
        self.maximalVelocity = 10*factor # m/s (horizontal)
        self.descentVelocity = 10
        self.originOffset_frd = np.array([0,0,0])   # target waypoint in mode WAYPOINT or center of the circle in mode CIRCLE
        self.terminalHomingAlowed = True 
        self.circleRadius = 15*factor
        self.controllerType = CONTROLLER_TYPE.VELOCITYRL
        
        if self.controllerType == CONTROLLER_TYPE.VELOCITYRL:        
            self._controlAux = Control(self._config_dir, self._log_dir, controller=VelocityPIDController(mass=self.dronemass, currentTime=self._overall_start), maximalVelocity=self.maximalVelocity)
            # self._controlMain = Control(self._config_dir, self._log_dir, controller=VelocityPIDController(mass=self.dronemass), maximalVelocity=self.maximalVelocity)
            self._controlMain = Control(self._config_dir, self._log_dir, controller=VelocityRLController(mass=self.dronemass, maximalVelocity=self.maximalVelocity, currentTime=self._overall_start)) 
        elif self.controllerType == CONTROLLER_TYPE.VELOCITYPID:
            self._controlMain = Control(self._config_dir, self._log_dir, controller=VelocityPIDController(mass=self.dronemass), maximalVelocity=self.maximalVelocity)
            
        # self._control = Control(self._config_dir, self._log_dir, controller=AccelerationPIDController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=GeometricController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=AdaptiveGeometricController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=JaeyoungController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=BrescianiniController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=KooijmanController(mass=self.dronemass))
        self.rateControlEnabled = False
        
        ###############################
        # end of scenario definitions #
        ###############################
        
        self.subsSock = zmqWrapper.subscribe( topics=[ zmqTopics.topicMavlinkFlightData ], port=zmqTopics.topicMavlinkPort )

        self._currentData = Flight_Data()
        self.trackData = None
        self.pubSock = zmqWrapper.publisher(zmqTopics.topicGuidenceCmdPort)

        ##self._control.reset(thrust=self._hardware_adapter.get_current_thrust(), yaw=self._hardware_adapter.get_current_yaw())
        self._current_pos_lla = LLA(timestamp=0, lla=np.zeros(3))

        self.tic = time.monotonic()
        self.dest_pos_ned = None

        # self.allInOneKalman = AllInOneKalman()

#################################################################################################################
    def _get_vehicle_config_file(self, config_dir):
        system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        vehicle_config_file_name =  system_config_parser.get_value("vehicle_config_file", default_value="")
        vehicle_config_path = os.path.join(self._config_dir,"vehicle", vehicle_config_file_name)
        return vehicle_config_path

#################################################################################################################
    def _log_input_data(self,   current_ts, imu_ts,
                                command, rpy_rate_cmd, quat_ned_desbodyfrd_cmd, step_dt, counter,
                                destination_ned, current_mode):
        # print(target_received,trk_i, trk_j )
        self._input_logger.log({"comp_time":time.monotonic(), "imu_ts":self._currentData.imu_ned.timestamp,
                                "accl_ned/x": self._currentData.imu_ned.accel[0],"accl_ned/y": self._currentData.imu_ned.accel[1], "accl_ned/z": self._currentData.imu_ned.accel[2],
                                "gyro_ned/x": self._currentData.imu_ned.gyro[0], "gyro_ned/y": self._currentData.imu_ned.gyro[1], "gyro_ned/z": self._currentData.imu_ned.gyro[2],
                                "pos_ned_ts": self._currentData.pos_ned_m.timestamp,
                                "pos_ned/x": self._currentData.pos_ned_m.ned[0], "pos_ned/y": self._currentData.pos_ned_m.ned[1], "pos_ned/z": self._currentData.pos_ned_m.ned[2],
                                "vel_ned/x": self._currentData.pos_ned_m.vel_ned[0], "vel_ned/y": self._currentData.pos_ned_m.vel_ned[1], "vel_ned/z": self._currentData.pos_ned_m.vel_ned[2],
                                "quat_ned_bodyfrd_ts": self._currentData.quat_ned_bodyfrd.timestamp,
                                "quat_ned_bodyfrd/x": self._currentData.quat_ned_bodyfrd.x, "quat_ned_bodyfrd/y": self._currentData.quat_ned_bodyfrd.y,
                                "quat_ned_bodyfrd/z": self._currentData.quat_ned_bodyfrd.z, "quat_ned_bodyfrd/w": self._currentData.quat_ned_bodyfrd.w,
                                "lla_ts": self._currentData.raw_pos_lla_deg.timestamp,
                                "lla/lat_deg": self._currentData.raw_pos_lla_deg.lla[0], "lla/lon_deg": self._currentData.raw_pos_lla_deg.lla[1], "lla/alt": self._currentData.raw_pos_lla_deg.lla[2],
                                "current_ts":current_ts, "step_dt":step_dt, "imu_ts":imu_ts,
                                "current_alt": self._currentData.relative_m, "command": command, 
                                "rpy_rate_cmd/x": rpy_rate_cmd[0], "rpy_rate_cmd/y": rpy_rate_cmd[1], "rpy_rate_cmd/z": rpy_rate_cmd[2], 
                                "quat_ned_desbodyfrd_cmd/x": quat_ned_desbodyfrd_cmd.x, "quat_ned_desbodyfrd_cmd/y": quat_ned_desbodyfrd_cmd.y,
                                "quat_ned_desbodyfrd_cmd/z": quat_ned_desbodyfrd_cmd.z, "quat_ned_desbodyfrd_cmd/w": quat_ned_desbodyfrd_cmd.w,
                                "destination_ned/x": destination_ned[0], "destination_ned/y": destination_ned[1], "destination_ned/z": destination_ned[2],
                                "counter": counter, "current_mode": current_mode, "timestamp": self._currentData.timestamp, "local_ts":self._currentData.local_ts,
                                "modeID":self._currentData.custom_mode_id})
                                 
###############################################################################################################################################
    def _get_los_ned_dir(self, track_pos_px, quat_ned_bodyfrd:Quaternion, img_size_px, camera_fov_vec, quat_bodyfrd_cam:Quaternion):
        centerPixel = img_size_px/2
        track_angle_cam = (track_pos_px - centerPixel) / img_size_px *camera_fov_vec
        
        los_dir_cam = np.array([np.sin(track_angle_cam[0]), np.sin(track_angle_cam[1]), -1])
        los_dir_cam = unitVec(los_dir_cam)
        #TODO: add here transform for camera angle as well 
        quat_ned_cam = quat_ned_bodyfrd*quat_bodyfrd_cam
        los_ned_dir = quat_ned_cam.rotate_vec(los_dir_cam)
        return los_ned_dir

###############################################################################################################################################
    def sys_manager_step(self, counter = -1, log_data=True, flight_Data=None, curTime=None):
        if curTime is None:
            curTime = time.monotonic()

        if flight_Data is None:
            self.gatherData()

            if (not self._currentData.gathered['quat_ned_bodyfrd']) or \
                (not self._currentData.gathered['pos_ned_m']) or \
                (not self._currentData.gathered['imu_ned']) :#or \
            # (not self._currentData.gathered['tracker_px']):
                print('-return-0-')
                return
        else:
            self._currentData = flight_Data
            
        self.holdonHeading = self._currentData.quat_ned_bodyfrd.rotate_vec(np.array([1, 0, 0])) if self.holdonHeading is None else self.holdonHeading
        self.holdonPos_ned = self._currentData.pos_ned_m.ned if self.holdonPos_ned is None else self.holdonPos_ned
        self.holdonTime = curTime if self.holdonTime is None else self.holdonTime
        
        # imu_ned = deepcopy(self._currentData.imu_ned)          
        pos_ned = deepcopy(self._currentData.pos_ned_m.ned)
        
        quat_ned_bodyfrd = self._currentData.quat_ned_bodyfrd
               
        current_ts = curTime
               
        controlType = None
        desired_trajectory = None
        headingTarget=None
        
        # self.heading_dir_ned = np.array([-15,0,0])-np.array([pos_ned[0],pos_ned[1],0]); self.heading_dir_ned=self.heading_dir_ned/np.linalg.norm(self.heading_dir_ned)
        self.heading_dir_ned = deepcopy(self.holdonHeading)
        if self.yawControlType == YAW_COMMAND.HOLD_CUR_DIR and self.yawDefinedDir_ned is not None:
            self.heading_dir_ned = deepcopy(self.yawDefinedDir_ned)    
        elif self.yawControlType == YAW_COMMAND.VELOCITY_DIR:
            if np.linalg.norm(self._currentData.pos_ned_m.vel_ned[0:2])>1:
                self.heading_dir_ned = deepcopy(self._currentData.pos_ned_m.vel_ned)
                self.heading_dir_ned[2] = 0; self.heading_dir_ned = unitVec(self.heading_dir_ned)
        elif self.yawControlType == YAW_COMMAND.CAMERA_DIR:
            self.heading_dir_ned = unitVec(np.array([self.los_ned_dir[0], self.los_ned_dir[1], 0]))
        elif self.yawControlType == YAW_COMMAND.DEFINED_DIR:
            self.heading_dir_ned = deepcopy(self.desiredHeadingDir_ned)
            
        missionPoint = self.holdonPos_ned + self.referencePoint;   
        if self.missionType == MISSION_TYPE.WAYPOINT:
            desired_trajectory = self.pos_point(missionPoint=[missionPoint], missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
         
        elif self.missionType == MISSION_TYPE.VELOCITY:
            missionVelocity = unitVec(missionPoint-self.holdonPos_ned)*self.maximalVelocity
            desired_trajectory = self.vel_point(missionVelocity=missionVelocity, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.CIRCLE: 
            desired_trajectory = self.horz_circle(center=missionPoint, radius=self.circleRadius, Vel=3, missionAttitudeDirection=self.heading_dir_ned)   # bui
            # desired_trajectory = self.horz_circle(center = np.array([-10,20,0]), radius=10)    # corner ok, bui fades away
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
        
        elif self.missionType == MISSION_TYPE.LISSAJOUS:
            desired_trajectory = self.command_Lissajous()
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.SECTION:
            startPoint = deepcopy(self.holdonPos_ned); endPoint = deepcopy(missionPoint)
            desired_trajectory = self.lineConstVel(startPoint=startPoint, endPoint=endPoint, startTime=self.holdonTime, 
                                                speed=self.maximalVelocity, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]           
               
        trajDest_pos_ned = np.array(self.dest_pos_ned);  trajDest_pos_ned[2] = self._currentData.pos_ned_m.ned[2] if self.destHeight is None else self.destHeight
        trajDest_vel_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][1]
        trajDest_acc_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][2]
        
        trajDest = (trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned)  
            
        # at HIGH_ALTITUDE_FOLLOW guidance state command is derived from trajDest
        # at TERMINAL_GUIDANCE    guidance state command is derived from los_ned_dir and los_distance
        command, rpyRate_cmd, quat_ned_desbodyfrd_cmd = self._controlMain.get_cmd(pos_ned=deepcopy(self._currentData.pos_ned_m.ned), 
                                                                                  vel_ned=deepcopy(self._currentData.pos_ned_m.vel_ned), 
                                                                                  accel_ned=deepcopy(self._currentData.imu_ned.accel), 
                                                                                  gyro_ned=deepcopy(self._currentData.imu_ned.gyro),
                                                                                  quat_ned_bodyfrd=deepcopy(quat_ned_bodyfrd),
                                                                                  imu_ts=self._currentData.imu_ts, step_dt=current_ts-self._prev_ts, current_ts=current_ts,                                                                
                                                                                  counter=counter, trajDest_ned=deepcopy(trajDest), controlType=controlType, 
                                                                                  headingDest=(self.heading_dir_ned, np.zeros(3), np.zeros(3)),
                                                                                  homingStage=self.homingStage, currentData = self._currentData,
                                                                                  log_data=self._input_logger is not None)
        if hasattr(self, '_controlAux'):
            commandAux, rpyRate_cmdAux, quat_ned_desbodyfrd_cmdAux = self._controlAux.get_cmd(pos_ned=deepcopy(self._currentData.pos_ned_m.ned), 
                                                                                  vel_ned=deepcopy(self._currentData.pos_ned_m.vel_ned), 
                                                                                  accel_ned=deepcopy(self._currentData.imu_ned.accel), 
                                                                                  gyro_ned=deepcopy(self._currentData.imu_ned.gyro),
                                                                                  quat_ned_bodyfrd=deepcopy(quat_ned_bodyfrd),
                                                                                  imu_ts=self._currentData.imu_ts, step_dt=current_ts-self._prev_ts, current_ts=current_ts,                                                                
                                                                                  counter=counter, trajDest_ned=trajDest, controlType=controlType, 
                                                                                  headingDest=(self.heading_dir_ned, np.zeros(3), np.zeros(3)),
                                                                                  homingStage=self.homingStage, currentData = deepcopy(self._currentData),
                                                                                  log_data=self._input_logger is not None)
            commandBody = commandAux
            if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYRL:
                commandBody = quat_ned_bodyfrd.inv().rotate_vec(commandAux)   # suppose that the main controller command is in a body frame
                
            command[2] = commandBody[2]
            
        
        rollpitchlimit = np.deg2rad(20)
        rpy=quat_ned_desbodyfrd_cmd.to_euler().rpy
        rpy[0] = np.clip(rpy[0], -rollpitchlimit, rollpitchlimit)
        rpy[1] = np.clip(rpy[1], -rollpitchlimit, rollpitchlimit)
        quat_ned_desbodyfrd_cmd = Quaternion.from_euler(rpy[0], rpy[1], rpy[2])
        
        if self.prev_quat_ned_desbodyfrd_cmd is not None and self.prev_quat_ned_desbodyfrd_cmd.dot(quat_ned_desbodyfrd_cmd) < 0:
            quat_ned_desbodyfrd_cmd = -quat_ned_desbodyfrd_cmd
        
        forward_dir_frd = quat_ned_bodyfrd.rotate_vec(np.array([1,0,0]))
        yawCmd=np.arctan2(forward_dir_frd[1], forward_dir_frd[0])
        if self.yawControlType == YAW_COMMAND.NO_CONTROL:
            yawCmd = self._currentData.heading
        else:
            heading_dir_ned = self.heading_dir_ned 
            yawCmd = np.arctan2(heading_dir_ned[1], heading_dir_ned[0])
        
        if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYPID:
            msg = { 'ts': time.monotonic(), 'velCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':np.nan, }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdVelNed, pickle.dumps(msg)])
        elif self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYRL:
            command_ned = command
            msg = { 'ts': time.monotonic(), 'velCmd':command_ned, 'yawCmd':np.nan, 'yawRateCmd':rpyRate_cmd[2] }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdVelNed, pickle.dumps(msg)])
        elif self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.ACCELERATIONPID:
            msg = { 'ts': time.monotonic(), 'accCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':np.nan, }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAcc, pickle.dumps(msg)])
        else:
            msg = { 'ts': time.monotonic(), 'thrustCmd':command, 'rpyRateCmd':rpyRate_cmd,
                'quatNedDesBodyFrdCmd':[quat_ned_desbodyfrd_cmd.w, quat_ned_desbodyfrd_cmd.x, quat_ned_desbodyfrd_cmd.y, quat_ned_desbodyfrd_cmd.z],
                'isRate':self.rateControlEnabled
            }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAttitude, pickle.dumps(msg)])

        monitorTime=1
        if time.monotonic() - self.tic >= monitorTime:
            # print('tar_pos_ned: %.3f %.3f %.3f, tar_vel_ned: %.3f %.3f %.3f'%(est_tracker_pos_ned[0], est_tracker_pos_ned[1], est_tracker_pos_ned[2], tar_vel_ned[0], tar_vel_ned[1], tar_vel_ned[2]))
            self.tic = time.monotonic()
            deltaPos_ned = missionPoint
            if quat_ned_bodyfrd is not None:
                deltaPos_ned = missionPoint - pos_ned
                deltaPos_frd = quat_ned_bodyfrd.inv().rotate_vec(deltaPos_ned)
            print('timestamp: ', self._currentData.timestamp, '--->referencePoint: %.3f %.3f %.3f, pos_ned:  %.3f %.3f %.3f '%( 
                self.referencePoint[0], self.referencePoint[1], self.referencePoint[2],
                pos_ned[0], pos_ned[1], pos_ned[2])+str(self.homingStage)+                
                " Command: + %.3f %.3f %.3f"%(command[0], command[1], command[2])+" yawRateCmd: %.3f"%(yawCmd)+
                " deltaPos_frd: %.3f %.3f %.3f"%(deltaPos_frd[0], deltaPos_frd[1], deltaPos_frd[2]))#+"   delta_frd"+str(deltaPos_frd))
            
            try:
                print('<<--', np.linalg.norm(missionPoint[0:2]-pos_ned[0:2])+" timestamp:"+str(self._currentData.timestamp))
            except:
                pass
        
        if log_data and self._input_logger is not None:
            self._log_input_data(current_ts=current_ts, step_dt=current_ts-self._prev_ts, imu_ts=self._currentData.imu_ts,                        
                                command=command, rpy_rate_cmd=rpyRate_cmd, quat_ned_desbodyfrd_cmd=quat_ned_desbodyfrd_cmd,
                                counter=counter, destination_ned=trajDest_pos_ned, current_mode=self._currentData.custom_mode_id)
            
        # self._hardware_adapter.send_command(quat_cmd=quat_ned_desbodyfrd_cmd, rpy_rate=rpyRate_cmd, thrust=command)
        self._prev_imu_ts = self._currentData.imu_ts
        self._prev_ts = current_ts
        self._prev_pos_ned = self._currentData.pos_ned_m.ned
        self.prev_quat_ned_desbodyfrd_cmd = quat_ned_desbodyfrd_cmd
        
        return msg
        
            
#################################################################################################################
    def getQuadBodyfrdCamera(self, nedYangle):
        yawCorrection = 0 #np.pi/2
        mat_cam_bodyfrd =  rotZ(-np.pi/2+yawCorrection) @ rotY(np.pi/2) @ rotY(-nedYangle) #   rotZned * rotYned * rotYned
        return Quaternion.from_matrix(mat=mat_cam_bodyfrd.T) 

#################################################################################################################
    def horz_circle(self, center=np.array([0,0,0]), radius=3, missionAttitudeDirection=None, Vel=1):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
        t=time.monotonic()-self._overall_start
        
        A = radius
        B = radius
        C = 0  #0.2

        R=(A+B)/2
        L=2*pi*R
        w=Vel/L

        d = pi / 2 * 0

        a = 2*pi*w  #.1
        b = 2*pi*w  #.2
        c = 0*2*pi*w*2  #.2

        # % t = linspace(0, 2*pi, 2*pi*100+1);
        # % x = A * sin(a * t + d);
        # % y = B * sin(b * t);
        # % z = alt + C * cos(2 * t);
        # % plot3(x, y, z);

        xd =      np.array([A *         sin(a * t + d), B *         cos(b * t), C * cos(c * t)])+center
        x_dot =  np.array([A * a *      cos(a * t + d), B * b *    -sin(b * t), C * c * -sin(c * t)])
        x_2dot =  np.array([A * a**2 * -sin(a * t + d), B * b**2 * -cos(b * t), C * c**2 * -cos(c * t)])
        x_3dot =  np.array([A * a**3 * -cos(a * t + d), B * b**3 *  sin(b * t), C * c**3 * sin(c * t)])
        x_4dot =  np.array([A * a**4 *  sin(a * t + d), B * b**4 *  cos(b * t), C * c**4 * cos(c * t)])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0,0,0])  #
        b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
        # b1d = np.array([cos(w * t), sin(w * t), 0])
        # b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
        # b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

        pos_control = [False, False, True]
        vel_control = [True, True, False]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (xd, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
############################################################################################################################

    def pos_point(self, missionPoint=[np.array([0, 0, -30])], missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
        
        t=time.monotonic()-self._overall_start
        x = missionPoint[0]
        if len(missionPoint) > 1:
            T = 10
            w = 2 * pi / T
            state = sin(w*t)
            x = missionPoint[0] + np.sign(state) * (missionPoint[1] - missionPoint[0])

        x_dot = np.array([0,0,0])
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = [True, True, True]
        vel_control = [False, False, False]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
################################################################################################################

    def vel_point(self, missionVelocity=np.array([1, 0, 0]), missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
            
        t=time.monotonic()-self._overall_start
        
        x = np.array([0, 0, 0])
        x_dot = missionVelocity
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = [False, False, False]
        vel_control = [True, True, True]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
################################################################################################################

    def lineConstVel(self, startPoint, endPoint, speed, startTime, missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
            
        finished = False
        t=time.monotonic()-startTime
        
        deltaDistance = endPoint-startPoint
        deltaDir = unitVec(deltaDistance)
        deltaDistance = np.linalg.norm(deltaDistance)
        velocity = deltaDir * speed
        x = startPoint + t*velocity
        x_dot = velocity
        if t*speed > deltaDistance:
            x = endPoint
            x_dot = np.array([0,0,0])
            finished = True
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = [True, True, True]
        vel_control = [t*speed < deltaDistance, t*speed < deltaDistance, t*speed < deltaDistance]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control), finished
##############################################################################################################
    def vert_circle(self):

        t=time.monotonic()-self._overall_start
        
        Vel=3

        A = 5
        B = 5
        C = 0  #0.2

        R=(A+B)/2
        L=2*pi*R
        w=Vel/L

        d = pi / 2 * 0

        a = 2*pi*w  #.1
        b = 2*pi*w  #.2
        c = 2*pi*w*2  #.2
        alt = -20

        # % t = linspace(0, 2*pi, 2*pi*100+1);
        # % x = A * sin(a * t + d);
        # % y = B * sin(b * t);
        # % z = alt + C * cos(2 * t);
        # % plot3(x, y, z);

        x =      np.array([C * cos(c * t),         A *         sin(a * t + d), B *         cos(b * t), alt + C * cos(c * t)])
        x_dot =  np.array([C * c * -sin(c * t),    A * a *     cos(a * t + d), B * b *    -sin(b * t)])
        x_2dot =  np.array([C * c**2 * -cos(c * t), A * a**2 * -sin(a * t + d), B * b**2 * -cos(b * t)])
        x_3dot =  np.array([C * c**3 *  sin(c * t), A * a**3 * -cos(a * t + d), B * b**3 *  sin(b * t)])
        x_4dot =  np.array([C * c**4 *  cos(c * t), A * a**4 *  sin(a * t + d), B * b**4 *  cos(b * t)])

        b1 = np.array([1,0,0])
        b1_dot = np.array([0,0,0])  #
        b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
        # self.b1d = np.array([cos(w * t), sin(w * t), 0])
        # self.b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
        # self.b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

        pos_control = [False, False, False]
        vel_control = [True, True, True]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
###############################################################################################################
    
    def command_Lissajous(self, t=None):
        if t is None:
            t=time.monotonic()-self._overall_start
        
        # Lissajous curve
        # x=A*sin(a*t+d), y=B*sin(b*t), z=alt+C*cos(c*t)
        # several common parameters:
        # a:b   d    0       pi/4      pi/2      3/4*pi    pi
        # 1:1        /      ellipse   circle    ellipce    \
        # 1:2        )      8-figure  8-figure  8-figure   (
        # 1:3        S      
        
        A = 15   # X amplitude
        B = 15   # Y amplitude
        C = 5 # Z amplitude
        alt = -1  # Z offset

        d = pi / 2 * 0

        a = .2*1.5  # X frequency
        b = .3*1.5  # Y frequency
        c = .2    # Z frequency
        w = 2 * pi / 10  # attitude rotation frequency
        (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot) = lissajous_func(t, A=A, B=B, C=C, a=a, b=b, c=c, alt=alt, w = w)
        
        pos_control = [False, False, False]
        vel_control = [True, True, True]
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
#################################################################################################################
    def gatherData(self):
        subsSock2 = zmqWrapper.subscribe( topics=[ zmqTopics.topicMavlinkFlightData ], port=zmqTopics.topicMavlinkPort )

        socks = zmq.select([subsSock2], [], [], 0.001)[0]
        
        while True:
            if len(socks) > 0:
                socket = socks[0]
                ret = socket.recv_multipart()
                topic = ret[0]
                data = pickle.loads(ret[1])
                if topic == zmqTopics.topicMavlinkFlightData:  
 
                    self._currentData = data# mavlink LOCAL_POSITION_NED      # Flight controller time                  
                    #if (self._currentData.custom_mode_id == 50593792) or \ # and self._currentData.groundspeed<0.5) or  \
                    if self._currentData.custom_mode_id != PX4_FLIGHT_STATE.OFFBOARD.value:  # HOLD ON state or POSITION state
                        
                        self._input_logger = None
                        self.yawDefinedDir_ned = np.array([np.cos(self._currentData.heading), np.sin(self._currentData.heading), 0])
                        self.holdonHeading = self.yawDefinedDir_ned
                        self.holdonPos_ned = self._currentData.pos_ned_m.ned
                        self.holdonTime = time.monotonic()
                        
                        if self._currentData.throttle>5:
                            if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYRL:
                                self._controlMain.MaximalThrust = 1
                            else:
                                self._controlMain.MaximalThrust = self._controlMain.controlnode.param.mass*9.81/self._currentData.throttle*100
                                self._controlMain.controlnode.resetIntegralErrorTerms()
                        
                        self.homingStage = HOMING_STAGE.SECTION if self.missionType == MISSION_TYPE.TRACKER else HOMING_STAGE.NONE
                        self._currentData.offboardMode = False
                        self.offboardEntry = True
                        
                    elif self._currentData.custom_mode_id == PX4_FLIGHT_STATE.OFFBOARD.value:  
                        if self._input_logger is None:
                            self._input_logger = Logger(log_name=time.strftime("%Y%m%d_%H%M%S")+"_system_manager", log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="CSV")

                        self._currentData.offboardMode = True
                        if self._controlMain.controlnode.controllerType != CONTROLLER_TYPE.VELOCITYRL:
                            if not self._controlMain.controlnode.use_integralTerm:
                                self._controlMain.controlnode.resetIntegralErrorTerms()
                                self._controlMain.controlnode.use_integralTerm = True

                        if self.offboardEntry and self.missionType == MISSION_TYPE.WAYPOINT and \
                           hasattr(self, 'pointList'):
                            self.pointIndex = (self.pointIndex + 1) % len(self.pointList)
                            self.referencePoint = self.pointList[self.pointIndex]
                            self.offboardEntry = False
                    break
                
            socks = zmq.select([subsSock2], [], [], 0.001)[0]
            
        subsSock2.close()


############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
def getFlightData(quad, t=None ):
    if t is None:
        t=time.monotonic()
        timestamp = time.time()
    else:
        timestamp = t
    flight_Data = Flight_Data()
    success = False
    
    flight_Data.local_ts = t
    flight_Data.timestamp = timestamp
    flight_Data.pos_ned_m.ned = quad.pos
    flight_Data.pos_ned_m.vel_ned = quad.vel
    flight_Data.pos_ned_m.timestamp = timestamp
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
############################################################################################################################
def quad_sim(t, Ts, quad, ctrl, wind, desired):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts

    # Trajectory for Desired States 
    # ---------------------------
    desPos = np.array([desired['pos'][0], desired['pos'][1], desired['pos'][2]])    # Desired position (x, y, z)
    desVel = np.array([desired['vel'][0], desired['vel'][1], 0.0])    # Desired velocity (xdot, ydot, zdot)
    desAcc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
    desThr = np.zeros(3)    # Desired thrust in N-E-D directions (or E-N-U, if selected)
    desEul = np.zeros(3)    # Desired orientation in the world frame (phi, theta, psi)
    desPQR = np.zeros(3)    # Desired angular velocity in the body frame (p, q, r)
    desYawRate = desired['yaw_rate']         # Desired yaw speed
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(quad=quad, sDes=sDes, Ts=Ts)

    return t
############################################################################################################################

def main():
    if REAL_TIME_SIMULATION:
        sysMgr = System_Manager(log_dir='../logs/', config_dir='config/')
        next_loop_time = time.monotonic()
        while True:
            # time.sleep(0.0001)
            
            # Variable delay to maintain 100Hz loop frequency
            sleep_duration = next_loop_time - time.monotonic()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            
            next_loop_time += 0.01
            
            startTime = time.monotonic()
            sysMgr.sys_manager_step()
            endTime = time.monotonic()
            # print("Computation Time",endTime-startTime)
    else:
        sysMgr = System_Manager(log_dir='../logs/', config_dir='config/', currentTime=0)
        sys.path.append(os.path.join(os.path.dirname(__file__), 'QuadSim'))
        from QuadSim.quadFiles.quad import Quadcopter
        from QuadSim.trajectory import Trajectory
        from QuadSim.ctrl import Control
        from QuadSim.utils.windModel import Wind
        from QuadSim.utils.display import makeFigures
        from QuadSim.utils.animation import sameAxisAnimation
        start_time = time.time()
    
        # Simulation Setup
        # --------------------------- 
        t=0
        Ti = 0
        Ts = 0.01
        control_dt = 0.1
        
        Tf = 40
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
        i = 0
        
        desired = {'pos': [np.nan, np.nan, 0], 
                   'vel': [0.0, 0.0, 0.0], 
                   'yaw_rate': 0.0}
        globalTime = 0
        while(globalTime<Tf):
            flight_Data = getFlightData(quad, t=globalTime)
            msg=sysMgr.sys_manager_step(flight_Data=flight_Data, curTime=globalTime)

            desired['vel'] = msg['velCmd']
            desired['yaw_rate'] = msg['yawRateCmd']
            for ind in range(int(control_dt/Ts)):
                quad_sim(t=globalTime+ind*Ts, Ts=Ts, quad=quad, ctrl=ctrl, wind=wind, desired=desired)
            globalTime += control_dt
            
            i += 1           
            t_all[i]             = globalTime
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
        pass
    
    # View Results
    # ---------------------------

    # utils.fullprint(sDes_traj_all[:,3:6])
    makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all)
    ani = sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    plt.show()
    pass
if __name__=='__main__':
    main()
