#!/bin/python3
# Fix for Raspberry Pi Zero: disable SVE detection to avoid prctl(PR_SVE_GET_VL) error
import os
import sys
# Force disable SVE detection before any imports
os.environ['CPUINFO_DISABLE_SVE'] = '1'
# Suppress cpuinfo errors
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning, message='.*cpuinfo.*')
warnings.filterwarnings('ignore', message='.*prctl.*')
import time
#from hardware_adapter import Hardware_Adapter
#from virtual_tracker import Virtual_Tracker
from common import *
import numpy as np
# from config_parser import Config_Parser
from control import *
# from tracker import Tracker
import pymap3d
# import cv2
import sys
import zmq
import zmqWrapper
import zmqTopics
import pickle
import struct
import json
import argparse
# from Filter.AllInOneKalman import AllInOneKalman
# from virtual_tracker import Tracker

# from controlGeom import GeometricController
# from controlGeomAdaptive import AdaptiveGeometricController
# from controlBrescianini import BrescianiniController
# from controlKooijman import KooijmanController
from controlVelocityPID import VelocityPIDController
from controlVelocityRL import VelocityRLController
from extraControllers.controlAccelerationPID import AccelerationPIDController
# from controlJaeyoung import JaeyoungController
from trajectories import horz_circle, pos_point, vel_point, lineConstVel, vert_circle, command_Lissajous

# sys.path.append('../')
# try:
#     import src.zmqTopics as zmqTopics
#     import src.zmqWrapper as zmqWrapper
#     import src.mps as mps
# except:
#     import zmqTopics
#     import zmqWrapper
#     import mps
REAL_TIME = True
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

# Default configuration parameters (matching SystemManager.h)
DEFAULT_LOOP_PERIOD = 0.03
DEFAULT_CURRENT_TIME = -1.0
DEFAULT_DRONE_MASS = 0.55
DEFAULT_YAW_COMMAND_FACTOR = 1.0
DEFAULT_MAXIMAL_VELOCITY = 0.75
DEFAULT_DESCENT_VELOCITY = 10.0
DEFAULT_TARGET_VELOCITY = 7.5
DEFAULT_CIRCLE_RADIUS = 5.0
DEFAULT_WAYPOINT_REACH_THRESHOLD = 0.5

# Default Vector3d components
DEFAULT_HEADING_DIR_NED_X = -1.0
DEFAULT_HEADING_DIR_NED_Y = 1.0
DEFAULT_HEADING_DIR_NED_Z = 0.0

DEFAULT_DESIRED_HEADING_DIR_NED_X = -1.0
DEFAULT_DESIRED_HEADING_DIR_NED_Y = 1.0
DEFAULT_DESIRED_HEADING_DIR_NED_Z = 0.0

# Default boolean values
DEFAULT_TERMINAL_HOMING_ALLOWED = True
DEFAULT_RATE_CONTROL_ENABLED = False
#################################################################################################################
class System_Manager():
    def __init__(self, log_dir, sim_object=None, external_imu=None, use_usb_for_mavlink=False, currentTime=None, config_dict=None):
        print("System_Manager: Initializing...")
        self._overall_start = time.monotonic() if currentTime is None else currentTime
        self._log_dir = log_dir
        self._prev_los_ned_dir = np.zeros(3)
        self._prev_pos_ned = np.zeros(3)
        self._prev_imu_ts = 0
        self._prev_ts = self._overall_start

        self.message_count = 0
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
        
        # Initialize controller parameter file paths (will be set from config if provided)
        self.primaryControllerParamsFile = None
        self.secondaryControllerParamsFile = None
        
        # Load configuration from JSON if provided, otherwise use defaults
        if config_dict is not None:
            self._load_config_from_dict(config_dict)
        else:
            # Use default parameters (using constants matching C++ version)
            ##############################
            # start scenario definitions #
            ##############################
            self.referencePoint = np.array([0, 0, 0])
            self.desiredHeadingDir_ned = np.array([DEFAULT_DESIRED_HEADING_DIR_NED_X, DEFAULT_DESIRED_HEADING_DIR_NED_Y, DEFAULT_DESIRED_HEADING_DIR_NED_Z])
            
            self.missionType = MISSION_TYPE.WAYPOINT
            self.yawControlType = YAW_COMMAND.DEFINED_DIR
            
            self.yawCommandFactor = DEFAULT_YAW_COMMAND_FACTOR
            self.maximalVelocity = DEFAULT_MAXIMAL_VELOCITY
            self.descentVelocity = DEFAULT_DESCENT_VELOCITY
            self.targetVelocity = DEFAULT_TARGET_VELOCITY
            self.originOffset_frd = np.array([0, 0, 0])
            self.terminalHomingAlowed = DEFAULT_TERMINAL_HOMING_ALLOWED
            self.circleRadius = DEFAULT_CIRCLE_RADIUS
            self.controllerType = CONTROLLER_TYPE.VELOCITYRL
            self.yawCommandType = YAW_COMMAND_TYPE.RATE
            self.rateControlEnabled = DEFAULT_RATE_CONTROL_ENABLED
            self.loop_period = DEFAULT_LOOP_PERIOD
        
        # Initialize controllers based on controller type (preserve original code)
        if self.controllerType == CONTROLLER_TYPE.VELOCITYRL:        
            # Use parameter file if available (from JSON config), otherwise use defaults
            aux_params_file = self.secondaryControllerParamsFile if hasattr(self, 'secondaryControllerParamsFile') and self.secondaryControllerParamsFile else None
            main_params_file = self.primaryControllerParamsFile if hasattr(self, 'primaryControllerParamsFile') and self.primaryControllerParamsFile else None
            self._controlAux = Control(self._log_dir, controller=VelocityPIDController(mass=self.dronemass, currentTime=self._overall_start, params_file=aux_params_file), maximalVelocity=self.maximalVelocity)
            # self._controlMain = Control(self._log_dir, controller=VelocityPIDController(mass=self.dronemass), maximalVelocity=self.maximalVelocity)
            self._controlMain = Control(self._log_dir, controller=VelocityRLController(mass=self.dronemass, maximalVelocity=self.maximalVelocity, currentTime=self._overall_start, params_file=main_params_file)) 
        elif self.controllerType == CONTROLLER_TYPE.VELOCITYPID:
            # Use parameter file if available (from JSON config), otherwise use defaults
            main_params_file = self.primaryControllerParamsFile if hasattr(self, 'primaryControllerParamsFile') and self.primaryControllerParamsFile else None
            self._controlMain = Control(self._log_dir, controller=VelocityPIDController(mass=self.dronemass, yawCommandType=self.yawCommandType, params_file=main_params_file), maximalVelocity=self.maximalVelocity)
        elif self.controllerType == CONTROLLER_TYPE.ACCELERATIONPID:
            self._controlMain = Control(self._log_dir, controller=AccelerationPIDController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=AccelerationPIDController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=GeometricController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=AdaptiveGeometricController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=JaeyoungController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=BrescianiniController(mass=self.dronemass))
        # self._control = Control(self._log_dir, controller=KooijmanController(mass=self.dronemass))
        
        ###############################
        # end of scenario definitions #
        ###############################
        
        # Create persistent subscription socket for flight data
        # Use CONFLATE to keep only the latest message (now that we use single-part messages)
        self.subsSock = zmqWrapper.context.socket(zmq.SUB)
        self.subsSock.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message (works with single-part)
        # Set receive timeout
        self.subsSock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        self.subsSock.connect(f"tcp://127.0.0.1:{zmqTopics.topicMavlinkPort}")
        self.subsSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicMavlinkFlightData)
        print(f"System_Manager: Subscribed to topic: {zmqTopics.topicMavlinkFlightData} on port: {zmqTopics.topicMavlinkPort}")
        # Give socket time to connect (important for ZMQ PUB/SUB pattern)
        # NOTE: In ZMQ PUB/SUB, the publisher should bind BEFORE the subscriber connects
        # If hardware_adapter starts after system_manager, some initial messages may be lost
        time.sleep(0.2)

        self._currentData = Flight_Data()
        self.trackData = None
        # Use '*' to bind to all interfaces (standard for PUB sockets)
        self.pubSock = zmqWrapper.publisher(zmqTopics.topicGuidenceCmdPort, ip='*')
        print(f"System_Manager: Publishing commands on port: {zmqTopics.topicGuidenceCmdPort}")
        # Give publisher time to bind (ZMQ PUB sockets need time to establish)
        time.sleep(0.5)  # Increased to 500ms for PUB socket to be ready
        
        ##self._control.reset(thrust=self._hardware_adapter.get_current_thrust(), yaw=self._hardware_adapter.get_current_yaw())
        self._current_pos_lla = LLA(timestamp=0, lla=np.zeros(3))

        self.tic = time.monotonic()
        self.dest_pos_ned = None

        # self.allInOneKalman = AllInOneKalman()

#################################################################################################################
    def _get_vehicle_config_file(self, config_dir):
        system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        vehicle_config_file_name =  system_config_parser.get_value("vehicle_config_file", default_value="")
        vehicle_config_path = os.path.join(config_dir,"vehicle", vehicle_config_file_name)
        return vehicle_config_path

#################################################################################################################
    def _load_config_from_dict(self, config_dict):
        """
        Load configuration parameters from a dictionary (typically loaded from JSON).
        
        Args:
            config_dict: Dictionary containing configuration parameters
        """
        try:
            # Load values from JSON config
            factor = 1
            self.referencePoint = np.array(config_dict.get('referencePoint', [0, 0, 0]))
            self.desiredHeadingDir_ned = np.array(config_dict.get('desiredHeadingDir_ned', [DEFAULT_DESIRED_HEADING_DIR_NED_X, DEFAULT_DESIRED_HEADING_DIR_NED_Y, DEFAULT_DESIRED_HEADING_DIR_NED_Z]))
            # self.pointList = np.array([[0, 10*factor*0, 0], [0, 10*factor, 0]])
            if 'waypointList' in config_dict:
                self.pointList = np.array(config_dict['waypointList'])
        
            # Parse mission type from string
            mission_type_str = config_dict.get('missionType', 'CIRCLE')
            mission_type_map = {
                'NONE': MISSION_TYPE.NONE,
                'WAYPOINT': MISSION_TYPE.WAYPOINT,
                'VELOCITY': MISSION_TYPE.VELOCITY,
                'CIRCLE': MISSION_TYPE.CIRCLE,
                'LISSAJOUS': MISSION_TYPE.LISSAJOUS,
                'TRACKER': MISSION_TYPE.TRACKER,
                'SECTION': MISSION_TYPE.SECTION,
                'SPINNING': MISSION_TYPE.SPINNING
            }
            self.missionType = mission_type_map.get(mission_type_str.upper(), MISSION_TYPE.CIRCLE)
            
            # Parse yaw control type from string
            yaw_control_str = config_dict.get('yawControlType', 'DEFINED_DIR')
            yaw_control_map = {
                'NO_CONTROL': YAW_COMMAND.NO_CONTROL,
                'DEFINED_DIR': YAW_COMMAND.DEFINED_DIR,
                'HOLD_CUR_DIR': YAW_COMMAND.HOLD_CUR_DIR,
                'VELOCITY_DIR': YAW_COMMAND.VELOCITY_DIR,
                'CAMERA_DIR': YAW_COMMAND.CAMERA_DIR
            }
            self.yawControlType = yaw_control_map.get(yaw_control_str.upper(), YAW_COMMAND.DEFINED_DIR)
            
            self.yawCommandFactor = config_dict.get('yawCommandFactor', DEFAULT_YAW_COMMAND_FACTOR)
            self.maximalVelocity = config_dict.get('maximalVelocity', DEFAULT_MAXIMAL_VELOCITY)
            self.descentVelocity = config_dict.get('descentVelocity', DEFAULT_DESCENT_VELOCITY)
            self.targetVelocity = config_dict.get('targetVelocity', DEFAULT_TARGET_VELOCITY)
            self.originOffset_frd = np.array(config_dict.get('originOffset_frd', [0, 0, 0]))
            self.terminalHomingAlowed = config_dict.get('terminalHomingAlowed', DEFAULT_TERMINAL_HOMING_ALLOWED)
            self.circleRadius = config_dict.get('circleRadius', DEFAULT_CIRCLE_RADIUS)
            self.loop_period = config_dict.get('loop_period', DEFAULT_LOOP_PERIOD)
            
            # Parse controller type from string
            controller_type_str = config_dict.get('primaryControllerType', 'VELOCITYRL')
            controller_type_map = {
                'VELOCITYPID': CONTROLLER_TYPE.VELOCITYPID,
                'VELOCITYRL': CONTROLLER_TYPE.VELOCITYRL,
                'ACCELERATIONPID': CONTROLLER_TYPE.ACCELERATIONPID,
                'GEOMETRIC': CONTROLLER_TYPE.GEOMETRIC,
                'ADAPTIVEGEOMETRIC': CONTROLLER_TYPE.ADAPTIVEGEOMETRIC,
                'JAEYOUNG': CONTROLLER_TYPE.JAEYOUNG,
                'BRESCIANINI': CONTROLLER_TYPE.BRESCIANINI,
                'KOOIJMAN': CONTROLLER_TYPE.KOOIJMAN
            }
            self.controllerType = controller_type_map.get(controller_type_str.upper(), CONTROLLER_TYPE.VELOCITYRL)
            
            # Parse yaw command type from string
            yaw_cmd_type_str = config_dict.get('yawCommandType', 'RATE')
            yaw_cmd_type_map = {
                'NONE': YAW_COMMAND_TYPE.NONE,
                'ANGLE': YAW_COMMAND_TYPE.ANGLE,
                'RATE': YAW_COMMAND_TYPE.RATE
            }
            self.yawCommandType = yaw_cmd_type_map.get(yaw_cmd_type_str.upper(), YAW_COMMAND_TYPE.RATE)
            
            # Update log_dir if provided in config
            if 'log_dir' in config_dict:
                self._log_dir = config_dict['log_dir']
            
            # Update dronemass if provided
            if 'dronemass' in config_dict:
                self.dronemass = config_dict['dronemass']
            
            self.rateControlEnabled = config_dict.get('rateControlEnabled', DEFAULT_RATE_CONTROL_ENABLED)
            
            # Store controller parameter file paths for later use in controller initialization
            self.primaryControllerParamsFile = config_dict.get('primaryControllerParamsFile', None)
            self.secondaryControllerParamsFile = config_dict.get('secondaryControllerParamsFile', None)
        except (TypeError, ValueError, KeyError) as e:
            print(f"Error: Failed to parse parameter from config file")
            print(f"  Parameter error: {e}")
            print("Exiting program.")
            sys.exit(1)
        except Exception as e:
            print(f"Error: Unexpected error while loading configuration parameters")
            print(f"  Error: {e}")
            print("Exiting program.")
            sys.exit(1)

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
                # Only print warning occasionally to avoid spam
                if not hasattr(self, '_last_missing_data_warning') or (time.monotonic() - self._last_missing_data_warning) > 2.0:
                    print(f'System_Manager: -return-0- (Missing flight data. Gathered flags: quat={self._currentData.gathered.get("quat_ned_bodyfrd", False)}, pos={self._currentData.gathered.get("pos_ned_m", False)}, imu={self._currentData.gathered.get("imu_ned", False)})')
                    print(f'  System_Manager: Cannot send commands without flight data. Check if hardware_adapter is publishing on port {zmqTopics.topicMavlinkPort}')
                    print(f'  System_Manager: This is why hardware_adapter is not receiving commands - system_manager is waiting for flight data first')
                    self._last_missing_data_warning = time.monotonic()
                return
        else:
            self._currentData = flight_Data
        
        # Set desired state based on mission type
        controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts = self.setDesiredState(curTime)
        
        # Generate command from controller
        cmd_result = self.generateCommand(trajDest, controlType, current_ts, counter, quat_ned_bodyfrd)
        if cmd_result is None:
            # Command validation failed, return empty message
            return {}
        
        command, rpyRate_cmd, quat_ned_desbodyfrd_cmd, yawCmd, yawCmdRate = cmd_result
        
        # Check if command is all zeros (might indicate an issue)
        command_array = np.array(command)
        if np.allclose(command_array, 0.0):
            # This is actually valid - zero velocity command is okay
            pass
        
        # Initialize debug counters if needed
        if not hasattr(self, '_cmd_send_count'):
            self._cmd_send_count = 0
            self._last_cmd_send_debug_time = time.monotonic()
        
        # Increment message count for all command types
        self.message_count += 1
        
        # Publish command to hardware adapter
        msg = self.publishCommand(command, yawCmd, yawCmdRate, rpyRate_cmd, quat_ned_desbodyfrd_cmd)

        # Monitoring output
        monitorTime=1
        if time.monotonic() - self.tic >= monitorTime:
            self.tic = time.monotonic()
            missionPoint = self.holdonPos_ned + self.referencePoint
            deltaPos_ned = missionPoint
            if quat_ned_bodyfrd is not None:
                deltaPos_ned = missionPoint - pos_ned
                deltaPos_frd = quat_ned_bodyfrd.inv().rotate_vec(deltaPos_ned)
            print('timestamp: ', self._currentData.timestamp, '--->referencePoint: %.3f %.3f %.3f, pos_ned:  %.3f %.3f %.3f '%( 
                self.referencePoint[0], self.referencePoint[1], self.referencePoint[2],
                pos_ned[0], pos_ned[1], pos_ned[2])+str(self.homingStage)+                
                " Command: + %.3f %.3f %.3f"%(command[0], command[1], command[2])+
                " yawCmd: %.3f"%(yawCmd)+
                " yawRateCmd: %.3f"%(rpyRate_cmd[2])+
                " deltaPos_frd: %.3f %.3f %.3f"%(deltaPos_frd[0], deltaPos_frd[1], deltaPos_frd[2]))
            
            try:
                print('<<--', np.linalg.norm(missionPoint[0:2]-pos_ned[0:2])+" timestamp:"+str(self._currentData.timestamp))
            except:
                pass
        
        if log_data and self._input_logger is not None:
            trajDest_pos_ned = trajDest[0]
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
    def setDesiredState(self, curTime):
        """
        Set desired state based on mission type and current flight data.
        
        Args:
            curTime: Current timestamp
            
        Returns:
            Tuple of (controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts)
            where:
                controlType: Control type tuple (pos_control, vel_control, yaw_control)
                trajDest: Tuple of (trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned)
                pos_ned: Current position in NED frame
                quat_ned_bodyfrd: Current quaternion from body FRD to NED
                current_ts: Current timestamp
        """
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
            if np.linalg.norm(self._currentData.pos_ned_m.vel_ned[0:2])>0.1:
                self.heading_dir_ned = deepcopy(self._currentData.pos_ned_m.vel_ned)
                self.heading_dir_ned[2] = 0; self.heading_dir_ned = unitVec(self.heading_dir_ned)
        elif self.yawControlType == YAW_COMMAND.CAMERA_DIR:
            self.heading_dir_ned = unitVec(np.array([self.los_ned_dir[0], self.los_ned_dir[1], 0]))
        elif self.yawControlType == YAW_COMMAND.DEFINED_DIR:
            self.heading_dir_ned = deepcopy(self.desiredHeadingDir_ned)
            
        missionPoint = self.holdonPos_ned + self.referencePoint;   
        if self.missionType == MISSION_TYPE.WAYPOINT:
            desired_trajectory = pos_point(missionPoint=[missionPoint], missionAttitudeDirection=self.heading_dir_ned, start_time=self._overall_start)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
         
        elif self.missionType == MISSION_TYPE.VELOCITY:
            missionVelocity = unitVec(missionPoint-self.holdonPos_ned)*self.maximalVelocity
            desired_trajectory = vel_point(missionVelocity=missionVelocity, missionAttitudeDirection=self.heading_dir_ned, start_time=self._overall_start)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.CIRCLE: 
            desired_trajectory = horz_circle(center=missionPoint, radius=self.circleRadius, Vel=self.targetVelocity, missionAttitudeDirection=self.heading_dir_ned, start_time=self._overall_start)   # bui
            # desired_trajectory = horz_circle(center = np.array([-10,20,0]), radius=10, start_time=self._overall_start)    # corner ok, bui fades away
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.LISSAJOUS:
            desired_trajectory = command_Lissajous(start_time=self._overall_start)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.SECTION:
            startPoint = deepcopy(self.holdonPos_ned); endPoint = deepcopy(missionPoint)
            desired_trajectory = lineConstVel(startPoint=startPoint, endPoint=endPoint, startTime=self.holdonTime, 
                                                speed=self.maximalVelocity, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]           
               
        trajDest_pos_ned = np.array(self.dest_pos_ned);  trajDest_pos_ned[2] = self._currentData.pos_ned_m.ned[2] if self.destHeight is None else self.destHeight
        trajDest_vel_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][1]
        trajDest_acc_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][2]
        
        trajDest = (trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned)
        
        return (controlType, trajDest, pos_ned, quat_ned_bodyfrd, current_ts)
    
    #################################################################################################################
    def generateCommand(self, trajDest, controlType, current_ts, counter, quat_ned_bodyfrd):
        """
        Generate command from controller and process it.
        
        Args:
            trajDest: Tuple of (trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned)
            controlType: Control type tuple (pos_control, vel_control, yaw_control)
            current_ts: Current timestamp
            counter: Step counter
            quat_ned_bodyfrd: Current quaternion from body FRD to NED
            
        Returns:
            Tuple of (command, rpyRate_cmd, quat_ned_desbodyfrd_cmd, yawCmd, yawCmdRate) on success,
            None on validation failure
        """
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
        
        yawCmd = np.nan;         yawCmdRate = np.nan
        if self.yawCommandType == YAW_COMMAND_TYPE.ANGLE:
            yawCmd=np.arctan2(forward_dir_frd[1], forward_dir_frd[0])
            if self.yawControlType == YAW_COMMAND.NO_CONTROL:
                yawCmd = self._currentData.heading
            else:
                heading_dir_ned = self.heading_dir_ned 
                yawCmd = np.arctan2(heading_dir_ned[1], heading_dir_ned[0])
        elif self.yawCommandType == YAW_COMMAND_TYPE.RATE:
            yawCmdRate = rpyRate_cmd[2]
        
        # Validate command before sending
        if command is None:
            if not hasattr(self, '_last_none_cmd_warning') or (time.monotonic() - self._last_none_cmd_warning) > 2.0:
                print("Warning: command is None, skipping send")
                self._last_none_cmd_warning = time.monotonic()
            return None
        if not isinstance(command, (list, np.ndarray)) or len(command) != 3:
            if not hasattr(self, '_last_invalid_cmd_warning') or (time.monotonic() - self._last_invalid_cmd_warning) > 2.0:
                print(f"Warning: Invalid command format. Expected array of length 3, got: {type(command)}, value: {command}")
                self._last_invalid_cmd_warning = time.monotonic()
            return None
        
        return (command, rpyRate_cmd, quat_ned_desbodyfrd_cmd, yawCmd, yawCmdRate)
            
###############################################################################################################################################
    def publishCommand(self, command, yawCmd, yawCmdRate, rpyRate_cmd, quat_ned_desbodyfrd_cmd):
        """
        Publish command to hardware adapter based on controller type.
        
        Args:
            command: Command array (velocity, acceleration, or thrust)
            yawCmd: Yaw angle command (or NaN if not used)
            yawCmdRate: Yaw rate command (or NaN if not used)
            rpyRate_cmd: Roll, pitch, yaw rate command array
            quat_ned_desbodyfrd_cmd: Desired quaternion from body FRD to NED
            
        Returns:
            Dictionary with command information and metadata
        """
        # Initialize msg dict for return value (will be populated based on controller type)
        msg = {}
        
        if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYPID:
            try:
                # Serialize to binary format for C code
                cmd_data = self._serialize_vel_cmd(command, yawCmd, yawCmdRate)
                # Send as multipart message (topic + data) for proper ZMQ subscription filtering
                self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdVelNed, cmd_data])
                self._cmd_send_count += 1
                # Debug: Print first few commands sent
                #if self._cmd_send_count <= 5:
                #    print(f"System_Manager: Sent velocity command #{self._cmd_send_count}: vel={command}, yaw={yawCmd}, yaw_rate={yawCmdRate}")
                # Populate msg for return value
                msg = { 'ts': time.monotonic(), 'velCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':yawCmdRate, 'message_count':self.message_count, 'message_ts':time.monotonic()}
            except Exception as e:
                print(f"System_Manager: ERROR sending velocity command: {e}")
                import traceback
                traceback.print_exc()
        elif self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYRL:
            command_ned = command
            try:
                # Serialize to binary format for C code
                cmd_data = self._serialize_vel_cmd(command_ned, yawCmd, yawCmdRate*self.yawCommandFactor)
                # Send as multipart message (topic + data) for proper ZMQ subscription filtering
                self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdVelNed, cmd_data])
                self._cmd_send_count += 1
                # Debug: Print first few commands sent
                #if self._cmd_send_count <= 5:
                #    print(f"System_Manager: Sent velocity RL command #{self._cmd_send_count}: vel={command_ned}, yaw={yawCmd}, yaw_rate={yawCmdRate*self.yawCommandFactor}")
                # Populate msg for return value
                msg = { 'ts': time.monotonic(), 'velCmd':command_ned, 'yawCmd':yawCmd, 'yawRateCmd':yawCmdRate*self.yawCommandFactor, 'message_count':self.message_count, 'message_ts':time.monotonic()}
            except Exception as e:
                print(f"System_Manager: ERROR sending velocity RL command: {e}")
                import traceback
                traceback.print_exc()
        
        elif self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.ACCELERATIONPID:
            msg = { 'ts': time.monotonic(), 'accCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':yawCmdRate, 'message_count':self.message_count, 'message_ts':time.monotonic()}
            # Send as multipart message (topic + data) for proper ZMQ subscription filtering
            try:
                self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAcc, pickle.dumps(msg)])
                self._cmd_send_count += 1
            except Exception as e:
                print(f"Error sending acceleration command: {e}")
        else:
            try:
                # Serialize to binary format for C code
                # For attitude commands, command is typically a single thrust value or array
                # Extract thrust value - if it's an array, use the appropriate element, otherwise use directly
                if isinstance(command, (list, np.ndarray)) and len(command) >= 1:
                    # If it's an array, use the first element (or last if it's [x, y, z] format)
                    thrust_val = float(command[0]) if len(command) == 1 else float(command[2] if len(command) >= 3 else command[0])
                elif isinstance(command, (int, float, np.number)):
                    thrust_val = float(command)
                else:
                    thrust_val = 0.0
                cmd_data = self._serialize_attitude_cmd(thrust_val, rpyRate_cmd, quat_ned_desbodyfrd_cmd, self.rateControlEnabled)
                # Send as multipart message (topic + data) for proper ZMQ subscription filtering
                self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAttitude, cmd_data])
                self._cmd_send_count += 1
                # Populate msg for return value
                msg = { 'ts': time.monotonic(), 'thrustCmd':thrust_val, 'rpyRateCmd':rpyRate_cmd,
                       'quatNedDesBodyFrdCmd':[quat_ned_desbodyfrd_cmd.w, quat_ned_desbodyfrd_cmd.x, quat_ned_desbodyfrd_cmd.y, quat_ned_desbodyfrd_cmd.z],
                       'isRate':self.rateControlEnabled, 'message_count':self.message_count, 'message_ts':time.monotonic()}
            except Exception as e:
                print(f"Error sending attitude command: {e}")
                import traceback
                traceback.print_exc()
        
        # Debug output every 2 seconds (after all controller type checks)
        if not hasattr(self, '_last_cmd_send_debug_time'):
            self._last_cmd_send_debug_time = time.monotonic()
        current_time = time.monotonic()
        if current_time - self._last_cmd_send_debug_time > 2.0:
            controller_type_str = "VELOCITYPID" if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYPID else \
                                  "VELOCITYRL" if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.VELOCITYRL else \
                                  "ACCELERATIONPID" if self._controlMain.controlnode.controllerType == CONTROLLER_TYPE.ACCELERATIONPID else "ATTITUDE"
            print(f"System_manager: Sent {self._cmd_send_count} commands in last 2s. Controller: {controller_type_str}, message_count: {self.message_count}")
            self._cmd_send_count = 0
            self._last_cmd_send_debug_time = current_time
        
        return msg

#################################################################################################################
    def getQuadBodyfrdCamera(self, nedYangle):
        yawCorrection = 0 #np.pi/2
        mat_cam_bodyfrd =  rotZ(-np.pi/2+yawCorrection) @ rotY(np.pi/2) @ rotY(-nedYangle) #   rotZned * rotYned * rotYned
        return Quaternion.from_matrix(mat=mat_cam_bodyfrd.T) 

#################################################################################################################
    def _serialize_vel_cmd(self, vel_cmd, yaw_cmd, yaw_rate_cmd):
        """
        Serialize velocity command to binary format for C code.
        Format: magic (4 bytes), version (4 bytes), vel[3] (24 bytes), yaw (8 bytes), 
                yaw_rate (8 bytes), yaw_valid (1 byte), yaw_rate_valid (1 byte), padding (2 bytes)
        """
        # Magic number: "VELC" = 0x56454C43
        magic = 0x56454C43
        version = 1
        
        # Check if yaw/yaw_rate are valid (not NaN)
        yaw_valid = 1 if not np.isnan(yaw_cmd) else 0
        yaw_rate_valid = 1 if not np.isnan(yaw_rate_cmd) else 0
        
        # Use NaN-safe values
        yaw_val = yaw_cmd if not np.isnan(yaw_cmd) else 0.0
        yaw_rate_val = yaw_rate_cmd if not np.isnan(yaw_rate_cmd) else 0.0
        
        # Pack as little-endian binary: '<II' (magic, version), '<3d' (vel), '<2d' (yaw, yaw_rate), '<2B' (flags)
        return struct.pack('<II', magic, version) + \
               struct.pack('<3d', vel_cmd[0], vel_cmd[1], vel_cmd[2]) + \
               struct.pack('<2d', yaw_val, yaw_rate_val) + \
               struct.pack('<2B', yaw_valid, yaw_rate_valid) + \
               b'\x00\x00'  # padding
    
#################################################################################################################
    def _serialize_attitude_cmd(self, thrust_cmd, rpy_rate_cmd, quat_cmd, is_rate):
        """
        Serialize attitude command to binary format for C code.
        Format: magic (4 bytes), version (4 bytes), thrust (8 bytes), rpy_rate[3] (24 bytes), 
                quat[4] (32 bytes), is_rate (1 byte), padding (3 bytes)
        """
        # Magic number: "ATTI" = 0x41545449
        magic = 0x41545449
        version = 1
        
        # Pack as little-endian binary: '<II' (magic, version), '<d' (thrust), '<3d' (rpy_rate), '<4d' (quat), '<B' (is_rate)
        return struct.pack('<II', magic, version) + \
               struct.pack('<d', thrust_cmd) + \
               struct.pack('<3d', rpy_rate_cmd[0], rpy_rate_cmd[1], rpy_rate_cmd[2]) + \
               struct.pack('<4d', quat_cmd.w, quat_cmd.x, quat_cmd.y, quat_cmd.z) + \
               struct.pack('<B', 1 if is_rate else 0) + \
               b'\x00\x00\x00'  # padding

#################################################################################################################
    def _deserialize_flight_data(self, data_bytes):
        """
        Deserialize flight data from binary format sent by hardware_adapter.
        The C code sends a binary format that we deserialize here.
        Format: magic (4 bytes), version (4 bytes), then all numeric fields.
        """
        try:
            # Try to deserialize as pickle first (for backward compatibility)
            try:
                from common import Flight_Data
                data = pickle.loads(data_bytes)
                # Validate that pickle returned a Flight_Data object
                if isinstance(data, Flight_Data):
                    return data
                else:
                    # Pickle succeeded but returned wrong type, try binary format
                    raise ValueError(f"Pickle deserialization returned {type(data)}, expected Flight_Data")
            except (pickle.UnpicklingError, ValueError, TypeError):
                # Not pickle format or wrong type, try binary format
                pass
            
            # Deserialize binary format
            if len(data_bytes) < 8:
                raise ValueError("Data too short")
            
            # Read magic and version
            magic, version = struct.unpack('<II', data_bytes[0:8])
            if magic != 0x464C4947:  # "FLIG"
                raise ValueError(f"Invalid magic number: {hex(magic)}")
            if version != 1:
                raise ValueError(f"Unsupported version: {version}")
            
            offset = 8
            
            # Create Flight_Data object
            from common import Flight_Data, Quaternion, NED, LLA, Altitude, Imu, FLIGHT_MODE
            
            data = Flight_Data()
            
            # Read message_count (uint32_t)
            data.message_count = struct.unpack('<I', data_bytes[offset:offset+4])[0]
            offset += 4
            
            # Read all doubles (63 doubles total: 19 scalars + 5 quaternion + 4 altitude + 7 imu_raw + 7 imu_ned + 7 pos_ned + 4 raw_lla + 4 filt_lla + 3 rpy_rates + 3 rpy)
            # Format: '<63d' for 63 doubles in little-endian
            doubles = struct.unpack('<63d', data_bytes[offset:offset+504])
            offset += 504
            idx = 0
            
            data.quat_ts = doubles[idx]; idx += 1
            data.imu_ts = doubles[idx]; idx += 1
            data.timestamp = doubles[idx]; idx += 1
            data.local_ts = doubles[idx]; idx += 1
            data.temperature = doubles[idx]; idx += 1
            data.amsl_m = doubles[idx]; idx += 1
            data.local_m = doubles[idx]; idx += 1
            data.monotonic_m = doubles[idx]; idx += 1
            data.relative_m = doubles[idx]; idx += 1
            data.terrain_m = doubles[idx]; idx += 1
            data.bottom_clearance_m = doubles[idx]; idx += 1
            data.pressure = doubles[idx]; idx += 1
            data.absolute_press_hpa = doubles[idx]; idx += 1
            data.differential_press_hpa = doubles[idx]; idx += 1
            data.signal_strength_percent = doubles[idx]; idx += 1
            data.throttle = doubles[idx]; idx += 1
            data.heading = doubles[idx]; idx += 1
            data.groundspeed = doubles[idx]; idx += 1
            data.current_thrust = doubles[idx]; idx += 1
            
            # Quaternion
            data.quat_ned_bodyfrd = Quaternion(x=doubles[idx], y=doubles[idx+1], z=doubles[idx+2], w=doubles[idx+3])  # x, y, z, w
            data.quat_ned_bodyfrd.timestamp = doubles[idx+4]
            idx += 5
            
            # Altitude
            data.altitude_m = Altitude(doubles[idx], doubles[idx+1])  # amsl, relative
            data.altitude_m.vertical_speed_estimate = doubles[idx+2]
            data.altitude_m.timestamp = doubles[idx+3]
            idx += 4
            
            # IMU raw FRD
            data.imu_raw_frd = Imu(doubles[idx+6], np.array([doubles[idx], doubles[idx+1], doubles[idx+2]]), 
                                   np.array([doubles[idx+3], doubles[idx+4], doubles[idx+5]]))
            idx += 7
            
            # IMU NED
            data.imu_ned = Imu(doubles[idx+6], np.array([doubles[idx], doubles[idx+1], doubles[idx+2]]), 
                              np.array([doubles[idx+3], doubles[idx+4], doubles[idx+5]]))
            idx += 7
            
            # Position NED
            data.pos_ned_m = NED(ned=np.array([doubles[idx], doubles[idx+1], doubles[idx+2]]),
                                vel_ned=np.array([doubles[idx+3], doubles[idx+4], doubles[idx+5]]),
                                timestamp=doubles[idx+6])
            idx += 7
            
            # Raw position LLA
            data.raw_pos_lla_deg = LLA(timestamp=doubles[idx+3], lla=np.array([doubles[idx], doubles[idx+1], doubles[idx+2]]))
            idx += 4
            
            # Filtered position LLA
            data.filt_pos_lla_deg = LLA(timestamp=doubles[idx+3], lla=np.array([doubles[idx], doubles[idx+1], doubles[idx+2]]))
            idx += 4
            
            # RPY rates and RPY
            data.rpy_rates = np.array([doubles[idx], doubles[idx+1], doubles[idx+2]])
            data.rpy = np.array([doubles[idx+3], doubles[idx+4], doubles[idx+5]])
            idx += 6
            
            # Read uint32_t fields
            data.custom_mode_id = struct.unpack('<I', data_bytes[offset:offset+4])[0]
            offset += 4
            mode_val = struct.unpack('<I', data_bytes[offset:offset+4])[0]
            data.mode = FLIGHT_MODE(mode_val) if mode_val < len(list(FLIGHT_MODE)) else FLIGHT_MODE.UNKNOWN
            offset += 4
            
            # Read bools (packed as uint8_t)
            bools = struct.unpack('<B', data_bytes[offset:offset+1])[0]
            offset += 1
            data.is_armed = bool(bools & 1)
            data.offboardMode = bool(bools & 2)
            data.is_available = bool(bools & 4)
            data.was_available_once = bool(bools & 8)
            data.is_gyrometer_calibration_ok = bool(bools & 16)
            data.is_accelerometer_calibration_ok = bool(bools & 32)
            data.is_magnetometer_calibration_ok = bool(bools & 64)
            data.in_air = bool(bools & 128)
            
            # Read gathered flags (uint32_t bitfield)
            gathered_flags = struct.unpack('<I', data_bytes[offset:offset+4])[0]
            offset += 4
            data.gathered['euler_ned_bodyfrd'] = bool(gathered_flags & 1)
            data.gathered['quat_ned_bodyfrd'] = bool(gathered_flags & 2)
            data.gathered['pos_ned_m'] = bool(gathered_flags & 4)
            data.gathered['vel_ned_m'] = bool(gathered_flags & 8)
            data.gathered['imu_ned'] = bool(gathered_flags & 16)
            data.gathered['tracker_px'] = bool(gathered_flags & 32)
            data.gathered['rpy'] = bool(gathered_flags & 64)
            data.gathered['rpy_rates'] = bool(gathered_flags & 128)
            data.gathered['custom_mode_id'] = bool(gathered_flags & 256)
            data.gathered['mode'] = bool(gathered_flags & 512)
            data.gathered['relative_m'] = bool(gathered_flags & 1024)
            data.gathered['amsl_m'] = bool(gathered_flags & 2048)
            data.gathered['local_m'] = bool(gathered_flags & 4096)
            data.gathered['monotonic_m'] = bool(gathered_flags & 8192)
            data.gathered['terrain_m'] = bool(gathered_flags & 16384)
            data.gathered['bottom_clearance_m'] = bool(gathered_flags & 32768)
            data.gathered['absolute_press_hpa'] = bool(gathered_flags & 65536)
            data.gathered['differential_press_hpa'] = bool(gathered_flags & 131072)
            data.gathered['pressure'] = bool(gathered_flags & 262144)
            data.gathered['temperature'] = bool(gathered_flags & 524288)
            
            return data
            
        except Exception as e:
            # If deserialization fails, try pickle as fallback
            # But validate the result is a Flight_Data object
            try:
                from common import Flight_Data
                data = pickle.loads(data_bytes)
                if isinstance(data, Flight_Data):
                    return data
                else:
                    raise ValueError(f"Pickle fallback returned {type(data)}, expected Flight_Data")
            except:
                raise ValueError(f"Failed to deserialize flight data: {e}")

#################################################################################################################
    def gatherData(self):
        # Use the persistent socket created in __init__
        # With CONFLATE enabled, we only get the latest message
        # Receive single-part message with topic prefix
        ret = None
        # Try multiple times to catch messages (with CONFLATE, we only get the latest anyway)
        for attempt in range(3):
            try:
                ret = self.subsSock.recv(zmq.NOBLOCK)  # Single-part message
                break  # Got a message, exit loop
            except zmq.Again:
                # No message available - try again with small delay
                if attempt < 2:
                    time.sleep(0.0001)  # Small delay before retry
                    continue
                # No message after retries
                pass
            except Exception as e:
                if not hasattr(self, '_last_gather_error_time') or (time.monotonic() - self._last_gather_error_time) > 5.0:
                    print(f"Error in gatherData: {e}")
                    import traceback
                    traceback.print_exc()
                    self._last_gather_error_time = time.monotonic()
                break
        
        # Process the received message if we got one
        if ret is not None:
            try:
                # Strip topic prefix (needed for subscription filtering) before deserializing
                if ret.startswith(zmqTopics.topicMavlinkFlightData):
                    data_bytes = ret[len(zmqTopics.topicMavlinkFlightData):]
                    data = self._deserialize_flight_data(data_bytes)  # Deserialize using new format
                else:
                    # Unexpected message format
                    print(f"Warning: Received message without expected topic prefix")
                    return
                curTime = time.monotonic()
                # print(f"System_manager: message count {data.message_count}, message delay {curTime - data.local_ts}")
                # No need to check topic - we only subscribe to one topic (topicMavlinkFlightData)
                
                self._currentData = data# mavlink LOCAL_POSITION_NED      # Flight controller time
                # Set gathered flags to indicate we have received data
                self._currentData.gathered['quat_ned_bodyfrd'] = True
                self._currentData.gathered['pos_ned_m'] = True
                self._currentData.gathered['imu_ned'] = True
                                      
                #if (self._currentData.custom_mode_id == 50593792) or \ # and self._currentData.groundspeed<0.5) or  \
                #print(f"Current custom mode id: {self._currentData.custom_mode_id}")
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
            except Exception as e:
                if not hasattr(self, '_last_gather_error_time') or (time.monotonic() - self._last_gather_error_time) > 5.0:
                    print(f"Error processing flight data in gatherData: {e}")
                    import traceback
                    traceback.print_exc()
                    self._last_gather_error_time = time.monotonic()
        else:
            # No message received - warn occasionally
            if not hasattr(self, '_last_no_data_warning_time'):
                self._last_no_data_warning_time = time.monotonic()
            if time.monotonic() - self._last_no_data_warning_time > 5.0:
                print(f"System_manager: WARNING - No flight data received for 5s. Is hardware_adapter running and publishing on port {zmqTopics.topicMavlinkPort}?")
                self._last_no_data_warning_time = time.monotonic()


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
def quad_sim(t, Ts, quad, ctrl, wind, traj):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts


    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(quad=quad, Ts=Ts, traj=traj)

    return t
############################################################################################################################

def load_config_from_json(json_file_path):
    """
    Load configuration from JSON file, supporting // style comments.
    
    Args:
        json_file_path: Path to JSON configuration file
        
    Returns:
        Dictionary with configuration parameters, or None if file doesn't exist
    """
    import re
    try:
        with open(json_file_path, 'r') as f:
            content = f.read()
        
        # Remove // style comments (but preserve // inside strings)
        # This regex matches // followed by any characters until end of line,
        # but only if it's not inside a string (simplified approach)
        lines = content.split('\n')
        cleaned_lines = []
        in_string = False
        escape_next = False
        
        for line in lines:
            cleaned_line = ''
            i = 0
            while i < len(line):
                char = line[i]
                
                if escape_next:
                    cleaned_line += char
                    escape_next = False
                    i += 1
                    continue
                
                if char == '\\':
                    cleaned_line += char
                    escape_next = True
                    i += 1
                    continue
                
                if char == '"':
                    in_string = not in_string
                    cleaned_line += char
                    i += 1
                    continue
                
                # If we encounter // and we're not in a string, remove rest of line
                if not in_string and i < len(line) - 1 and line[i:i+2] == '//':
                    break
                
                cleaned_line += char
                i += 1
            
            cleaned_lines.append(cleaned_line)
        
        cleaned_content = '\n'.join(cleaned_lines)
        
        # Parse the cleaned JSON
        config = json.loads(cleaned_content)
        print(f"System_Manager: Loaded configuration from {json_file_path}")
        return config
    except FileNotFoundError:
        print(f"Error: Could not open config file: {json_file_path}")
        print("Exiting program.")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in config file: {json_file_path}")
        print(f"  JSON Error: {e}")
        print("Exiting program.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: Failed to load config file: {json_file_path}")
        print(f"  Error: {e}")
        print("Exiting program.")
        sys.exit(1)

############################################################################################################################

def main():
    import sys
    sys.stdout.flush()  # Ensure output is flushed immediately
    print("=" * 60)
    print("System_Manager starting...")
    print("=" * 60)
    sys.stdout.flush()
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='System Manager for quadcopter control')
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='Path to JSON configuration file (if not provided, uses default parameters)')
    parser.add_argument('config_file', nargs='?', type=str, default=None,
                        help='Path to JSON configuration file (positional argument, alternative to --config)')
    args = parser.parse_args()
    
    # Load configuration from JSON if provided (check both --config flag and positional argument)
    config_dict = None
    config_file_path = args.config or args.config_file
    if config_file_path:
        config_dict = load_config_from_json(config_file_path)
        # load_config_from_json now exits on error, so if we get here it succeeded
    
    if REAL_TIME:
        # Use log_dir from config if available, otherwise use defaults
        log_dir = config_dict.get('log_dir', '../logs/') if config_dict else '../logs/'
        current_time = config_dict.get('currentTime') if config_dict and 'currentTime' in config_dict else None
        # Only pass currentTime if it's not -1 (which seems to be a sentinel value)
        if current_time is not None and current_time < 0:
            current_time = None
        sysMgr = System_Manager(log_dir=log_dir, currentTime=current_time, config_dict=config_dict)
        print("System_Manager initialized. Starting main loop...")
        print("System_Manager: Waiting for flight data from hardware_adapter...")
        print("System_Manager: If you see '-return-0-' messages, hardware_adapter may not be publishing data")
        loop_period = config_dict.get('loop_period', DEFAULT_LOOP_PERIOD) if config_dict else DEFAULT_LOOP_PERIOD
        print(f"System_Manager: Main loop running at {1.0/loop_period:.1f}Hz (period: {loop_period}s)...")
        loop_count = 0
        next_loop_time = time.monotonic()
        while True:
            # time.sleep(0.0001)
            
            # Variable delay to maintain loop frequency
            sleep_duration = next_loop_time - time.monotonic()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            
            next_loop_time += loop_period
            
            startTime = time.monotonic()
            try:
                sysMgr.sys_manager_step()
                loop_count += 1
                # Print status every 1000 loops (every ~10 seconds at 100Hz)
                if loop_count % 1000 == 0:
                    print(f"System_Manager: Main loop running, iteration {loop_count}")
            except Exception as e:
                print(f"Error in sys_manager_step: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)  # Prevent tight error loop
            endTime = time.monotonic()
            # print("Computation Time",endTime-startTime)
    else:
        # Use log_dir from config if available, otherwise use defaults
        log_dir = config_dict.get('log_dir', '../logs/') if config_dict else '../logs/'
        current_time = config_dict.get('currentTime', 0) if config_dict else 0
        # Only use currentTime if it's >= 0, otherwise use 0
        if current_time < 0:
            current_time = 0
        sysMgr = System_Manager(log_dir=log_dir, currentTime=current_time, config_dict=config_dict)
        # Add the Simulation directory to the path (relative to this file's location)
        simulation_path = os.path.join(os.path.dirname(__file__), 'Quadcopter_SimCon', 'Simulation')
        sys.path.append(simulation_path)
        from quadFiles.quad import Quadcopter
        from trajectory import Trajectory
        from ctrl import Control, ControlType
        from utils.windModel import Wind
        from utils.display import makeFigures
        from utils.animation import sameAxisAnimation
        start_time = time.time()
    
        # Simulation Setup
        # --------------------------- 
        t=0
        Ti = 0
        Ts = 0.01
        control_dt = 0.1
        
        Tf = 20
        ifsave = 0
    
        # Choose trajectory settings
        # --------------------------- 
        trajSelect = np.zeros(3)

        # Initialize Quadcopter, Controller, Wind, Result Matrixes
        # ---------------------------
        quad = Quadcopter(Ti)
        traj = Trajectory(quad, trajSelect=trajSelect, ctrlType=ControlType.XY_VEL_Z_POS)
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
            traj.desiredState(t=globalTime, Ts=Ts, quad=quad, desired=desired)
            # if globalTime > 5:
            #     pass
            for ind in range(int(control_dt/Ts)):
                quad_sim(t=globalTime+ind*Ts, Ts=Ts, quad=quad, ctrl=ctrl, wind=wind, traj=traj)
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
    try:
        main()
    except KeyboardInterrupt:
        print("\nSystem_Manager: Shutting down...")
    except Exception as e:
        print(f"System_Manager: Fatal error: {e}")
        import traceback
        traceback.print_exc()
        raise
