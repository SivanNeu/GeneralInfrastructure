#!/bin/python3
import time
import zmq
import zmqTopics
import zmqWrapper
import pickle
import mps

import time
from common import Utils, Quaternion, Flight_Data, MAV_CMD, FLIGHT_MODE, Rate_Cmd, Logger, PX4_FLIGHT_STATE
from low_pass_filter import Low_Pass_Filter

import numpy as np

from pymavlink import mavutil
import multiprocessing
from multiprocessing import  Lock
from copy import deepcopy
import threading

import pymavlink

# from config_parser import Config_Parser
import os
#sudo nano /home/user/.local/bin/mavproxy.py
#mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
#find / -name "mavproxy.py" 2>/dev/null


mavlinkAddress = 'udp:127.0.0.1:14540'


MAVLINK_QUEUE_SIZE = 200
MAVLINK_RATE_HZ = 50
MUTEX_TIMEOUT_SEC = 1
TIME_BETWEEN_MODE_SET_ATTEMPTS = 1
ARM_LOOP_DELAY = 0.02
MAX_VERTICAL_VEL_JUMP_M_S = 3
USE_MAVLINK = True
DEVICE_STR_LIST = ["CubeBlack", "CubeOrange"]
RELEVANT_MAVLINK_MESSAGES = ['ALTITUDE', 
                             'ATTITUDE', 
                             'HIGHRES_IMU', 
                             'ATTITUDE_QUATERNION', 
                             'LOCAL_POSITION_NED', 
                             'GLOBAL_POSITION_INT', 
                             'ATTITUDE_TARGET', 
                             'CURRENT_MODE', 
                             'HEARTBEAT', 
                             'VFR_HUD', 
                             'DISTANCE_SENSOR', 
                             'OPTICAL_FLOW']

pubTopicsList = [
               [zmqTopics.topicMavlinkFlightData,         zmqTopics.topicMavlinkPort],
            ]

mpsDict = {}

sockPub = zmqWrapper.publisher(zmqTopics.topicMavlinkPort) #TODO: change to pubTopicsList

for topic in pubTopicsList:
    mpsDict[topic[0]] = mps.MPS(topic[0])

# Subscriber socket will be created in Hardware_Adapter.__init__ to ensure proper timing
subSock = None


#################################################################################################################
#################################################################################################################
#################################################################################################################
class Hardware_Adapter():
    def __init__(self, log_dir):
        self._log_dir = log_dir
        self._prev_alt_m = None
        self._alt_vel_count = 0
        self._prev_vel_vertical = 0
        self._prev_alt_ts = time.monotonic()
        self._vertical_speed_filter = Low_Pass_Filter(alpha=0.1, is_angle=False)
        self._altitude_filter = Low_Pass_Filter(alpha=0.3, is_angle=False) # was 0.3
        # system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        # vehicle_config_file_name = system_config_parser.get_value("vehicle_config_file", default_value="")
        # vehicle_data_parser = None
        # if(vehicle_config_file_name is not None):
        #     vehicle_data_parser = Config_Parser(path=os.path.join(config_dir,"vehicle", vehicle_config_file_name), save_copy=False)
        # if(vehicle_data_parser is None):
        #     print("config init failed in hardware adapter") 
        # self._connection_string = vehicle_data_parser.get_value("mavlink/connection_string", "")
        # self._baud_rate = float(vehicle_data_parser.get_value("mavlink/baud_rate", -1))
        # self._disable_offboard_control = vehicle_data_parser.get_value("control/disable_offboard_control", True)
        # if(self._disable_offboard_control):
        #     print("offboard control disabled")

        self.flight_data = {}
        self._current_airspeed = 0
        self._mavlink_logger = None #Logger("mavlink"+time.strftime("%Y%m%d_%H%M%S"), log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="TXT")

        # Initialize missing attributes
        self._mavlink_connected_to_usb = False
        self._disable_offboard_control = False

        success = self._init_mavlink()
        self._current_data = Flight_Data()
        self._init_success = True
        self._offboard_control_enabled = False
        if(not success):
            self._init_success = False
            print("mavlink failed to initialize")
            return
        
        # Thread synchronization
        self._data_lock = threading.Lock()
        self._running = True
        
        # Publishing frequency
        self._publish_freq_hz = 500
        self._publish_dt = 1.0 / self._publish_freq_hz
        
        # Create subscriber socket for commands (create here to ensure proper timing)
        # Wait a bit to ensure system_manager's publisher has time to bind
        time.sleep(0.2)
        global subSock
        # Create socket WITH CONFLATE for single-part messages (keeps only latest message)
        subSock = zmqWrapper.context.socket(zmq.SUB)
        subSock.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message (works with single-part)
        subSock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        # Subscribe to all command topics BEFORE connecting (important for ZMQ PUB/SUB)
        subSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicGuidenceCmdAttitude)
        subSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicGuidenceCmdVelNed)
        subSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicGuidenceCmdVelBody)
        subSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicGuidenceCmdAcc)
        subSock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicGuidanceCmdArm)
        # Connect AFTER subscribing (important for ZMQ PUB/SUB timing)
        subSock.connect(f"tcp://127.0.0.1:{zmqTopics.topicGuidenceCmdPort}")
        print(f"Hardware_adapter: Subscribed to commands on port: {zmqTopics.topicGuidenceCmdPort}")
        print(f"  Topics: {[zmqTopics.topicGuidenceCmdAttitude, zmqTopics.topicGuidenceCmdVelNed, zmqTopics.topicGuidenceCmdVelBody, zmqTopics.topicGuidenceCmdAcc, zmqTopics.topicGuidanceCmdArm]}")
        
        # Give ZMQ publisher socket time to bind and be ready
        # This prevents messages from being lost when subscribers connect
        time.sleep(0.1)
        
        # Start threads
        self._command_thread = threading.Thread(target=self._command_thread_func, daemon=True)
        self._data_thread = threading.Thread(target=self._data_thread_func, daemon=True)
        
        self._command_thread.start()
        self._data_thread.start()
        
        print("Hardware adapter threads started successfully")
        print(f"Publishing to topic: {zmqTopics.topicMavlinkFlightData} on port: {zmqTopics.topicMavlinkPort}")

#################################################################################################################
    def init_succeeded(self):
        return self._init_success

    def listenerToMavlink(self, blocking=True, timeout=0.001, use_lock=True, apply_filter=True):
        """
        Listen to mavlink messages and process them.
        
        Args:
            blocking: If True, blocks waiting for a message. If False, returns immediately if no message.
            timeout: Timeout in seconds (only used when blocking=True)
            use_lock: If True, uses thread-safe locks for data access (default True)
            apply_filter: If True, applies filtering to the data after parsing (default True)
        """
        msg = self.mavlink_connection.recv_match(blocking=blocking, timeout=1 if blocking else 0.0)
        if(msg is None):
            return
        if(msg.get_type() == "BAD_DATA"):
            return
        msg_dict = msg.to_dict()
        msg_dict['local-ts'] = time.monotonic()

        if 'time_boot_ms' in msg_dict.keys():
            pass
        
        if(msg_dict['mavpackettype'] == 'HEARTBEAT'):
            msg_dict['mode_string'] = ( mavutil.mode_string_v10(msg))
            
        if msg_dict['mavpackettype'] in RELEVANT_MAVLINK_MESSAGES:
            ind = RELEVANT_MAVLINK_MESSAGES.index(msg_dict['mavpackettype'])
            # Use lock for thread safety if requested
            if use_lock:
                with self._data_lock:
                    self.parse(msg_dict)
                    # Apply filtering if requested
                    if apply_filter:
                        self._filter_data(self._current_data)
            else:
                self.parse(msg_dict)
                if apply_filter:
                    self._filter_data(self._current_data)
        else:
            pass
            # print(str(msg_dict))
        
        # Handle mavlink logger with thread safety
        if use_lock:
            with self._data_lock:
                if self._current_data.custom_mode_id != PX4_FLIGHT_STATE.OFFBOARD.value:  # OFFBOARD state
                    self._mavlink_logger = None
                elif self._mavlink_logger is None:
                    self._mavlink_logger = Logger(log_name=time.strftime("%Y%m%d_%H%M%S")+"_mavlink", log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="TXT")                    
                
                if self._mavlink_logger is not None:
                    self._mavlink_logger.log(str(msg_dict))
        else:
            if self._current_data.custom_mode_id != PX4_FLIGHT_STATE.OFFBOARD.value:  # OFFBOARD state
                self._mavlink_logger = None
            elif self._mavlink_logger is None:
                self._mavlink_logger = Logger(log_name=time.strftime("%Y%m%d_%H%M%S")+"_mavlink", log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="TXT")                    
            
            if self._mavlink_logger is not None:
                self._mavlink_logger.log(str(msg_dict))
################################################################################################################
    def listenerToCommands(self):
        ret = zmq.select([subSock], [], [], timeout=0.001)
        # ret = ret[0]
        if ret[0] is None or len(ret[0]) == 0:
            return
        data = subSock.recv_multipart()
        topic = data[0]
        data = pickle.loads(data[1])
        
        if topic == zmqTopics.topicGuidenceCmdAttitude:
            targetQuat = Quaternion(x=data['quatNedDesBodyFrdCmd'][1], y=data['quatNedDesBodyFrdCmd'][2], z=data['quatNedDesBodyFrdCmd'][3], w=data['quatNedDesBodyFrdCmd'][0])
            rpyRateCmd = Rate_Cmd(rpydot=np.array([data['rpyRateCmd'][0], data['rpyRateCmd'][1], data['rpyRateCmd'][2]]))
            thrustCmd = data['thrustCmd']
            isRate = data['isRate']
            if isRate:
                self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=None, rates=rpyRateCmd)
            else:
                self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=targetQuat, rates=None)
            
        elif topic == zmqTopics.topicGuidenceCmdVelNed:     
            yawCmd = data['yawCmd'] if not np.isnan(data['yawCmd']) else None
            yawRateCmd = data['yawRateCmd'] if not np.isnan(data['yawRateCmd']) else None
            velCmd = data['velCmd']
            self._send_setpoint(pos=None, vel=velCmd, acc=None, yaw=yawCmd, yaw_rate=yawRateCmd)
            
        elif topic == zmqTopics.topicGuidenceCmdVelBody:     
            yawCmd = data['yawCmd'] if not np.isnan(data['yawCmd']) else None
            yawRateCmd = data['yawRateCmd'] if not np.isnan(data['yawRateCmd']) else None
            velCmd = self._current_data.quat_ned_bodyfrd.rotate_vec(data['velCmd'])
            # velCmd[2] = 0.0
            self._send_setpoint(pos=None, vel=velCmd, acc=None, yaw=yawCmd, yaw_rate=yawRateCmd)
            
        elif topic == zmqTopics.topicGuidenceCmdAcc:
            yawCmd = data['yawCmd'] if not np.isnan(data['yawCmd']) else None
            yawRateCmd = data['yawRateCmd'] if not np.isnan(data['yawRateCmd']) else None
            accCmd = data['accCmd']
            self._send_setpoint(pos=None, vel=None, acc=accCmd, yaw=yawCmd, yaw_rate=yawRateCmd)
            
        elif topic == zmqTopics.topicGuidenceCmdTakeoff:
            msg = pickle.loads(data[1])
            takeoff_altitude = msg['takeoff_altitude']
            self._send_takeoff_cmd(takeoff_altitude)
            
        elif topic == zmqTopics.topicGuidenceCmdLand:
            self._send_land_cmd()
            
        elif topic == zmqTopics.topicGuidanceCmdArm:
            msg = pickle.loads(data[1])
            self._arm()
                    
        
#################################################################################################################

    def _send_takeoff_cmd(self, takeoff_altitude = 10):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller takeoff is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
 
        print("Connected to PX4 autopilot")
        print(self.mavlink_connection.mode_mapping())
        mode_id = self.mavlink_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
         # Command Takeoff
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        time.sleep(5)
        print("Takeoff done")
#################################################################################################################
    def _send_arm_cmd(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller takeoff is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
 
        print("Connected to PX4 autopilot")
        print(self.mavlink_connection.mode_mapping())
        mode_id = self.mavlink_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + 1]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
        #  Command Takeoff
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        time.sleep(1)
        print("Takeoff done")
   
#################################################################################################################
    def _filter_data(self, current_mavlink_data:Flight_Data):
        vel_vertical = 0
        # current_mavlink_data.altitude_m.relative = self._vertical_speed_filter.step(current_mavlink_data.altitude_m.relative)
        if(self._prev_alt_m is not None):
            
            dt = current_mavlink_data.altitude_m.timestamp - self._prev_alt_ts
            if(dt != 0 ):
                current_mavlink_data.altitude_m.relative = self._altitude_filter.step(current_mavlink_data.altitude_m.relative)
                d_alt = current_mavlink_data.altitude_m.relative - self._prev_alt_m
                vel_vertical = d_alt/dt
                if(abs(vel_vertical - self._prev_vel_vertical) > MAX_VERTICAL_VEL_JUMP_M_S): #10, 0
                    # print("vel jump",abs(vel_vertical - self._prev_vel_vertical))
                    if(vel_vertical - self._prev_vel_vertical > 0 ):
                        vel_vertical = self._prev_vel_vertical + MAX_VERTICAL_VEL_JUMP_M_S
                    else:
                        vel_vertical = self._prev_vel_vertical - MAX_VERTICAL_VEL_JUMP_M_S
                
                current_mavlink_data.altitude_m.vertical_speed_estimate = self._vertical_speed_filter.step(vel_vertical)
                self._prev_vel_vertical = current_mavlink_data.altitude_m.vertical_speed_estimate
                if(self._alt_vel_count < 10 and 0):
                    self._alt_vel_count = self._alt_vel_count + 1
                    current_mavlink_data.altitude_m.vertical_speed_estimate  = 0
                # else:
                    # d_alt = current_mavlink_data.altitude_m.relative - self._prev_alt_m
                    # vel_vertical = d_alt/dt
                    # current_mavlink_data.altitude_m.vertical_speed_estimate = self._vertical_speed_filter.step(vel_vertical)
                # print("vel vertical",current_mavlink_data.altitude_m.vertical_speed_estimate, d_alt, dt, current_mavlink_data.altitude_m.relative)
                
                self._prev_alt_m = np.copy(current_mavlink_data.altitude_m.relative)
                self._prev_alt_ts = np.copy(current_mavlink_data.altitude_m.timestamp)
        else:
            self._prev_alt_m = np.copy(current_mavlink_data.altitude_m.relative)
            self._prev_alt_ts = np.copy(current_mavlink_data.altitude_m.timestamp)
            
#################################################################################################################
    def _init_mavlink(self):
        try:
            self.mavlink_connection = mavutil.mavlink_connection(mavlinkAddress, autoreconnect=True)
            self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            # self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            #                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            ARDUPILOT = False
            if(ARDUPILOT):
                self.mavlink_connection.mav.request_data_stream_send(self.mavlink_connection.target_system, 
                                                                     self.mavlink_connection.target_component,
                                                                     mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                                     MAVLINK_RATE_HZ, 1)
        except Exception as e:
            print("failed to initialize mavlink connection with error: "+str(e))
            return
        count = 0
        success = True
        out = None
        while(count < 3):
            count = count + 1
            try:
                self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                out = self.mavlink_connection.wait_heartbeat(timeout=2)
                if(out is not None):
                    break

            except:
                pass
        if(out is None):
            success = False

        return success

#################################################################################################################
    def _send_setpoint(self, pos=None, vel=None, acc=None, yaw=None, yaw_rate=None, frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED):
        # if(not self._offboard_control_enabled):
        #     return
        time_boot_ms = 0
        coordinate_frame = frame
        target_system = self.mavlink_connection.target_system
        target_component = self.mavlink_connection.target_component
        # Bitmask to indicate which fields should be ignored by the vehicle 
        # (see POSITION_TARGET_TYPEMASK enum) https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, 
        # bit7:AccX, bit8:AccY, bit9:AccZ, bit10:Force(if1) or bit11:yaw, bit12:yaw rate
        # When providing Pos, Vel and/or Accel all 3 axis must be provided. 
        # At least one of Pos, Vel and Accel must be provided (e.g. providing 
        # Yaw or YawRate alone is not supported)                    
        # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        # Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        # Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
        # Use Force : 0b111000111111 
        # Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
        # Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
        # Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
        # Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)    
        type_mask2 = 0b010111000111
        type_mask = 0b000000000000
        if pos is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            pos = np.zeros(3)
        if vel is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            vel = np.zeros(3)
        if acc is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            acc = np.zeros(3)
        if yaw is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw = 0
        if yaw_rate is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_rate = 0
        north = np.float16(pos[0])
        east = np.float16(pos[1])
        down = np.float16(pos[2])
        vx = np.float16(vel[0])
        vy = np.float16(vel[1])
        vz = np.float16(vel[2])
        afx = np.float16(acc[0])
        afy = np.float16(acc[1])
        afz = np.float16(acc[2])
        yaw = np.float16(yaw)
        yaw_rate = np.float16(yaw_rate)
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            coordinate_frame,
            type_mask,
            north,
            east,
            down,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate
         )

#################################################################################################################
    def _send_goal_attitude(self, goal_thrust, goal_attitude:Quaternion=None, rates:Rate_Cmd=None):
        if(not self._offboard_control_enabled):
            return
        # print("sending attitude command", goal_thrust)
        type_mask= 0b000000000000 #ignore rates
        if goal_attitude is None:
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
            goal_attitude = Quaternion.euler_to_quat(roll=0, pitch=0, yaw=0)
        if rates is None:
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            rates = Rate_Cmd(np.zeros(3))
            
        time_boot_ms = int(time.monotonic()*1000)
        target_system = 0
        target_component = 1
        q = [goal_attitude.w, goal_attitude.x, goal_attitude.y, goal_attitude.z]
        # q = [1,0,0,0]
        body_roll_rate = rates.rpydot[0]
        body_pitch_rate = rates.rpydot[1]
        body_yaw_rate = rates.rpydot[2]
        thrust_bodyfrd = [0,0,0]
        thrust = goal_thrust
        self.mavlink_connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.mavlink_connection.target_system,
            target_component,
            type_mask,
            q,
            body_roll_rate,
            body_pitch_rate,
            body_yaw_rate,
            thrust,
            thrust_bodyfrd
        )


#################################################################################################################
    def _send_offboard_cmd(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller switching to offboard is currently not allowed")
            return
        if(self._disable_offboard_control):
            return
        mode = "OFFBOARD"
        if mode not in  self.mavlink_connection.mode_mapping():
            print("unknown mode")
        else:
            
            mode_id = self.mavlink_connection.mode_mapping()[mode]
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system, 
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                0,
                mode_id[0], 
                mode_id[1],
                mode_id[2], 
                0, 0, 0, 0)

#################################################################################################################
    def _arm(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller arming is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

#################################################################################################################
    def _send_land_cmd(self):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

#################################################################################################################

    def parse(self, msg_dict):
        success = False


        key = msg_dict['mavpackettype']
        if(key == 'TIMESYNC'):
            pass
        
        elif(key == 'VFR_HUD'):
            self._current_data.local_ts = time.monotonic()
            self._current_data.altitude_m = msg_dict['alt']
            self._current_data.heading = np.deg2rad(msg_dict['heading'])
            self._current_data.throttle = msg_dict['throttle']
        elif(key == 'SCALED_PRESSURE'):
            pass

        elif(key == 'SCALED_IMU2'):
            pass
        elif(key =='ATTITUDE_QUATERNION'):  # 50 HZ
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_boot_ms']
            self._current_data.quat_ned_bodyfrd = Quaternion(w=float(msg_dict['q1']), x=float(msg_dict['q2']), y=float(msg_dict['q3']), z=float(msg_dict['q4']))
            self._current_data.rpy_rates = np.array([float(msg_dict['rollspeed']), float(msg_dict['pitchspeed']), float(msg_dict['yawspeed'])])
            self._current_data.gathered['quat_ned_bodyfrd'] = True
            self._current_data.gathered['rpy_rates'] = True
        elif(key == 'ATTITUDE'):
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_boot_ms']
            self._current_data.rpy = np.array([float(msg_dict['roll']), float(msg_dict['pitch']), float(msg_dict['yaw'])])
            self._current_data.rpy_rates = np.array([float(msg_dict['rollspeed']), float(msg_dict['pitchspeed']), float(msg_dict['yawspeed'])])
            self._current_data.gathered['rpy'] = True
            self._current_data.gathered['rpy_rates'] = True
        elif(key == 'ATTITUDE_TARGET'):
            pass
        elif(key == 'GLOBAL_POSITION_INT'):
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_boot_ms']
            self._current_data.filt_pos_lla_deg.lla = np.array([float(msg_dict['lat'])/(1e7), float(msg_dict['lon'])/(1e7), float(msg_dict['alt'])/(1e3)])
            self._current_data.relative_m = msg_dict['relative_alt']/1000.0
            self._current_data.heading = np.deg2rad(msg_dict['hdg']/100.0)
            self._current_data.pos_ned_m.vel_ned = np.array([float(msg_dict['vx'])/100.0, float(msg_dict['vy'])/100.0, float(msg_dict['vz'])/100.0])
            self._current_data.gathered['pos_ned_m'] = True
            self._current_data.gathered['vel_ned_m'] = True
        elif(key == 'ALTITUDE'): # 10 HZ
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_usec']/1000.0
            self._current_data.relative_m = float(msg_dict['altitude_relative'])
            self._current_data.amsl_m = float(msg_dict['altitude_amsl'])
            self._current_data.local_m = float(msg_dict['altitude_local'])
            self._current_data.monotonic_m = float(msg_dict['altitude_monotonic'])
            self._current_data.terrain_m = float(msg_dict['altitude_terrain'])
            self._current_data.bottom_clearance_m = float(msg_dict['bottom_clearance'])
            self._current_data.gathered['relative_m'] = True
            self._current_data.gathered['amsl_m'] = True
            self._current_data.gathered['local_m'] = True
            self._current_data.gathered['monotonic_m'] = True
            self._current_data.gathered['terrain_m'] = True
            self._current_data.gathered['bottom_clearance_m'] = True
        elif(key == 'HIGHRES_IMU'): # 50 HZ
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_usec']/1000.0
            self._current_data.imu_ned.accel = np.array([float(msg_dict['xacc']), float(msg_dict['yacc']), float(msg_dict['zacc'])])
            self._current_data.imu_ned.gyro = np.array([float(msg_dict['xgyro']), float(msg_dict['ygyro']), float(msg_dict['zgyro'])])
            self._current_data.imu_ned.timestamp = self._current_data.timestamp
            self._current_data.absolute_press_hpa = float(msg_dict['abs_pressure'])
            self._current_data.differential_press_hpa = float(msg_dict['diff_pressure'])
            self._current_data.pressure = float(msg_dict['pressure_alt'])
            self._current_data.temperature = float(msg_dict['temperature'])
            self._current_data.gathered['imu_ned'] = True
            self._current_data.gathered['absolute_press_hpa'] = True
            self._current_data.gathered['differential_press_hpa'] = True
            self._current_data.gathered['pressure'] = True
            self._current_data.gathered['temperature'] = True
        elif(key == 'HEARTBEAT'):
            self._current_data.local_ts = time.monotonic()
            if(msg_dict['custom_mode'] != 0):
                self._current_data.custom_mode_id = msg_dict['custom_mode']
                self._current_data.mode = msg_dict['mode_string']
                self._current_data.gathered['custom_mode_id'] = True
                self._current_data.gathered['mode'] = True
        elif(key== 'LOCAL_POSITION_NED' ):
            self._current_data.local_ts = time.monotonic()
            self._current_data.timestamp = msg_dict['time_boot_ms']
            self._current_data.pos_ned_m.ned = np.array([msg_dict['x'], msg_dict['y'], msg_dict['z']])
            self._current_data.pos_ned_m.vel_ned = np.array([msg_dict['vx'], msg_dict['vy'], msg_dict['vz']])
            self._current_data.pos_ned_m.timestamp = msg_dict['time_boot_ms']
            self._current_data.gathered['pos_ned_m'] = True
            self._current_data.gathered['vel_ned_m'] = True
                
        if((key == 'HEARTBEAT') and (self._current_data.custom_mode_id == 0)):
            pass 
        elif(key == 'HEARTBEAT'):
            pass

#################################################################################################################
    def _command_thread_func(self):
        """
        Thread function for handling commands from system_manager and sending them to mavlink.
        This thread continuously listens for commands via ZMQ and forwards them to the mavlink server.
        """
        global subSock
        
        print("Hardware_adapter: Command thread started, waiting for commands...")
        
        # Wait for subscriber socket to be created
        while subSock is None and self._running:
            time.sleep(0.01)
        
        if subSock is None:
            print("Error: Subscriber socket not created!")
            return
        
        # Set socket to non-blocking
        subSock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        print(f"Hardware_adapter: Command thread ready, socket connected to port {zmqTopics.topicGuidenceCmdPort}")
        
        # Test connection by waiting a bit and checking if we can receive
        print("Hardware_adapter: Waiting 1 second to test connection to system_manager...")
        time.sleep(1.0)
        test_receive_count = 0
        for i in range(20):  # Check for 200ms
            try:
                test_msg = subSock.recv(zmq.NOBLOCK)  # Single-part message with topic prefix
                # Check if message has a valid topic prefix
                if (test_msg.startswith(zmqTopics.topicGuidenceCmdAttitude) or
                    test_msg.startswith(zmqTopics.topicGuidenceCmdVelNed) or
                    test_msg.startswith(zmqTopics.topicGuidenceCmdVelBody) or
                    test_msg.startswith(zmqTopics.topicGuidenceCmdAcc) or
                    test_msg.startswith(zmqTopics.topicGuidanceCmdArm)):
                    test_receive_count += 1
            except zmq.Again:
                pass
            except Exception as e:
                print(f"Hardware_adapter: Error testing subscriber connection: {e}")
                break
            time.sleep(0.01)
        
        if test_receive_count > 0:
            print(f"Hardware_adapter: SUCCESS - Received {test_receive_count} test messages from system_manager!")
        else:
            print(f"Hardware_adapter: WARNING - No messages received during connection test.")
            print(f"  This means system_manager is either:")
            print(f"    1. Not running")
            print(f"    2. Not publishing commands yet")
            print(f"    3. Publishing on a different port")
            print(f"  Expected publisher port: {zmqTopics.topicGuidenceCmdPort}")
        
        last_debug_time = time.monotonic()
        msg_count = 0
        no_msg_warning_time = time.monotonic()
        
        while self._running:
            try:
                # Receive single-part message with topic prefix
                try:
                    msg = subSock.recv(zmq.NOBLOCK)
                    # Reset warning timer if we received a message
                    no_msg_warning_time = time.monotonic()
                    
                    # Debug: Print first message received to verify format
                    if not hasattr(self, '_first_msg_received'):
                        self._first_msg_received = True
                        print(f"Hardware_adapter: First command message received! Length: {len(msg)} bytes")
                        # Check topic prefix
                        if msg.startswith(zmqTopics.topicGuidenceCmdVelNed):
                            print(f"  Topic: {zmqTopics.topicGuidenceCmdVelNed}")
                        elif msg.startswith(zmqTopics.topicGuidenceCmdAttitude):
                            print(f"  Topic: {zmqTopics.topicGuidenceCmdAttitude}")
                        elif msg.startswith(zmqTopics.topicGuidenceCmdAcc):
                            print(f"  Topic: {zmqTopics.topicGuidenceCmdAcc}")
                        else:
                            print(f"  First 50 bytes: {msg[:50]}")
                except zmq.Again:
                    # No message available, continue
                    # Warn if no messages received for 5 seconds
                    if time.monotonic() - no_msg_warning_time > 5.0:
                        print(f"Hardware_adapter: No commands received for 5s. Is system_manager sending? Port: {zmqTopics.topicGuidenceCmdPort}")
                        no_msg_warning_time = time.monotonic()
                    time.sleep(0.001)
                    continue
                except zmq.ZMQError as e:
                    if e.errno == zmq.EAGAIN:
                        time.sleep(0.001)
                        continue
                    else:
                        print(f"ZMQ error in command thread: {e}")
                        time.sleep(0.01)
                        continue
                
                if msg is None or len(msg) == 0:
                    print(f"Warning: Received empty message")
                    continue
                
                # Extract topic from message prefix
                topic = None
                data_bytes = None
                if msg.startswith(zmqTopics.topicGuidenceCmdAttitude):
                    topic = zmqTopics.topicGuidenceCmdAttitude
                    data_bytes = msg[len(zmqTopics.topicGuidenceCmdAttitude):]
                elif msg.startswith(zmqTopics.topicGuidenceCmdVelNed):
                    topic = zmqTopics.topicGuidenceCmdVelNed
                    data_bytes = msg[len(zmqTopics.topicGuidenceCmdVelNed):]
                elif msg.startswith(zmqTopics.topicGuidenceCmdVelBody):
                    topic = zmqTopics.topicGuidenceCmdVelBody
                    data_bytes = msg[len(zmqTopics.topicGuidenceCmdVelBody):]
                elif msg.startswith(zmqTopics.topicGuidenceCmdAcc):
                    topic = zmqTopics.topicGuidenceCmdAcc
                    data_bytes = msg[len(zmqTopics.topicGuidenceCmdAcc):]
                elif msg.startswith(zmqTopics.topicGuidanceCmdArm):
                    topic = zmqTopics.topicGuidanceCmdArm
                    data_bytes = msg[len(zmqTopics.topicGuidanceCmdArm):]
                else:
                    # Debug: print first few bytes to see what we received
                    prefix_bytes = msg[:min(50, len(msg))]
                    print(f"Warning: Received message with unknown topic prefix. First 50 bytes: {prefix_bytes}")
                    print(f"  Expected topics: {[zmqTopics.topicGuidenceCmdAttitude, zmqTopics.topicGuidenceCmdVelNed, zmqTopics.topicGuidenceCmdVelBody, zmqTopics.topicGuidenceCmdAcc, zmqTopics.topicGuidanceCmdArm]}")
                    continue
                
                try:
                    data = pickle.loads(data_bytes)
                except Exception as e:
                    print(f"Error unpickling command data: {e}, topic: {topic}, data length: {len(data_bytes) if data_bytes else 0}")
                    continue
                
                if data is None or (isinstance(data, dict) and len(data) == 0):
                    print(f"Warning: Received empty command data for topic: {topic}")
                    continue
                
                # Safely access optional message metadata (not all command types include these)
                message_count = data.get('message_count', 0)
                message_ts = data.get('message_ts', time.monotonic())
                message_delay = time.monotonic() - message_ts
                # print(f"Hardware_adapter: message count {message_count}, message delay {message_delay:.6f}")
                msg_count += 1
                
                # Debug: Print received command info (only occasionally to avoid spam)
                current_time = time.monotonic()
                if current_time - last_debug_time > 2.0:  # Print every 2 seconds
                    print(f"Hardware_adapter: Received {msg_count} commands in last 2s. Latest - Topic: {topic}, Data keys: {list(data.keys()) if isinstance(data, dict) else 'N/A'}")
                    if isinstance(data, dict) and 'velCmd' in data:
                        print(f"  velCmd: {data['velCmd']}")
                    msg_count = 0
                    last_debug_time = current_time
                
                if topic == zmqTopics.topicGuidenceCmdAttitude:
                    targetQuat = Quaternion(x=data['quatNedDesBodyFrdCmd'][1], y=data['quatNedDesBodyFrdCmd'][2], z=data['quatNedDesBodyFrdCmd'][3], w=data['quatNedDesBodyFrdCmd'][0])
                    rpyRateCmd = Rate_Cmd(rpydot=np.array([data['rpyRateCmd'][0], data['rpyRateCmd'][1], data['rpyRateCmd'][2]]))
                    thrustCmd = data['thrustCmd']
                    isRate = data['isRate']
                    if isRate:
                        self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=None, rates=rpyRateCmd)
                    else:
                        self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=targetQuat, rates=None)
                    
                elif topic == zmqTopics.topicGuidenceCmdVelNed:     
                    if 'velCmd' not in data:
                        print(f"Warning: velCmd missing in command data. Keys: {data.keys() if isinstance(data, dict) else 'not a dict'}")
                        continue
                    yawCmd = data.get('yawCmd', np.nan)
                    yawCmd = yawCmd if not np.isnan(yawCmd) else None
                    yawRateCmd = data.get('yawRateCmd', np.nan)
                    yawRateCmd = yawRateCmd if not np.isnan(yawRateCmd) else None
                    velCmd = data['velCmd']
                    if velCmd is None or (isinstance(velCmd, np.ndarray) and velCmd.size == 0):
                        print(f"Warning: Empty velCmd received: {velCmd}")
                        continue
                    if not isinstance(velCmd, (list, np.ndarray)) or len(velCmd) != 3:
                        print(f"Warning: Invalid velCmd format. Expected array of length 3, got: {type(velCmd)}, length: {len(velCmd) if hasattr(velCmd, '__len__') else 'N/A'}")
                        continue
                    self._send_setpoint(pos=None, vel=velCmd, acc=None, yaw=yawCmd, yaw_rate=yawRateCmd)
                    
                elif topic == zmqTopics.topicGuidenceCmdVelBody:     
                    # Need to read current quaternion for rotation - use lock for thread safety
                    with self._data_lock:
                        if not self._current_data.gathered.get('quat_ned_bodyfrd', False):
                            # Quaternion not yet available, skip this command
                            continue
                        current_quat = self._current_data.quat_ned_bodyfrd
                    yawCmd = data['yawCmd'] if not np.isnan(data['yawCmd']) else None
                    yawRateCmd = data['yawRateCmd'] if not np.isnan(data['yawRateCmd']) else None
                    velCmd = current_quat.rotate_vec(data['velCmd'])
                    self._send_setpoint(pos=None, vel=velCmd, acc=None, yaw=yawCmd, yaw_rate=yawRateCmd)
                    
                elif topic == zmqTopics.topicGuidenceCmdAcc:
                    yawCmd = data['yawCmd'] if not np.isnan(data['yawCmd']) else None
                    yawRateCmd = data['yawRateCmd'] if not np.isnan(data['yawRateCmd']) else None
                    accCmd = data['accCmd']
                    self._send_setpoint(pos=None, vel=None, acc=accCmd, yaw=yawCmd, yaw_rate=yawRateCmd)
                    
                elif topic == zmqTopics.topicGuidenceCmdTakeoff:
                    takeoff_altitude = data.get('takeoff_altitude', 10)
                    self._send_takeoff_cmd(takeoff_altitude)
                    
                elif topic == zmqTopics.topicGuidenceCmdLand:
                    self._send_land_cmd()
                    
                elif topic == zmqTopics.topicGuidanceCmdArm:
                    self._arm()
                    
            except Exception as e:
                print(f"Error in command thread: {e}")
                time.sleep(0.001)

#################################################################################################################
    def _data_thread_func(self):
        """
        Thread function for maintaining current_data structure and publishing to system_manager.
        This thread continuously listens to mavlink messages, updates the current_data structure,
        and publishes it via ZMQ to system_manager.
        """
        outTime = time.monotonic() + self._publish_dt
        printTime = time.monotonic() + 1.0  # Print every second
        
        while self._running:
            try:
                current_time = time.monotonic()
                
                # Process mavlink messages (non-blocking, with thread safety and filtering)
                self.listenerToMavlink(blocking=True, timeout=0.0, use_lock=True, apply_filter=True)
                
                # Publish data at specified frequency (independent of mavlink message arrival)
                if current_time >= outTime:
                    outTime = current_time + self._publish_dt
                    with self._data_lock:
                        self._current_data.local_ts = time.monotonic()
                        data = pickle.dumps(self._current_data)
                    try:
                        # Send as single-part message with topic prefix to enable CONFLATE support
                        # Topic prefix is needed for ZMQ subscription filtering, but it's still a single-part message
                        sockPub.send(zmqTopics.topicMavlinkFlightData + data)
                        self._current_data.message_count += 1
                        # Debug: Track publishing rate
                        if not hasattr(self, '_publish_count'):
                            self._publish_count = 0
                            self._last_publish_debug_time = current_time
                        self._publish_count += 1
                        if current_time - self._last_publish_debug_time > 2.0:
                            print(f"Hardware_adapter: Published {self._publish_count} messages in last 2s to port {zmqTopics.topicMavlinkPort}")
                            self._publish_count = 0
                            self._last_publish_debug_time = current_time
                    except Exception as e:
                        print(f"Error publishing data: {e}")
                
                # Print status at lower frequency
                if current_time >= printTime:
                    printTime = current_time + 1.0
                    with self._data_lock:
                        print("timestamp: ", self._current_data.timestamp)
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.0001)
                        
            except Exception as e:
                # print(f"Error in data thread: {e}")
                time.sleep(0.001)

#################################################################################################################
    def stop(self):
        """Stop both threads gracefully."""
        self._running = False
        if self._command_thread.is_alive():
            self._command_thread.join(timeout=1.0)
        if self._data_thread.is_alive():
            self._data_thread.join(timeout=1.0)

#################################################################################################################
def main():
    hardware_adapter = Hardware_Adapter(log_dir='../logs/')
    
    if not hardware_adapter.init_succeeded():
        print("Hardware adapter initialization failed. Exiting.")
        return
    
    print("Hardware adapter started with two threads:")
    print("  - Command thread: handling commands from system_manager to mavlink")
    print("  - Data thread: maintaining current_data and publishing to system_manager")
    
    try:
        # Main loop just keeps the process alive
        # The threads handle all the work asynchronously
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down hardware adapter...")
        hardware_adapter.stop()
        print("Hardware adapter stopped.")

if __name__ == '__main__':
    main()


