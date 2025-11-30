import time
import numpy as np
import torch
from common import CONTROLLER_TYPE
from rl_policyClean import RLPolicyClean
from copy import deepcopy
from common import PX4_FLIGHT_STATE

from SF_Enjoy import SF_Enjoy_main

from math import sin, cos, pi, sqrt, atan2

###############################################################################################################
class VelocityRLControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
   
###############################################################################################################
class VelocityRLController:

    def __init__(self, mass=0.5, maximalVelocity=3.0, currentTime=None):

        self.controllerName = "VelocityRL"
        self.controllerType = CONTROLLER_TYPE.VELOCITYRL
        
        self.lastTime = time.monotonic() if currentTime is None else currentTime
        self.current_time = time.monotonic() if currentTime is None else currentTime

        # state space parameters
        self.pos_self = [0.0,0.0,0.0] # NED
        self.vel_self = [0.0,0.0,0.0] # NED
        self.pos_target = [0.0,0.0,0.0]
        self.heading_target = [0.0,0.0,0.0]

        self.max_vel = maximalVelocity
        # Position integral scale
        self.max_range = 15
        self.int_scale = self.max_range * 20  # 20sec

        self.max_omega = np.deg2rad(30)
        
        self.ringLen = 1
        self.ringV = np.zeros((2,self.ringLen))
        self.ringIndex = 0
        self.ringAverage = np.zeros(2)

        self.param = VelocityRLControllerParameters(mass=mass)

        # policy network setup
        
        # sf_enjoy = SF_Enjoy_main()
        # self.sf_policy = sf_enjoy.enjoy
        
        self.rl_policyVfVr     = RLPolicyClean.load_from_checkpoint(path='./train_dir/gazebo_quad_no_int/checkpoint_p0/best_000001173_1201152_reward_295.335.pth')
        self.rl_policyOmegaYaw = RLPolicyClean.load_from_checkpoint(path='./train_dir/relu_yaw/checkpoint_p0/best_000009767_10001408_reward_7754.513.pth')
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, self.vel_self, _, gyro_ned, quat_ned_bodyfrd = currentBodyState
        gyro_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(gyro_ned)
        
        self.pos_target, self.vel_target, _, _, _ = desiredBodyState[0]
        self.heading_target, _, _= desiredBodyState[1]; self.heading_target = self.heading_target/np.linalg.norm(self.heading_target)
        
        heading_error = np.arctan2(self.heading_target[1], self.heading_target[0])
        
        if currentData is not None:
            self.yaw=currentData.rpy[2]
        
        ## transform into observation space data
        pos_error_ned = (self.pos_target - self.pos_self)[:2]
        vel_error_ned = self.vel_self[:2]
        if np.linalg.norm(pos_error_ned) > self.max_range:
            pos_error_ned = pos_error_ned / np.linalg.norm(pos_error_ned) * self.max_range
        if np.linalg.norm(vel_error_ned) > self.max_vel:
            vel_error_ned = vel_error_ned / np.linalg.norm(vel_error_ned) * self.max_vel

        # Update position integral (accumulate position error)
        self.current_time = currentData.local_ts
        dt = self.current_time - self.lastTime
        if dt < 0:
            dt = 0
            self.lastTime = self.current_time
        self.lastTime = self.current_time
        
        obsXY = np.array([pos_error_ned[0], pos_error_ned[1], vel_error_ned[0], vel_error_ned[1]], dtype=np.float32)
        obsHeading = np.array([[np.sin(-heading_error), np.cos(-heading_error), gyro_bodyfrd[2]]],dtype=np.float32)
        # obs = np.concatenate([obsXY[0], obsHeading[0]], dtype=np.float32)
        
        ## execute policy inference

        # action_logits, rl_hxs = self.sf_policy(obsXY) #, deterministic=True
        rl_action_logitsClean, rl_hxsClean = self.rl_policyVfVr(obsXY)
        rl_action_logitsOmegaYaw, rl_hxsOmegaYaw = self.rl_policyOmegaYaw(obsHeading)

        action_mean = rl_action_logitsClean[0][:2]
        action_logstd = rl_action_logitsClean[0][:2]
        # action_mean = action_logits[0][:2]
        # action_logstd = action_logits[0][2:4]

        vf_,vr_ = action_mean.detach().numpy()  # [v_r, v_θ, w_yaw]
        vf = np.clip(vf_,-self.max_vel,self.max_vel)
        vr = np.clip(vr_,-self.max_vel,self.max_vel)

        w  = rl_action_logitsOmegaYaw.detach().numpy()[0][0]
        ## vectorize and send outputs
        vel_vector = np.array([vf,vr,0]) # in FRD
        omega_vector = np.array([0,0,w])
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        return vel_vector, np.eye(3), omega_vector, obsXY

    def resetIntegralErrorTerms(self):
        pass



