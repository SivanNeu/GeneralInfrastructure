import time
import numpy as np
import torch
from rl_policyClean import RLPolicyClean
from copy import deepcopy

from SF_Enjoy import SF_Enjoy_main

from math import sin, cos, pi, sqrt, atan2

        
###############################################################################################################
class VelocityRLController:

    def __init__(self, maximalVelocity=3.0):

        self.controllerType = "VelocityRL"
        self.dt = 0.01

        # state space parameters
        self.pos_self = [0.0,0.0,0.0] # NED

        self.pos_target = [0.0,0.0,0.0]
        self.heading_target = [0.0,0.0,0.0]
        self.target_distance = 0.0    # relative to self
        self.target_distance_dot = 0.0
        self.last_distance = 0.0
        self.last_theta = 0.0
        self.target_angle = 0.0       # relative to Forward
        self.max_vel = maximalVelocity
        self.max_omega = np.deg2rad(30)
        
        self.ringLen = 1
        self.ringV = np.zeros((2,self.ringLen))
        self.ringIndex = 0
        self.ringAverage = np.zeros(2)

        # policy network setup
        
        # sf_enjoy = SF_Enjoy_main()
        # self.sf_policy = sf_enjoy.enjoy
        
        self.rl_policyVfVr = RLPolicyClean.load_from_checkpoint(path='./train_dir/relu_r/checkpoint_p0/best_000007954_8144896_reward_121.191.pth')
        self.rl_policyOmegaYaw = RLPolicyClean.load_from_checkpoint(path='./train_dir/relu_yaw/checkpoint_p0/best_000009767_10001408_reward_7754.513.pth')
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, _, _, gyro_ned, quat_ned_bodyfrd = currentBodyState
        gyro_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(gyro_ned)
        
        self.pos_target, _, _, _, _ = desiredBodyState[0]
        self.heading_target, _, _= desiredBodyState[1]; self.heading_target = self.heading_target/np.linalg.norm(self.heading_target)
        self.heading_target_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(self.heading_target)
        
        heading_error = np.arctan2(self.heading_target_bodyfrd[1], self.heading_target_bodyfrd[0])
        
        if currentData is not None:
            self.yaw=currentData.rpy[2]
        
        ## transform into observation space data
        delta = self.pos_target[0:2] - self.pos_self[0:2]
        self.target_distance = np.linalg.norm(delta)
        delta_body = quat_ned_bodyfrd.inv().rotate_vec(self.pos_target - self.pos_self)
        if np.linalg.norm(delta_body) < 1e-6:
            self.target_angle = self.last_theta
        else:
            self.target_angle = np.arctan2(delta_body[0], delta_body[1])
            
        self.target_distance_dot = (self.last_distance - self.target_distance)/self.dt
        self.last_distance = self.target_distance
        self.last_theta = self.target_angle

        obsXY = np.array([[self.target_distance, 
                        np.sin(self.target_angle), 
                        np.cos(self.target_angle),  
                        ]],dtype=np.float32)
        obsHeading = np.array([[np.sin(heading_error), 
                                np.cos(heading_error),
                                gyro_bodyfrd[2]*0
                                ]],dtype=np.float32)
        
        ## execute policy inference

        # action_logits = self.sf_policy(obs)  #, deterministic=True

        
        # action_logits_sf, rnn_states_sf = self.sf_policy(obsXY)
        rl_action_logitsClean, rl_hxsClean = self.rl_policyVfVr(obsXY)
        rl_action_logitsOmegaYaw, rl_hxsOmegaYaw = self.rl_policyOmegaYaw(obsHeading)

        action_mean = rl_action_logitsClean[0][:3]
        action_logstd = rl_action_logitsClean[0][3:6]            
        vf_,vr_,w = action_mean.detach().numpy()  # [v_r, v_θ, w_yaw]
        vf = np.clip(vf_,-self.max_vel,self.max_vel)
        vr = np.clip(vr_,-self.max_vel,self.max_vel)
        w  = rl_action_logitsOmegaYaw.detach().numpy()[0][0]
        ## vectorize and send outputs
        vel_vector = [vf,vr,0] # in FRD
        omega_vector = [0,0,w]
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        return vel_vector, np.eye(3), omega_vector, obsXY

    



