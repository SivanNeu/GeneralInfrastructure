import time
import numpy as np
import torch
from rl_policy import RLPolicy
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
        self.yaw = 0.0                # relative to North
        self.pos_target = [0.0,0.0,0.0]
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
        
        sf_enjoy, self.rnn_states = SF_Enjoy_main()
        self.policy = sf_enjoy.enjoy
        
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, _, _, _, quat_ned_bodyfrd = currentBodyState
        self.pos_target, _, _, _, _ = desiredBodyState[0]
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
            
        # self.target_angle = (self.target_angle + np.pi) % (2*np.pi) - np.pi
        self.target_distance_dot = (self.last_distance - self.target_distance)/self.dt
        self.theta_dot = (self.last_theta - self.target_angle) / self.dt
        self.last_distance = self.target_distance
        self.last_theta = self.target_angle

        obs = np.array([[self.target_distance, 
                        np.sin(self.target_angle), 
                        np.cos(self.target_angle),  
                        ]],dtype=np.float32)
        # obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        
        ## execute policy inference

        action, rnn_states, action_mean, action_logstd = self.policy(obs, deepcopy(self.rnn_states))  #, deterministic=True
        self.rnn_states = deepcopy(rnn_states)
            
        vf_,vr_,w = action_mean.squeeze(0).numpy()  # [v_r, v_θ, w_yaw]
        vf = np.clip(vf_,-self.max_vel,self.max_vel)
        vr = np.clip(vr_,-self.max_vel,self.max_vel)
        w  = 0.0
        ## vectorize and send outputs
        vel_vector = [vf,vr,0] # in FRD
        omega_vector = [0,0,w]
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        return vel_vector, np.eye(3), omega_vector, obs

    



