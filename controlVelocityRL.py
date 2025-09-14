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
        
        self.policy = SF_Enjoy_main().enjoy
        
        # initial GRU hidden state
        self.rnn_states = torch.zeros(1, 1, 512)[0]
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, _, _, _, _ = currentBodyState
        self.pos_target, _, _, _, _ = desiredBodyState[0]
        if currentData is not None:
            self.yaw=currentData.rpy[2]
        
        ## transform into observation space data
        delta = self.pos_target[0:2] - self.pos_self[0:2]
        self.target_distance = np.linalg.norm(delta)
        if self.target_distance > 20.0: self.target_distance = 20.0
        self.target_angle = atan2(delta[1],delta[0])
        self.target_angle = (self.target_angle - self.yaw + pi) % (2*pi) - pi
        self.target_distance_dot = (self.last_distance - self.target_distance)/self.dt
        self.theta_dot = (self.last_theta - self.target_angle) / self.dt
        self.last_distance = self.target_distance
        self.last_theta = self.target_angle

        obs = np.array([self.target_distance, 
                        self.target_angle, 
                        self.target_distance_dot, 
                        self.theta_dot, 
                        self.ringV[0,self.ringIndex],
                        self.ringV[1,self.ringIndex],
                        self.ringAverage[0], 
                        self.ringAverage[1]],dtype=np.float32)
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        
        ## execute policy inference
        with torch.no_grad():
            rnn_states = deepcopy(self.rnn_states)
            action, rnn_states, action_mean, action_logstd = self.policy(obs_tensor, rnn_states)  #, deterministic=True
            self.rnn_states = deepcopy(rnn_states)
            
            vf_,vr_,w = action_mean.squeeze(0).numpy()  # [v_r, v_θ, w_yaw]
            vf = np.clip(vf_,-self.max_vel,self.max_vel)
            vr = np.clip(vr_,-self.max_vel,self.max_vel)
            w = np.clip(w,-self.max_omega,self.max_omega)
            
            self.ringIndex = (self.ringIndex + 1) % self.ringLen
            self.ringV[0,self.ringIndex] = vf_
            self.ringV[1,self.ringIndex] = vr_
            self.ringAverage = np.mean(self.ringV,axis=1)
        
        ## vectorize and send outputs
        vel_vector = [vf,vr,0] # in FRD
        omega_vector = [0,0,w]
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        return vel_vector, np.eye(3), omega_vector, obs

    



