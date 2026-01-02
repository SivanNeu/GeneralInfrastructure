import time
import os
import numpy as np
# import torch
from common import CONTROLLER_TYPE
from rl_policyClean import RLPolicyClean
from copy import deepcopy
from common import PX4_FLIGHT_STATE

# from SF_Enjoy import SF_Enjoy_main

from math import sin, cos, pi, sqrt, atan2

###############################################################################################################
class VelocityRLControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
        self.max_vel = 3.0
        self.max_range = 15.0
        self.int_scale = 300.0
        self.max_omega = np.deg2rad(90)
        self.rlFilePathVfVr = None
        self.rlFilePathOmegaYaw = None
    
    @staticmethod
    def load_from_json(json_file_path):
        """
        Load parameters from JSON file.
        
        Args:
            json_file_path: Path to JSON file containing controller parameters
            
        Returns:
            VelocityRLControllerParameters object with loaded values
        """
        import json
        params = VelocityRLControllerParameters()
        try:
            with open(json_file_path, 'r') as f:
                config = json.load(f)
            params.mass = config.get('mass', 0.5)
            params.max_vel = config.get('max_vel', 3.0)
            params.max_range = config.get('max_range', 15.0)
            params.int_scale = config.get('int_scale', 300.0)
            params.max_omega = config.get('max_omega', np.deg2rad(90))
            params.rlFilePathVfVr = config.get('rlFilePathVfVr', None)
            params.rlFilePathOmegaYaw = config.get('rlFilePathOmegaYaw', None)
            print(f"Loaded RL controller parameters from: {json_file_path}")
        except FileNotFoundError:
            print(f"Warning: Could not load controller parameter file: {json_file_path}")
            print("  Using default parameters")
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in controller parameter file: {e}")
            print("  Using default parameters")
        except Exception as e:
            print(f"Error loading controller parameter file: {e}")
            print("  Using default parameters")
        return params
   
###############################################################################################################
class VelocityRLController:

    def __init__(self, mass=0.5, maximalVelocity=3.0, currentTime=None, params=None, params_file=None):

        self.controllerName = "VelocityRL"
        self.controllerType = CONTROLLER_TYPE.VELOCITYRL
        
        self.lastTime = time.monotonic() if currentTime is None else currentTime
        self.current_time = time.monotonic() if currentTime is None else currentTime

        # state space parameters
        self.pos_self = [0.0,0.0,0.0] # NED
        self.vel_self = [0.0,0.0,0.0] # NED
        self.pos_target = [0.0,0.0,0.0]
        self.heading_target = [0.0,0.0,0.0]

        # Load parameters from file if provided, otherwise use params object or defaults
        if params_file is not None:
            self.param = VelocityRLControllerParameters.load_from_json(params_file)
        elif params is not None:
            self.param = params
        else:
            self.param = VelocityRLControllerParameters(mass=mass)

        # Use parameters from param object
        self.max_vel = maximalVelocity if maximalVelocity is not None else self.param.max_vel
        self.max_range = self.param.max_range
        self.int_scale = self.param.int_scale if self.param.int_scale > 0 else (self.max_range * 20)
        self.max_omega = self.param.max_omega
        
        self.ringLen = 1
        self.ringV = np.zeros((2,self.ringLen))
        self.ringIndex = 0
        self.ringAverage = np.zeros(2)

        # policy network setup
        # Use paths from parameters if available, otherwise use defaults
        if self.param.rlFilePathVfVr is not None:
            rlFilePathVfVr = self.param.rlFilePathVfVr
        else:
            rlFilePathVfVr = './train_dir/rlcat2_quad/checkpoint_p0/best_000003172_3248128_reward_176.079.pth'
        
        if self.param.rlFilePathOmegaYaw is not None:
            rlFilePathOmegaYaw = self.param.rlFilePathOmegaYaw
        else:
            rlFilePathOmegaYaw = './train_dir/rlcat2_yawrate/checkpoint_p0/best_000008610_8816640_reward_4791.792.pth'
        
        print("Loading RL policy from: "+rlFilePathVfVr)
        print("Loading RL policy from: "+rlFilePathOmegaYaw)
        time.sleep(0.5)
        
        # Try to load from .pth file first, if .json is provided, try to find corresponding .pth
        try:
            if rlFilePathVfVr.endswith('.json'):
                # Try to find corresponding .pth file
                pth_path = rlFilePathVfVr.replace('.json', '.pth')
                if os.path.exists(pth_path):
                    rlFilePathVfVr = pth_path
                    print(f"Using .pth file instead: {pth_path}")
                else:
                    # If .pth doesn't exist, try loading from JSON (if supported)
                    # For now, fall back to default .pth path
                    print(f"Warning: .pth file not found for {rlFilePathVfVr}, using default")
                    rlFilePathVfVr = './train_dir/rlcat2_quad/checkpoint_p0/best_000003172_3248128_reward_176.079.pth'
            
            if rlFilePathOmegaYaw.endswith('.json'):
                # Try to find corresponding .pth file
                pth_path = rlFilePathOmegaYaw.replace('.json', '.pth')
                if os.path.exists(pth_path):
                    rlFilePathOmegaYaw = pth_path
                    print(f"Using .pth file instead: {pth_path}")
                else:
                    # If .pth doesn't exist, try loading from JSON (if supported)
                    # For now, fall back to default .pth path
                    print(f"Warning: .pth file not found for {rlFilePathOmegaYaw}, using default")
                    rlFilePathOmegaYaw = './train_dir/rlcat2_yawrate/checkpoint_p0/best_000008610_8816640_reward_4791.792.pth'
            
            self.rl_policyVfVr     = RLPolicyClean.load_from_checkpoint(path=rlFilePathVfVr)
            self.rl_policyOmegaYaw = RLPolicyClean.load_from_checkpoint(path=rlFilePathOmegaYaw)
        except Exception as e:
            print(f"Error loading RL policies: {e}")
            print("Using default RL policy paths")
            self.rl_policyVfVr     = RLPolicyClean.load_from_checkpoint(path='./train_dir/rlcat2_quad/checkpoint_p0/best_000003172_3248128_reward_176.079.pth')
            self.rl_policyOmegaYaw = RLPolicyClean.load_from_checkpoint(path='./train_dir/rlcat2_yawrate/checkpoint_p0/best_000008610_8816640_reward_4791.792.pth')
       
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, self.vel_self, _, gyro_ned, quat_ned_bodyfrd = currentBodyState
        gyro_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(gyro_ned)
        
        self.pos_target, self.vel_target, _, _, _ = desiredBodyState[0]
        self.heading_target, _, _= desiredBodyState[1]; self.heading_target = self.heading_target/np.linalg.norm(self.heading_target)
        
        # Heading Observation
        curHeading= quat_ned_bodyfrd.rotate_vec(np.array([1, 0, 0]))
        # Compute signed angle: use z-component of cross product for sign
        cross = np.cross(curHeading, self.heading_target)
        cross_z = cross[2]  # z-component determines direction (positive = counterclockwise)
        dot = np.dot(curHeading, self.heading_target)
        theta = np.arctan2(cross_z, dot)  # Signed angle in [-pi, pi]
        obsHeading = np.array([theta, gyro_bodyfrd[2]], dtype=np.float32)
        
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
        w = np.clip(w,-self.max_omega,self.max_omega)
        ## vectorize and send outputs
        vel_vector = np.array([vf,vr,0]) # in FRD
        omega_vector = np.array([0,0,w])
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        obsTotal = np.concatenate([obsXY, obsHeading])
        return vel_vector, np.eye(3), omega_vector, obsTotal

    def resetIntegralErrorTerms(self):
        pass



