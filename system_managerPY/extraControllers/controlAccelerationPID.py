from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3
from common import CONTROLLER_TYPE

import time
import numpy as np

from math import sin, cos, pi, sqrt

###############################################################################################################
class AccelerationPIDControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
        self.gravity = 9.81  # Gravitational acceleration (m/s^2)

            # controller
        # Position gains
        commonFactor = 4
        self.kX = np.diag([1.0, 1.0, 1.0])*commonFactor  # Position gains
        self.kV = np.diag([1, 1, 2])*commonFactor  # Velocity gains
        # Integral gains
        self.kIX = np.diag([0, 0, .01])*commonFactor # Position integral gains
        self.kIV = np.diag([0.8, 0.8, 0.8])*0*commonFactor # Velocity integral gains
        # Differentiation gains
        self.c1 = 1.0  # Parameters for Position decoupled-yaw integral control
        
        self.sat_sigmaX = 1
        self.sat_sigmaV = 1
    
    @staticmethod
    def load_from_json(json_file_path):
        """
        Load parameters from JSON file.
        
        Args:
            json_file_path: Path to JSON file containing controller parameters
            
        Returns:
            AccelerationPIDControllerParameters object with loaded values
        """
        import json
        import sys
        params = AccelerationPIDControllerParameters()
        try:
            with open(json_file_path, 'r') as f:
                config = json.load(f)
            try:
                params.mass = config.get('mass', 0.5)
                params.gravity = config.get('gravity', 9.81)
                # Load diagonal matrices from lists
                kX_list = config.get('kX', [4.0, 4.0, 4.0])
                params.kX = np.diag(kX_list) if isinstance(kX_list, list) else np.diag([kX_list]*3)
                kV_list = config.get('kV', [4.0, 4.0, 8.0])
                params.kV = np.diag(kV_list) if isinstance(kV_list, list) else np.diag([kV_list]*3)
                kIX_list = config.get('kIX', [0.0, 0.0, 0.04])
                params.kIX = np.diag(kIX_list) if isinstance(kIX_list, list) else np.diag([kIX_list]*3)
                kIV_list = config.get('kIV', [0.0, 0.0, 0.0])
                params.kIV = np.diag(kIV_list) if isinstance(kIV_list, list) else np.diag([kIV_list]*3)
                params.c1 = config.get('c1', 1.0)
                params.sat_sigmaX = config.get('sat_sigmaX', 1.0)
                params.sat_sigmaV = config.get('sat_sigmaV', 1.0)
                print(f"Loaded Acceleration PID controller parameters from: {json_file_path}")
            except (TypeError, ValueError, KeyError) as e:
                print(f"Error: Failed to parse parameter from Acceleration PID controller parameter file: {json_file_path}")
                print(f"  Parameter error: {e}")
                print("Exiting program.")
                sys.exit(1)
        except FileNotFoundError:
            print(f"Error: Could not load Acceleration PID controller parameter file: {json_file_path}")
            print("Exiting program.")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in Acceleration PID controller parameter file: {json_file_path}")
            print(f"  JSON Error: {e}")
            print("Exiting program.")
            sys.exit(1)
        except Exception as e:
            print(f"Error: Failed to load Acceleration PID controller parameter file: {json_file_path}")
            print(f"  Error: {e}")
            print("Exiting program.")
            sys.exit(1)
        return params
        
###############################################################################################################
class AccelerationPIDController:

    def __init__(self, mass, params=None, params_file=None, currentTime=None):

        self.controllerName = "AccelerationPID"
        self.controllerType = CONTROLLER_TYPE.ACCELERATIONPID
        self.t0 = time.monotonic() if currentTime is None else currentTime
        self.t = 0.0
        self.t_pre = 0.0
        self.dt = 1e-9

        # Flag to enable/disable integral control 
        self.use_integralTerm = True

        self.e1 = np.zeros(3)
        self.e1[0] = 1.0
        self.e2 = np.zeros(3)
        self.e2[1] = 1.0
        self.e3 = np.zeros(3)
        self.e3[2] = 1.0
        
        self.A = np.zeros((3, 3))

        # Load parameters from file if provided, otherwise use params object or defaults
        if params_file is not None:
            self.param = AccelerationPIDControllerParameters.load_from_json(params_file)
        elif params is not None:
            self.param = params
        else:
            self.param = AccelerationPIDControllerParameters(mass=mass)

            # controller
        # Control errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        
        # Integral errors
        self.eIX = IntegralErrorVec3()  # Position integral error
        self.eIV = IntegralErrorVec3()  # Velocity integral error

###############################################################################################################
    def resetIntegralErrorTerms(self):
        self.eIX.set_zero()
        self.eIV.set_zero()
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
        if controlType is None:
            pos_control=np.array([True, True, True])
            vel_control=np.array([True, True, True])
        else:
            pos_control = np.array(controlType[0]).astype(int)
            vel_control = np.array(controlType[1]).astype(int)
            
        (pos_ned, vel_ned, accel_ned, omega_ned, quat_ned_bodyfrd) = currentBodyState

        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot) = desiredBodyState[0]
        (b1d, b1d_dot, b1d_2dot) = desiredBodyState[1] 

        kX = self.param.kX @ np.diag(pos_control) if (xd is not None) else np.zeros((3,3))
        kIX = self.param.kIX @ np.diag(pos_control) if (xd is not None) else np.zeros((3,3))
        kV = self.param.kV @ np.diag(vel_control | pos_control)
        kIV = self.param.kIV @ np.diag(vel_control)

        self.update_current_time(currentTime=currentData.local_ts if currentData is not None else None)
        self.dt = self.t - self.t_pre

        # Translational error functions
        e3 = self.e3

        mass = self.param.mass
        gravity = self.param.gravity
        
        eX = (xd - pos_ned) * pos_control  if (xd is not None)     else np.zeros(3) # position error - eq (11)
        eV = (xd_dot - vel_ned)                                                          # velocity error - eq (12)

        # Position integral terms
        if self.use_integralTerm:
            self.eIX.integrate(self.param.c1 * eX + eV, self.dt)                 # eq (13)
            self.eIX.error = saturate(self.eIX.error, -self.param.sat_sigmaX, self.param.sat_sigmaX)
            self.eIV.integrate(eV, self.dt)                 # eq (13)
            self.eIV.error = saturate(self.eIV.error, -self.param.sat_sigmaV, self.param.sat_sigmaV)

        # Force 'f' along negative b3-axis -                                 eq (14)
        # This term equals to R.e3

        A =   kX @ eX \
            + kV @ eV \
            + kIX @ self.eIX.error \
            + kIV @ self.eIV.error \
            + xd_2dot 
            # - mass * gravity * e3
             
        self.A = A
        
        obs = np.concatenate((pos_ned, vel_ned, omega_ned))
        
        return A, np.eye(3), np.zeros(3), obs
###############################################################################################################

    def update_current_time(self, currentTime=None):
        """Update the current time since epoch."""
        self.t_pre = self.t

        if currentTime is not None:
            self.t = currentTime - self.t0
        else:
            self.t = time.time() - self.t0
    



