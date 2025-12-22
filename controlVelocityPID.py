from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3
from common import CONTROLLER_TYPE, YAW_COMMAND_TYPE

import time
import numpy as np

from math import sin, cos, pi, sqrt

###############################################################################################################
class VelocityPIDControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)

            # controller
        # Position gains
        commonFactor = 2
        self.kX = np.diag([1.0, 1.0, 1.0])*commonFactor  # Position gains
        self.kV = np.diag([0.8, 0.8, 1.0])*commonFactor # Velocity gains
        # Integral gains
        self.kIX = np.diag([0, 0, 0])*commonFactor # Position integral gains
        self.kIV = np.diag([1, 1, 1])*0 # Velocity integral gains

        self.c1 = 1.0  # Parameters for Position decoupled-yaw integral control
        
        self.sat_sigmaX = 3
        self.sat_sigmaV = 3
        
        self.keYaw = 1.0
        self.keYawRate = 0.01
        self.keYawIntegral = 1.0*0
        self.sat_sigmaYaw = 3
        
###############################################################################################################
class VelocityPIDController:

    def __init__(self, mass, currentTime=None, yawCommandType=YAW_COMMAND_TYPE.NONE):

        self.controllerName = "VelocityPID"
        self.controllerType = CONTROLLER_TYPE.VELOCITYPID
        
        self.yawCommandType = yawCommandType 
        self.t0 = time.monotonic() if currentTime is None else currentTime
        self.t = 0.0
        self.t_pre = 0.0
        self.dt = 1e-9

        # Flag to enable/disable integral control 
        self.use_integralTerm = True

        self.A = np.zeros((3, 3))

        self.param = VelocityPIDControllerParameters(mass=mass)

            # controller
        # Control errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        
        # Integral errors
        self.eIX = IntegralErrorVec3()  # Position integral error
        self.eIV = IntegralErrorVec3()  # Velocity integral error
        
        self.eYaw = 0.0
        self.prev_eYaw = None
        self.eYawRate = 0.0
        self.eYawIntegral = 0.0

        # derivative errors
        self.eDV = np.zeros(3)      # Derivative of velocity error

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

        kX = self.param.kX @ np.diag(pos_control)
        kIX = self.param.kIX @ np.diag(pos_control)
        kV = self.param.kV @ np.diag(vel_control | pos_control)
        kIV = self.param.kIV @ np.diag(vel_control)

        self.update_current_time(currentTime=currentData.local_ts)
        self.dt = self.t - self.t_pre

        # Translational error functions
        
        eX = (xd - pos_ned) * pos_control  if (xd is not None)     else np.zeros(3) # position error - eq (11)
        eV = (xd_dot - vel_ned) #if (vel_control and xd_dot is not None) else np.zeros(3) # velocity error - eq (12)

        # Position integral terms
        if self.use_integralTerm:
            self.eIX.integrate(self.param.c1 * eX + eV, self.dt)                 # eq (13)
            self.eIX.error = saturate(self.eIX.error, -self.param.sat_sigmaX, self.param.sat_sigmaX)
            self.eIV.integrate(eV, self.dt)                 # eq (13)
            self.eIV.error = saturate(self.eIV.error, -self.param.sat_sigmaV, self.param.sat_sigmaV)

        velocity_command =    kX @ eX \
                            + kV @ eV \
                            + kIX @ self.eIX.error \
                            + kIV @ self.eIV.error \
                            + xd_2dot# Velocity Command
                            

        self.A = velocity_command
        
        yaw_command = 0.0
        
        obs = np.concat((pos_ned, vel_ned,omega_ned))
        if self.yawCommandType == YAW_COMMAND_TYPE.RATE:
            b1d_bodyfrd = quat_ned_bodyfrd.inv().rotate_vec(b1d)
            self.eYaw = np.arctan2(b1d_bodyfrd[1], b1d_bodyfrd[0])
            self.eYawRate = (self.eYaw - self.prev_eYaw) / self.dt if self.prev_eYaw is not None and self.dt > 0.0 else 0.0
            self.prev_eYaw = self.eYaw  
            self.eYawIntegral += self.eYaw * self.dt
            self.eYawIntegral = np.clip(self.eYawIntegral, -self.param.sat_sigmaYaw, self.param.sat_sigmaYaw)
            yaw_command = self.param.keYaw * self.eYaw + self.param.keYawRate * self.eYawRate + self.param.keYawIntegral * self.eYawIntegral
            
 # f_total or desired velocity, R_desired, Omega_desired_frd 
        return velocity_command, np.eye(3), np.array([0.0, 0.0, yaw_command]), obs
###############################################################################################################

    def update_current_time(self, currentTime):
        """Update the current time since epoch."""
        self.t_pre = self.t

        self.t = currentTime - self.t0
    



