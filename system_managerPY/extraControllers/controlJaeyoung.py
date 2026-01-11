from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3
from common import *

import time
import numpy as np

from math import sin, cos, pi, sqrt

###############################################################################################################
class JaeyoungControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
        self.gravity = 9.81  # Gravitational acceleration (m/s^2)
 
        # Position gains
        self.kX = np.diag([0.8, 0.8, 1.0])  # Position gains
        self.kV = np.diag([0.4, 0.4, 0.4])*2 # Velocity gains
        # Integral gains
        self.kIX = np.diag([0.2, 0.2, .8]) # Position integral gains
        self.kIV = np.diag([0.7, 0.7, 1]) # Velocity integral gains

        k_X = np.diag(self.kX)
        k_V = np.diag(self.kV)  
        # Parameters for Position decoupled-yaw integral control
        self.c1 = np.min([np.sqrt(k_X / self.mass), 
                          4 * k_X * k_V / (k_V**2 + 4 * self.mass * k_X)],axis=0) # Parameters for Position decoupled-yaw integral control
        # self.c1 = 1.0  
        
        # attitude gain
        self.attctrl_tau = 0.7

        self.max_fb_acc = 10
        
        # Integral saturation limits
        self.sat_sigmaX = 2
        self.sat_sigmaV = 2

class JaeyoungController:
    """Controller for the UAV trajectory tracking.
    """
###############################################################################################################

    def __init__(self, mass):

        self.controllerType = "Jaeyoung"
        
        self.feedthrough_enable = False
        self.velocity_yaw_ = True
        
        self.t0 = time.time()
        self.t = 0.0
        self.t_pre = 0.0
        self.dt = 1e-9
        
        self.omegaFactor = 0.1

        # Flag to enable/disable integral control 
        self.use_integralTerm = True

        self.e1 = np.zeros(3)
        self.e1[0] = 1.0
        self.e2 = np.zeros(3)
        self.e2[1] = 1.0
        self.e3 = np.zeros(3)
        self.e3[2] = 1.0

        self.A = np.zeros(3)
        self.param = JaeyoungControllerParameters(mass=mass)
        self.param.mass = mass  # Mass of the rover (kg)
        self.gravity = 9.81  # Gravitational acceleration (m/s^2)
            # controller
        
        # Integral errors
        self.eIX = IntegralErrorVec3()  # Position integral error
        self.eIV = IntegralErrorVec3()  # Velocity integral error

        # self.attitudeController = self.nonLinearGeometricControl
        # self.attitudeController = self.nonLinearAttitudeControl
        self.attitudeController = self.jerkTrackingControl
        
        self.last_acc_d = np.zeros(3)
        self.norm_thrust_const_=0.05
        self.norm_thrust_offset_=0.1
###############################################################################################################
    def resetIntegralErrorTerms(self):
        """Reset the integral error terms."""
        self.eIX.set_zero()
        self.eIV.set_zero()
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None):
        """Position controller to determine desired attitude and angular rates
        to achieve the deisred states.

        This uses the controller defined in "Control of Complex Maneuvers
        for a Quadrotor UAV using Geometric Methods on SE(3)"
        URL: https://arxiv.org/pdf/1003.2005.pdf
        """
        if controlType is None:
            pos_control=True
            vel_control=True
        else:
            pos_control = controlType[0]
            vel_control = controlType[1]
        
        (pos_ned, vel_ned, accel_ned, omega_ned, quat_ned_bodyfrd) = currentBodyState
        # (pos_des_ned, vel_des_ned, accel_des_ned, b1d_ned) = desiredState
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot) = desiredBodyState[0]
        (b1d, b1d_dot, b1d_2dot) = desiredBodyState[1] 

        posFactor = 1 if pos_control else 0
        kX = self.param.kX*posFactor
        kIX = self.param.kIX*posFactor
        kV = self.param.kV
        kIV = self.param.kIV
        
        self.update_current_time()
        self.dt = self.t - self.t_pre
 
        #   /// Compute BodyRate commands using differential flatness
        #   /// Controller based on Faessler 2017

        (pos_ned, vel_ned, accel_ned, omega_ned, quat_ned_bodyfrd) = currentBodyState
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot) = desiredBodyState[0]

        eX = (pos_ned - xd) if pos_control else np.zeros(3)   # position error - eq (11)
        eV = vel_ned - xd_dot                            # velocity error - eq (12)

        # Position integral terms
        self.eIX.integrate(self.param.c1 * eX + eV, self.dt)                 # eq (13)
        self.eIX.error = saturate(self.eIX.error, -self.param.sat_sigmaX, self.param.sat_sigmaX)
        self.eIV.integrate(eV, self.dt)
        self.eIV.error = saturate(self.eIV.error, -self.param.sat_sigmaV, self.param.sat_sigmaV)

        if self.use_integralTerm:
            if not pos_control:
                kIX = np.zeros((3,3))       
            if not vel_control:
                kIV = np.zeros((3,3))  
        else:
            kIX = np.zeros((3,3))
            kIV = np.zeros((3,3))

        # Reference acceleration
        desired_acc = - kX @ eX \
                      - kV @ eV \
                      - kIX @ self.eIX.error \
                      - kIV @ self.eIV.error \
                      - self.e3*self.param.gravity  # feedforward term for trajectory error
        self.A = desired_acc

        R = quat_ned_bodyfrd.to_rotation_matrix()

        zb_dir = -desired_acc / np.linalg.norm(desired_acc)
        proj_xb_dir = np.array([b1d[0], b1d[1], 0.0])
        yb_dir = np.cross(zb_dir, proj_xb_dir) / np.linalg.norm(np.cross(zb_dir, proj_xb_dir))
        xb_dir = np.cross(yb_dir, zb_dir) / np.linalg.norm(np.cross(yb_dir, zb_dir))

        R_desired = np.array([[xb_dir[0], yb_dir[0], zb_dir[0]],
                           [xb_dir[1], yb_dir[1], zb_dir[1]],
                           [xb_dir[2], yb_dir[2], zb_dir[2]]])

        acc_cmd, desired_rate = self.attitudeController(R, R_desired, desired_acc, xd_2dot)
        
        f_total = -self.param.mass * acc_cmd[2]
        return f_total, R_desired, desired_rate
###############################################################################################################

    def update_current_time(self):
        """Update the current time since epoch."""
        self.t_pre = self.t

        self.t = time.time() - self.t0

###############################################################################################################
    def nonLinearGeometricControl(self, rotmat, rotmat_d, acc_d, ref_jerk):
        """Nonlinear geometric control for attitude."""
        # Geometric attitude controller
        # Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
        # of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
        # The original paper inputs moment commands, but for offboard control, angular rate commands are sent

        error_att = 0.5 * vee(rotmat_d.T @ rotmat - rotmat.T @ rotmat_d)
        desired_rate = -self.param.attctrl_tau * error_att
        zb = rotmat[:, 2]
        acc_cmd = np.array([0.0, 0.0, acc_d.dot(zb)])
        return acc_cmd, desired_rate

###############################################################################################################
    def nonLinearAttitudeControl(self, rotmat, rotmat_d, acc_d, ref_jerk):
        """Nonlinear attitude control."""
        # Geometric attitude controller
        # Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
        # attitude control: Technical report. ETH Zurich, 2013.

        q_inv = Quaternion.from_matrix(rotmat).inv()
        qe = q_inv * Quaternion.from_matrix(rotmat_d)
        desired_rate = (2.0 / self.param.attctrl_tau) * np.array([np.sign(qe.w) * qe.x, 
                                                                  np.sign(qe.w) * qe.y, 
                                                                  np.sign(qe.w) * qe.z])

        zb = rotmat_d[:, 2]
        acc_cmd = np.array([0.0, 0.0, acc_d.dot(zb)])
        return acc_cmd, desired_rate

###############################################################################################################
    def jerkTrackingControl(self, rotmat, rotmat_d, acc_d, ref_jerk):
        """Jerk tracking control."""
        # Jerk feedforward control
        # Based on: Lopez, Brett Thomas. Low-latency trajectory planning for high-speed navigation in unknown environments.
        # Diss. Massachusetts Institute of Technology, 2016.
        # Feedforward control from Lopez(2016)

        # Numerical differentiation to calculate jerk
        jerk_fb = (acc_d - self.last_acc_d) / self.dt
        jerk_des = ref_jerk + jerk_fb
        R = rotmat
        zb = R[:, 2]
        jerk_vector = jerk_des / np.linalg.norm(acc_d) - acc_d * np.dot(acc_d, jerk_des) / np.linalg.norm(acc_d)**3
        jerk_quat = Quaternion(w=0.0, x=jerk_vector[0], y=jerk_vector[1], z=jerk_vector[2])
        curr_att = Quaternion.from_matrix(rotmat)
        qd = curr_att.inv() * Quaternion.from_matrix(rotmat_d)
        qd_star = qd.conjugate()
        ratecmd_pre = qd_star * jerk_quat * qd
        desired_rate = np.array([ratecmd_pre.y, -ratecmd_pre.x, 0.0])
        acc_cmd = np.array([0.0, 0.0, acc_d.dot(zb)])
        
        self.last_acc_d = acc_d
        return acc_cmd, desired_rate

###############################################################################################################

###############################################################################################################

###############################################################################################################

###############################################################################################################

###############################################################################################################

###############################################################################################################

###############################################################################################################

###############################################################################################################

    



