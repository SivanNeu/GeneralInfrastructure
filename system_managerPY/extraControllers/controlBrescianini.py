from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3
from common import Quaternion

import time
import numpy as np

from math import sin, cos, pi, sqrt

###############################################################################################################
class BrescianiniControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
        self.gravity = 9.81  # Gravitational acceleration (m/s^2)
        self.J = np.diag([0.04, 0.04, 0.08])  # Inertia matrix of the rover

            # controller

        # Position gains
        self.kX = np.diag([1.0, 1.0, 1.0])*1  # Position gains
        self.kV = np.diag([1.4, 1.4, 1.4])*3 # Velocity gains
        # Integral gains
        self.kIX = np.diag([.3, .3, .8]) # Position integral gains
        self.kIV = np.diag([0.2, 0.2, .2]) # Velocity integral gains

       # Parameters for Position decoupled-yaw integral control
        k_X = np.diag(self.kX)
        k_V = np.diag(self.kV)  
        self.c1 = np.min([np.sqrt(k_X / self.mass), 
                          4 * k_X * k_V / (k_V**2 + 4 * self.mass * k_X)])# Parameters for Position decoupled-yaw integral control
        self.c1 = 1.0  # Parameters for Position decoupled-yaw integral control

        # Attitude

        self.kp_xy = 0.3
        self.kd_xy = 0.2
        self.kp_z = 0.7
        self.kd_z = 0.3

        # Integral saturation limits
        self.sat_sigmaX = 2
        self.sat_sigmaV = 2

        
###############################################################################################################
class BrescianiniController:

    def __init__(self, mass, use_integralTerm=True):

        self.controllerType = "Brescianini"
        self.t0 = time.time()
        self.t = 0.0
        self.t_pre = 0.0
        self.dt = 1e-9

        # Flag to enable/disable integral control 
        self.use_integralTerm = use_integralTerm

        self.e1 = np.zeros(3)
        self.e1[0] = 1.0
        self.e2 = np.zeros(3)
        self.e2[1] = 1.0
        self.e3 = np.zeros(3)
        self.e3[2] = 1.0
        self.A = np.zeros(3)

        self.param = BrescianiniControllerParameters(mass=mass)

            # controller
        
        # Integral errors
        self.eIX = IntegralErrorVec3()  # Position integral error
        self.eIV = IntegralErrorVec3()  # Velocity integral error
        self.eIR = IntegralErrorVec3()  # Attitude integral error

###############################################################################################################
    def resetIntegralErrorTerms(self):
        """Reset the integral error terms."""
        self.eIX.set_zero()
        self.eIV.set_zero()
        self.eIR.set_zero()
        
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
        R = quat_ned_bodyfrd.to_rotation_matrix()
        W = omega_ned
        # (pos_des_ned, vel_des_ned, accel_des_ned, b1d_ned) = desiredState
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot) = desiredBodyState[0]
        (b1d, b1d_dot, b1d_2dot) = desiredBodyState[1] 
    
        e3 = self.e3

        mass = self.param.mass
        gravity = self.param.gravity
        kV = self.param.kV
        kIV = self.param.kIV

        posFactor = 1 if pos_control else 0
        kX = self.param.kX*posFactor
        kIX = self.param.kIX*posFactor

        self.update_current_time()
        self.dt = self.t - self.t_pre

        # Translational error functions
        
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

        # Force 'f' along negative b3-axis -                                 eq (14)
        # This term equals to R.e3

        A = - kX @ eX \
            - kV @ eV \
            - kIX @ self.eIX.error \
            - kIV @ self.eIV.error \
            - mass*gravity*e3 \
            + mass*xd_2dot

        b3 = R @ e3
        f_total = -A @ b3
        ea = gravity*e3 - f_total/mass*b3 - xd_2dot
        A_dot = -kX @ eV \
                - kV @ ea \
                + mass*xd_3dot

        b3_dot = R @ hat(W) @ e3
        f_dot = -A_dot @ b3 - A @ b3_dot
        eb = - f_dot/mass*b3 \
             - f_total/mass*b3_dot \
             - xd_3dot
        A_ddot = -kX @ ea \
                - kV @ eb \
                + mass*xd_4dot

        b3c, b3c_dot, b3c_ddot = deriv_unit_vector(-A, -A_dot, -A_ddot)

        A2 = -hat(b1d) @ b3c
        A2_dot = -hat(b1d_dot) @ b3c - hat(b1d) @ b3c_dot
        A2_ddot = (-hat(b1d_2dot) @ b3c - 2*hat(b1d_dot) @ b3c_dot 
                - hat(b1d) @ b3c_ddot)

        b2c, b2c_dot, b2c_ddot = deriv_unit_vector(A2, A2_dot, A2_ddot)

        b1c = hat(b2c) @ b3c
        b1c_dot = hat(b2c_dot) @ b3c + hat(b2c) @ b3c_dot
        b1c_ddot = (hat(b2c_ddot) @ b3c + 2*hat(b2c_dot) @ b3c_dot 
                + hat(b2c) @ b3c_ddot)

        Rc = np.column_stack([b1c, b2c, b3c])
        Rc_dot = np.column_stack([b1c_dot, b2c_dot, b3c_dot])
        Rc_ddot = np.column_stack([b1c_ddot, b2c_ddot, b3c_ddot])

        Wc = vee(Rc.T @ Rc_dot)
        Wc_dot = vee(Rc.T @ Rc_ddot - hat(Wc) @ hat(Wc))

        # Run attitude controller
        M, _ = self.attitude_control_brescianini(R, W, Rc, Wc, Wc_dot, self.param)
        eR = 0.5 * vee(Rc.T @ R - R.T @ Rc)
        
        R_desired=Rc
        Omega_desired=-M  #-eR*.2  #-M
  
        return f_total, R_desired, Omega_desired
###############################################################################################################

    def update_current_time(self):
        """Update the current time since epoch."""
        self.t_pre = self.t

        self.t = time.time() - self.t0
    
########################################################################################################

    def attitude_control_brescianini(self, R, w, Rd, wd, wd_dot, param):
        """Attitude controller."""
        J = self.param.J
        kp_xy = self.param.kp_xy
        kp_z = self.param.kp_z
        kd_xy = self.param.kd_xy
        kd_z = self.param.kd_z

        q = Quaternion.from_matrix(R)
        qd = Quaternion.from_matrix(Rd)
        qe = qd * q.conjugate()

        wd_bar = qe.to_rotation_matrix().T @ wd
        we = wd_bar - w
        wd_bar_dot = hat(we) @ wd_bar + qe.to_rotation_matrix().T @ wd_dot

        qe = qe.conjugate()
        q0, q1, q2, q3 = qe.w, qe.x, qe.y, qe.z

        q0q3 = np.sqrt(q0*q0 + q3*q3)
        B = np.array([
            q0*q0 + q3*q3,
            q0*q1 - q2*q3,
            q0*q2 + q1*q3,
            0
        ])
        qe_red = B / q0q3
        qe_yaw = np.array([q0, 0, 0, q3]) / q0q3

        tilde_qe_red = qe_red[1:]
        tilde_qe_yaw = qe_yaw[1:]

        tau_ff = J @ wd_bar_dot - hat(J @ w) @ w

        Kd = np.diag([kd_xy, kd_xy, kd_z])
        tau = kp_xy * tilde_qe_red \
            + kp_z * np.sign(q0) * tilde_qe_yaw #\
            # + Kd @ we #\
            # + tau_ff

        error = {
            'qe': qe,
            'we': we
        }

        return tau, error



