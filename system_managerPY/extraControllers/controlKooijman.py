from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3

import time
import numpy as np
from numpy.linalg import norm

from math import sin, cos, pi, sqrt

###############################################################################################################
class KooijmanControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)
        self.gravity = 9.81  # Gravitational acceleration (m/s^2)
        self.J = np.diag([0.02, 0.02, 0.04])  # Inertia matrix of the rover
        self.A = np.zeros(3)  

            # controller

        # kp = 10
        # kv = 4
        self.k1 = .6
        self.k2 = .2
        self.kW = 8
        self.kwy = 1
        
        # Position gains
        self.kX = np.diag([10, 10, 10])  # Position gains
        self.kV = np.diag([4, 4, 4]) # Velocity gains
        # Integral gains
        self.kIX = np.diag([0, 0, .8]) # Position integral gains
        self.kIV = np.diag([0.7, 0.7, 1]) # Velocity integral gains

        k_X = np.diag(self.kX)
        k_V = np.diag(self.kV)  
        # Parameters for Position decoupled-yaw integral control
        self.c1 = np.min([np.sqrt(k_X / self.mass), 
                          4 * k_X * k_V / (k_V**2 + 4 * self.mass * k_X)],axis=0) # Parameters for Position decoupled-yaw integral control
        # self.c1 = 1.0  
        
        # Attitude
        self.kW = np.diag([0.8, 0.8, 0.8]) # angular rate gains

        # Integral saturation limits
        self.sat_sigmaX = 2
        self.sat_sigmaV = 2
        
class KooijmanController:
###############################################################################################################

    def __init__(self, mass, use_integralTerm=False):

        self.controllerType = "Kooijman"
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

        self.param = KooijmanControllerParameters(mass=mass)
        
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
        W = omega_ned
        R = quat_ned_bodyfrd.to_rotation_matrix()
        R_dot = R@hat(omega_ned)
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
        
        eX = -(pos_ned - xd) if pos_control else np.zeros(3)   # position error - eq (11)
        eV = -(vel_ned - xd_dot)                            # velocity error - eq (12)

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

        r1 = R[:,0]
        r2 = R[:,1]
        r3 = R[:,2]

        r1_dot = R_dot[:,0]
        r2_dot = R_dot[:,1]
        r3_dot = R_dot[:,2]

        b3 = R@e3
        b3_dot = R_dot@e3

        b = -xd_2dot + gravity*e3

        T_bar = mass*norm(b)
        T_msqrt3 = T_bar / (np.sqrt(3)*mass)
        L_lower = np.array([-T_msqrt3, -T_msqrt3, gravity - T_msqrt3])
        L_upper = np.array([T_msqrt3, T_msqrt3, gravity + T_msqrt3])

        u_bar = xd_2dot
        u = u_bar \
            + kX@eX \
            + kV@eV \
            - kIX @ self.eIX.error \
            - kIV @ self.eIV.error 

        a_bar_ref = gravity*e3 - u
        n_a_bar_ref = norm(a_bar_ref)

        T = mass*n_a_bar_ref
        u_bar_dot = xd_3dot
        v_dot = gravity * e3 - T / mass * b3
        ea = xd_2dot - v_dot
        u_dot = u_bar_dot \
                + kX@eV \
                + kV@ea \
                - kIX @ self.eIV.error 
        a_ref_dot = -u_dot

        n_a_ref_dot = a_bar_ref.T@a_ref_dot / n_a_bar_ref
        T_dot = mass*n_a_ref_dot
        v_2dot = - T_dot / mass * b3 - T / mass * b3_dot
        ea_dot = xd_3dot - v_2dot

        u_bar_2dot = xd_4dot
        u_2dot = u_bar_2dot \
                + kX@ea \
                + kV@ea_dot
        a_ref_2dot = -u_2dot

        r3_bar, r3_bar_dot, r3_bar_2dot = deriv_unit_vector(a_bar_ref, a_ref_dot, a_ref_2dot)

        # phi_bar = desired['yaw']
        # phi_bar_dot = desired['w']
        # phi_bar_2dot = desired['w_dot']

        r_yaw = b1d/np.linalg.norm(b1d) #np.array([-np.sin(phi_bar), np.cos(phi_bar), 0])
        # r_yaw_dot = np.array([
        #     -np.cos(phi_bar)*phi_bar_dot,
        #     -np.sin(phi_bar)*phi_bar_dot,
        #     0
        # ])
        r_yaw_dot = b1d_dot
        # r_yaw_2dot = np.array([
        #     np.sin(phi_bar)*phi_bar_dot**2 + -np.cos(phi_bar)*phi_bar_2dot,
        #     -np.cos(phi_bar)*phi_bar_dot**2 - np.sin(phi_bar)*phi_bar_2dot,
        #     0
        # ])
        r_yaw_2dot = b1d_2dot
        
        num = hat(r_yaw)@r3_bar
        num_dot = hat(r_yaw_dot)@r3_bar + hat(r_yaw)@r3_bar_dot
        num_2dot = hat(r_yaw_2dot)@r3_bar + hat(r_yaw_dot)@r3_bar_dot + hat(r_yaw_dot)@r3_bar_dot + hat(r_yaw)@r3_bar_2dot

        den = s(r_yaw, r3_bar)
        den_dot = s_dot(r_yaw, r3_bar, r_yaw_dot, r3_bar_dot)
        den_2dot = s_2dot(r_yaw, r3_bar, r_yaw_dot, r3_bar_dot, r_yaw_2dot, r3_bar_2dot)

        r1_bar = num/den
        r1_bar_dot = diff_num_den(num, num_dot, den, den_dot)
        r1_bar_2dot = diff2_num_den(num, num_dot, num_2dot, den, den_dot, den_2dot)

        r2_bar = hat(r3_bar)@r1_bar

        u_v = calculate_u_v(r3, r3_bar, r3_bar_dot, r1, self.param)
        u_w = calculate_u_w(r1, r2, r3, r1_bar, r1_bar_dot, r3_bar, self.param)

        R_e, R_r = get_Re_Rr(r3, r3_bar)

        # r3_dot = (np.eye(3) - r3[:,None]@r3[None,:])@u_v
        R_r_dot = get_Rr_dot(r3, r3_dot, r3_bar, r3_bar_dot)
        w_r = vee(R_r.T@R_r_dot)

        R_e_dot = get_Re_dot(r3, r3_dot, r3_bar, r3_bar_dot)
        w_e = vee(R_e.T@R_e_dot)

        W_bar1 = -r2.T@u_v
        W_bar2 = r1.T@u_v

        if abs(r3.T@r3_bar) > 1e-3:
            w1 = r1.T@R_r@R_e.T@r1_bar
            w2 = r2.T@R_r@R_e.T@r1_bar
        else:
            w1 = r1.T@r1_bar
            w2 = r2.T@r2_bar

        beta1 = w2*r3.T@R_r@R_e.T@r1_bar - r1.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar
        beta2 = w1*r3.T@R_r@R_e.T@r1_bar + r2.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar

        if abs(w1) > abs(w2):
            w_r = beta2/w1
        else:
            w_r = beta1/w2

        W_bar = np.array([W_bar1, W_bar2, u_w + w_r])

        r3_dot = R_dot[:,2]
        u_v_dot = calculate_u_v_dot(r3, r3_dot, r3_bar, r3_bar_dot, r3_bar_2dot, r1_dot, self.param)

        u_w_dot = calculate_u_w_dot(r1, r1_dot, r2, r2_dot, r3, r3_dot, 
                                r1_bar, r1_bar_dot, r1_bar_2dot, r3_bar, r3_bar_dot, self.param)

        r3_2dot = (- r3_dot[:,None]@r3[None,:] - r3[:,None]@r3_dot[None,:])@u_v + (np.eye(3) - r3[:,None]@r3[None,:])@u_v_dot

        w1_dot = (r1_dot.T@R_r@R_e.T@r1_bar + r1.T@R_r_dot@R_e.T@r1_bar + 
                r1.T@R_r@R_e_dot.T@r1_bar + r1.T@R_r@R_e.T@r1_bar_dot)

        w2_dot = (r2_dot.T@R_r@R_e.T@r1_bar + r2.T@R_r_dot@R_e.T@r1_bar + 
                r2.T@R_r@R_e_dot.T@r1_bar + r2.T@R_r@R_e.T@r1_bar_dot)

        R_r_2dot = get_Rr_2dot(r3, r3_dot, r3_2dot, r3_bar, r3_bar_dot, r3_bar_2dot)
        R_e_2dot = get_Re_2dot(r3, r3_dot, r3_2dot, r3_bar, r3_bar_dot, r3_bar_2dot)

        w_r_dot = vee(R_r_dot.T@R_r_dot) + vee(R_r.T@R_r_2dot)
        w_e_dot = vee(R_e_dot.T@R_e_dot) + vee(R_e.T@R_e_2dot)

        beta1_dot = (w2_dot*r3.T@R_r@R_e.T@r1_bar + w2*r3_dot.T@R_r@R_e.T@r1_bar + 
                    w2*r3.T@R_r_dot@R_e.T@r1_bar + w2*r3.T@R_r@R_e_dot.T@r1_bar + 
                    w2*r3.T@R_r@R_e.T@r1_bar_dot - r1_dot.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar - 
                    r1.T@R_r_dot@hat(w_r - w_e)@R_e.T@r1_bar - r1.T@R_r@hat(w_r_dot - w_e_dot)@R_e.T@r1_bar - 
                    r1.T@R_r@hat(w_r - w_e)@R_e_dot.T@r1_bar - r1.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar_dot)

        beta2_dot = (w1_dot*r3.T@R_r@R_e.T@r1_bar + w1*r3_dot.T@R_r@R_e.T@r1_bar + 
                    w1*r3.T@R_r_dot@R_e.T@r1_bar + w1*r3.T@R_r@R_e_dot.T@r1_bar + 
                    w1*r3.T@R_r@R_e.T@r1_bar_dot + r2_dot.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar + 
                    r2.T@R_r_dot@hat(w_r - w_e)@R_e.T@r1_bar + r2.T@R_r@hat(w_r_dot - w_e_dot)@R_e.T@r1_bar + 
                    r2.T@R_r@hat(w_r - w_e)@R_e_dot.T@r1_bar + r2.T@R_r@hat(w_r - w_e)@R_e.T@r1_bar_dot)

        if abs(w1) > abs(w2):
            w_r_dot = diff_num_den(beta2, beta2_dot, w1, w1_dot)
        else:
            w_r_dot = diff_num_den(beta1, beta1_dot, w2, w2_dot)

        W1_dot = - r2_dot.T@u_v - r2.T@u_v_dot
        W2_dot = r1_dot.T@u_v + r1.T@u_v_dot
        W3_dot = u_w_dot + w_r_dot
        W_bar_dot = np.array([W1_dot, W2_dot, W3_dot])

        Rd = np.column_stack([r1_bar, r2_bar, r3_bar])

        Wd = W_bar
        Wd_dot = W_bar_dot

        eW = W - Wd
        tau = -self.param.kW @ eW + hat(W)@self.param.J@W + self.param.J@Wd_dot

        calculated = {
            'b3': r3_bar,
            'b3_dot': r3_bar_dot,
            'R': Rd
        }
        eR = 0.5*vee(Rd.T@R - R.T@Rd)

        # return T, tau, error, calculated
        f_total = T
        R_desired = Rd
        Omega_desired = tau
        self.A = a_bar_ref
        return f_total, R_desired, Omega_desired
###############################################################################################################

    def update_current_time(self):
        """Update the current time since epoch."""
        self.t_pre = self.t

        self.t = time.time() - self.t0

###############################################################################################################
def calculate_u_v_dot(v, v_dot, v_bar, v_bar_dot, v_bar_2dot, r1_dot, param):
    k1 = param.k1

    if v.T@v_bar >= 0:
        u_v_FB_dot = k1*v_bar_dot
    elif np.allclose(v, -v_bar):
        u_v_FB_dot = k1*r1_dot
    else:
        num = k1*v_bar
        num_dot = k1*v_bar_dot
        den = s(v, v_bar)
        den_dot = s_dot(v, v_bar, v_dot, v_bar_dot)
        u_v_FB_dot = diff_num_den(num, num_dot, den, den_dot)

    if np.allclose(v, v_bar):
        u_v_FF_dot = v_bar_2dot
    elif np.allclose(v, -v_bar):
        u_v_FF_dot = -v_bar_2dot
    else:
        vxvbar = np.cross(v, v_bar)
        vxvbar_dot = np.cross(v_dot, v_bar) + np.cross(v, v_bar_dot)

        num = vxvbar[:,None]@vxvbar[None,:] - (np.eye(3) - v[:,None]@v[None,:])@v_bar[:,None]@v[None,:]
        num_dot = (vxvbar_dot[:,None]@vxvbar[None,:] + vxvbar[:,None]@vxvbar_dot[None,:] - 
                  (- v_dot[:,None]@v[None,:] - v[:,None]@v_dot[None,:])@v_bar[:,None]@v[None,:] - 
                  (np.eye(3) - v[:,None]@v[None,:])@v_bar_dot[:,None]@v[None,:] - 
                  (np.eye(3) - v[:,None]@v[None,:])@v_bar[:,None]@v_dot[None,:])

        den = s(v, v_bar)*s(v, v_bar)
        den_dot = 2*s(v, v_bar)*s_dot(v, v_bar, v_dot, v_bar_dot)

        theta = num / den
        theta_dot = diff_num_den(num, num_dot, den, den_dot)

        u_v_FF_dot = theta_dot@v_bar_dot + theta@v_bar_2dot

    return u_v_FB_dot + u_v_FF_dot

###############################################################################################################
def calculate_u_v(v, v_bar, v_bar_dot, r1, param):
    k1 = param.k1

    if v.T@v_bar >= 0:
        u_v_FB = k1*v_bar
    elif np.allclose(v, -v_bar):
        u_v_FB = k1*r1
    else:
        u_v_FB = k1*v_bar / s(v, v_bar)

    if np.allclose(v, v_bar):
        u_v_FF = v_bar_dot
    elif np.allclose(v, -v_bar):
        u_v_FF = -v_bar_dot
    else:
        vxvbar = np.cross(v, v_bar)
        theta = 1 / s(v, v_bar)/ s(v, v_bar) * (vxvbar[:,None]@vxvbar[None,:] - (np.eye(3) - v[:,None]@v[None,:])@v_bar[:,None]@v[None,:])
        u_v_FF = theta@v_bar_dot

    return u_v_FB + u_v_FF

###############################################################################################################
def calculate_u_w_dot(r1, r1_dot, r2, r2_dot, r3, r3_dot, r1_bar, r1_bar_dot, r1_bar_2dot, r3_bar, r3_bar_dot, param):
    k2 = param.k2

    R_e, R_r = get_Re_Rr(r3, r3_bar)
    R_r_dot = get_Rr_dot(r3, r3_dot, r3_bar, r3_bar_dot)
    R_e_dot = get_Re_dot(r3, r3_dot, r3_bar, r3_bar_dot)

    w1 = r1.T@R_r@R_e.T@r1_bar
    w1_dot = (r1_dot.T@R_r@R_e.T@r1_bar + r1.T@R_r_dot@R_e.T@r1_bar + 
             r1.T@R_r@R_e_dot.T@r1_bar + r1.T@R_r@R_e.T@r1_bar_dot)

    w2 = r2.T@R_r@R_e.T@r1_bar
    w2_dot = (r2_dot.T@R_r@R_e.T@r1_bar + r2.T@R_r_dot@R_e.T@r1_bar + 
             r2.T@R_r@R_e_dot.T@r1_bar + r2.T@R_r@R_e.T@r1_bar_dot)

    if abs(w1) > abs(w2):
        theta2 = r2.T@R_r@R_e.T@r1_bar_dot
        theta2_dot = (r2_dot.T@R_r@R_e.T@r1_bar_dot + r2.T@R_r_dot@R_e.T@r1_bar_dot + 
                     r2.T@R_r@R_e_dot.T@r1_bar_dot + r2.T@R_r@R_e.T@r1_bar_2dot)
        
        num = theta2
        num_dot = theta2_dot

        den = w1
        den_dot = w1_dot

        u_w_FF_dot = diff_num_den(num, num_dot, den, den_dot)
    else:
        theta1 = -r1.T@R_r@R_e.T@r1_bar_dot
        theta1_dot = (-r1_dot.T@R_r@R_e.T@r1_bar_dot - r1.T@R_r_dot@R_e.T@r1_bar_dot - 
                     r1.T@R_r@R_e_dot.T@r1_bar_dot - r1.T@R_r@R_e.T@r1_bar_2dot)

        num = theta1
        num_dot = theta1_dot

        den = w2
        den_dot = w2_dot

        u_w_FF_dot = diff_num_den(num, num_dot, den, den_dot)

    if w1 >= 0:
        u_w_FB_dot = k2*w2_dot
    elif w1 < 0 and w2 < 0:
        u_w_FB_dot = -k2
    else:
        u_w_FB_dot = k2

    return u_w_FB_dot + u_w_FF_dot

###############################################################################################################
def calculate_u_w(r1, r2, r3, r1_bar, r1_bar_dot, r3_bar, param):
    k2 = param.k2

    R_e, R_r = get_Re_Rr(r3, r3_bar)

    w1 = r1.T@R_r@R_e.T@r1_bar
    w2 = r2.T@R_r@R_e.T@r1_bar

    if abs(w1) > abs(w2):
        theta2 = r2.T@R_r@R_e.T@r1_bar_dot
        u_w_FF = theta2 / w1
    else:
        theta1 = -r1.T@R_r@R_e.T@r1_bar_dot
        u_w_FF = theta1 / w2

    if w1 >= 0:
        u_w_FB = k2*w2
    elif w1 < 0 and w2 < 0:
        u_w_FB = -k2
    else:
        u_w_FB = k2

    return u_w_FB + u_w_FF

###############################################################################################################
def diff_num_den(num, num_dot, den, den_dot):
    return (den*num_dot - num*den_dot) / den**2

###############################################################################################################
def diff2_num_den(num, num_den, num_2dot, den, den_dot, den_2dot):
    numerator = den**2*(den*num_2dot - num*den_2dot) - (den*num_den - num*den_dot)*2*den*den_dot
    denominator = den**4
    return numerator / denominator
    
###############################################################################################################
def get_Re_2dot(v, v_dot, v_2dot, v_bar, v_bar_dot, v_bar_2dot):
    den = s(v_bar, v)
    den_dot = s_dot(v, v_bar, v_dot, v_bar_dot)
    den_2dot = s_2dot(v, v_bar, v_dot, v_bar_dot, v_2dot, v_bar_2dot)

    num = hat(v_bar)@v
    num_dot = hat(v_bar_dot)@v + hat(v_bar)@v_dot
    num_2dot = hat(v_bar_2dot)@v + hat(v_bar_dot)@v_dot + hat(v_bar_dot)@v_dot + hat(v_bar)@v_2dot
    Rrd1 = diff2_num_den(num, num_dot, num_2dot, den, den_dot, den_2dot)

    num1 = (np.eye(3) - v_bar[:,None]@v_bar[None,:])
    num1_dot = -(v_bar_dot[:,None]@v_bar[None,:] + v_bar[:,None]@v_bar_dot[None,:])
    num1_2dot = -(v_bar_2dot[:,None]@v_bar[None,:] + v_bar_dot[:,None]@v_bar_dot[None,:] + 
                 v_bar_dot[:,None]@v_bar_dot[None,:] + v_bar[:,None]@v_bar_2dot[None,:])

    num = -num1@v
    num_dot = -num1_dot@v - num1@v_dot
    num_2dot = - num1_2dot@v - 2*(num1_dot@v_dot) - num1@v_2dot

    Rrd2 = diff2_num_den(num, num_dot, num_2dot, den, den_dot, den_2dot)

    Rrd3 = v_bar_2dot

    return np.column_stack([Rrd1, Rrd2, Rrd3])

###############################################################################################################
def get_Re_dot(v, v_dot, v_bar, v_bar_dot):
    den = s(v_bar, v)
    den_dot = s_dot(v, v_bar, v_dot, v_bar_dot)

    num = hat(v_bar)@v
    num_dot = hat(v_bar_dot)@v + hat(v_bar)@v_dot
    Rrd1 = diff_num_den(num, num_dot, den, den_dot)
    if norm(den) < 1e-3:
        Rrd1 *= 0

    num = -(np.eye(3) - v_bar[:,None]@v_bar[None,:])@v
    num_dot = (v_bar_dot[:,None]@v_bar[None,:] + v_bar[:,None]@v_bar_dot[None,:])@v - (np.eye(3) - v_bar[:,None]@v_bar[None,:])@v_dot
    Rrd2 = diff_num_den(num, num_dot, den, den_dot)
    if norm(den) < 1e-3:
        Rrd2 *= 0

    Rrd3 = v_bar_dot

    return np.column_stack([Rrd1, Rrd2, Rrd3])

###############################################################################################################
def get_Re_Rr(v, v_bar):
    v_barTv = s(v_bar, v)

    R_e = np.column_stack([
        hat(v_bar)@v / v_barTv,
        -(np.eye(3) - v_bar[:,None]@v_bar[None,:])@v / v_barTv,
        v_bar
    ])

    R_r = np.column_stack([
        hat(v_bar)@v / v_barTv,
        (np.eye(3) - v[:,None]@v[None,:])@v_bar / v_barTv,
        v
    ])

    return R_e, R_r

###############################################################################################################
def get_Rr_2dot(v, v_dot, v_2dot, v_bar, v_bar_dot, v_bar_2dot):
    den = s(v_bar, v)
    den_dot = s_dot(v, v_bar, v_dot, v_bar_dot)
    den_2dot = s_2dot(v, v_bar, v_dot, v_bar_dot, v_2dot, v_bar_2dot)

    num = hat(v_bar)@v
    num_dot = hat(v_bar_dot)@v + hat(v_bar)@v_dot
    num_2dot = hat(v_bar_2dot)@v + hat(v_bar_dot)@v_dot + hat(v_bar_dot)@v_dot + hat(v_bar)@v_2dot

    Rrd1 = diff2_num_den(num, num_dot, num_2dot, den, den_dot, den_2dot)

    num = (np.eye(3) - v[:,None]@v[None,:])@v_bar

    num1 = v_dot[:,None]@v[None,:] + v[:,None]@v_dot[None,:]
    num1_dot = v_2dot[:,None]@v[None,:] + 2*(v_dot[:,None]@v_dot[None,:]) + v[:,None]@v_2dot[None,:]

    num_dot = -num1@v_bar + (np.eye(3) - v[:,None]@v[None,:])@v_bar_dot
    num_2dot = (-num1_dot@v_bar - num1@v_bar_dot + 
               (- v_dot[:,None]@v[None,:] - v[:,None]@v_dot[None,:])@v_bar_dot + 
               (np.eye(3) - v[:,None]@v[None,:])@v_bar_2dot)
    Rrd2 = diff2_num_den(num, num_dot, num_2dot, den, den_dot, den_2dot)

    Rrd3 = v_2dot

    return np.column_stack([Rrd1, Rrd2, Rrd3])

###############################################################################################################
def s_2dot(a, b, a_dot, b_dot, a_2dot, b_2dot):
    num = -a.T@b*(a_dot.T@b + a.T@b_dot)
    num_dot = (-a_dot.T@b*(a_dot.T@b + a.T@b_dot) - 
              a.T@b_dot*(a_dot.T@b + a.T@b_dot) - 
              a.T@b*(a_2dot.T@b + 2*a_dot.T@b_dot + a.T@b_2dot))

    den = s(a, b)
    den_dot = s_dot(a, b, a_dot, b_dot)

    return diff_num_den(num, num_dot, den, den_dot)

###############################################################################################################
def get_Rr_dot(v, v_dot, v_bar, v_bar_dot):
    den = s(v_bar, v)
    den_dot = s_dot(v, v_bar, v_dot, v_bar_dot)

    num = hat(v_bar)@v
    num_dot = hat(v_bar_dot)@v + hat(v_bar)@v_dot
    Rrd1 = diff_num_den(num, num_dot, den, den_dot)
    if norm(den) < 1e-3:
        Rrd1 *= 0

    num = (np.eye(3) - v[:,None]@v[None,:])@v_bar
    num_dot = -(v_dot[:,None]@v[None,:] + v[:,None]@v_dot[None,:])@v_bar + (np.eye(3) - v[:,None]@v[None,:])@v_bar_dot
    Rrd2 = diff_num_den(num, num_dot, den, den_dot)
    if norm(den) < 1e-3:
        Rrd2 *= 0

    Rrd3 = v_dot

    return np.column_stack([Rrd1, Rrd2, Rrd3])

###############################################################################################################
def s_dot(a, b, a_dot, b_dot):
    return -a.T@b*(a_dot.T@b + a.T@b_dot) / s(a, b)

###############################################################################################################
def s(a, b):
    return np.sqrt(1.0 - (a.T@b)**2)

###############################################################################################################
def saturate_fM(f, M, param):
    thr = fM_to_thr(f, M, param)

    max_f = 8
    min_f = 0.1

    for i in range(4):
        if thr[i] > max_f:
            thr[i] = max_f
        elif thr[i] < min_f:
            thr[i] = min_f

    return thr_to_fM(thr, param)

###############################################################################################################
def saturate_u(u, a, b):
    u_sat = np.zeros(3)

    for i in range(3):
        ui = u[i]
        ai = a[i]
        bi = b[i]
        
        e = 0.01
        e_upper = (bi - ai) / 2
        if e > e_upper:
            e = e_upper
        
        if ai + e < ui and ui < bi - e:
            u_sat[i] = u[i]
        elif ui <= ai - e:
            u_sat[i] = ai
        elif bi + e <= ui:
            u_sat[i] = bi
        elif ai - e < ui and ui <= ai + e:
            u_sat[i] = ui + 1 / (4*e) * (ui - (ai + e))**2
        elif bi - e <= ui and ui < bi + e:
            u_sat[i] = ui - 1 / (4*e) * (ui - (bi - e))**2
    
    return u_sat
###############################################################################################################



