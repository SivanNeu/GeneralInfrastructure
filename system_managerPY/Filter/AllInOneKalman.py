from Filter.SE3_se3_R3_EqF import SE3_se3_R3_EqF
from Filter.IEKF import IEKF
from Filter.LIEKF import LIEKF
from Filter.TF_IEKF import TF_IEKF
from Filter.TF_LIEKF import TF_LIEKF
from Filter.MEKF import MEKF
from Filter.SE23_se3_EqF import SE23_se3_EqF
from Filter.SE23_se23_EqF import SE23_se23_EqF

import numpy as np
from numpy import pi

class AllInOneKalman:
    def __init__(self, initial_att=None, initial_vel=None, initial_pos=None):
        self.filter = None
        self.last_time = 0
        self.curr_time = 0

        self.omega_noise = 1.0e-1 * pi / 180
        self.acc_noise = 5.0e-2
        self.tau_noise = 1.0e-3
        self.virtual_noise = 1e-9
        self.meas_noise = 0.2        
        
        initial_att_noise = 2.0
        initial_vel_noise = 2.0
        initial_pos_noise = 2.0
        initial_bias_noise = 0.05
        propagationonly = False
        equivariant_output = False
        curvature_correction = False
        measure_b_mu = False
        filter_args = [initial_att_noise, initial_vel_noise, initial_pos_noise, initial_bias_noise, \
                       propagationonly, equivariant_output, curvature_correction, measure_b_mu]

        self.iekf = IEKF(*filter_args[:-2])
        self.tfiekf = TF_IEKF(*filter_args[:-2])
        self.se3_R3_EqF = SE3_se3_R3_EqF(*(filter_args[:-3]+[filter_args[-2]]))
        self.se23_se3_EqF = SE23_se3_EqF(*filter_args[:-1])
        self.se23_se23_EqF = SE23_se23_EqF(*filter_args)
        self.tf_liekf = TF_LIEKF(*filter_args[:-2])
        self.liekf = LIEKF(*filter_args[:-2])
        self.mekf = MEKF(*filter_args[:-3])
        
        self.filters = {
            'iekf': self.iekf,
            # 'tfiekf': self.tfiekf,
            # 'se3_R3_EqF': self.se3_R3_EqF,
            # 'se23_se3_EqF': self.se23_se3_EqF,
            'se23_se23_EqF': self.se23_se23_EqF,
            # 'tf_liekf': self.tf_liekf,
            # 'liekf': self.liekf,
            # 'mekf': self.mekf
        }
        
        self.filter_energy = {'iekf': [], 'tfiekf': [], 'se3_R3_EqF': [], 'se23_se3_EqF': [], 'se23_se23_EqF': [], 'tf_liekf': [], 'liekf': [], 'mekf': [] }
        self.filter_energy_nav = {'iekf': [], 'tfiekf': [], 'se3_R3_EqF': [], 'se23_se3_EqF': [], 'se23_se23_EqF': [], 'tf_liekf': [], 'liekf': [], 'mekf': [] }
        self.filter_energy_pose = {'iekf': [], 'tfiekf': [], 'se3_R3_EqF': [], 'se23_se3_EqF': [], 'se23_se23_EqF': [], 'tf_liekf': [], 'liekf': [], 'mekf': [] }
        self.filter_energy_bias = {'iekf': [], 'tfiekf': [], 'se3_R3_EqF': [], 'se23_se3_EqF': [], 'se23_se23_EqF': [], 'tf_liekf': [], 'liekf': [], 'mekf': [] }
        
        self.ground_truth_ned = np.zeros((0, 10))  # [t, pos, vel, att]

    def predict(self, t, acc, omega):  
        # acc and omega are in the body frame - RFU - right-forward-up
        # which is "parallel" to ENU - east-north-up
        if self.last_time is None:
            self.last_time = t
        dt = t - self.curr_time
        if abs(dt) <= 0:
            return
        self.last_time = self.curr_time
        self.curr_time = t      
        vel = np.array([np.hstack((omega, acc))]).T
        for filter in self.filters.values():
            try:
                filter.propagate(t, vel, self.omega_noise, self.acc_noise, self.tau_noise, 0.0)
            except ValueError as e:
                print('Filter propagation Error')

    def update(self, t, pos_enu):
        dt = t - self.curr_time 
        if abs(dt) < 0:
            return
        if dt > 0:
            self.last_time = self.curr_time
            self.curr_time = t      
        
        for i in range(len(self.filters.values())):
            filter = list(self.filters.values())[i]
            try:
                dt = self.curr_time - self.last_time
                nis = filter.update(np.array([]), self.omega_noise, self.acc_noise, self.tau_noise, 0.0, pos_enu, self.meas_noise, dt, False)
                self.filter_energy[list(self.filters.keys())[i]].append(nis)
            except ValueError as e:
                nis = filter.update(np.array([]), self.omega_noise, self.acc_noise, self.tau_noise, 0.0, pos_enu, self.meas_noise, dt, False)
                print('Filter GNSS update Error '+filter.name)

            
    def update_ground_truth(self, t, pos, vel, att):
        self.ground_truth_ned.append(np.hstack((t, pos, vel, att)))
        
    def getEstimate(self):
        estimates = {}
        for i in range(len(self.filters.values())):
            filter = list(self.filters.values())[i]
            estimates[list(self.filters.keys())[i]] = filter.getEstimate()

        return estimates
