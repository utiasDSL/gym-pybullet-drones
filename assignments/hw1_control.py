import math
import numpy as np
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.BaseAviary import BaseAviary

class HW1Control(object):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    def __init__(self, 
                    env: BaseAviary
                    ):
        #### Set general use constants #####################################################################
        self.GRAVITY = env.GRAVITY # M * g
        self.TIMESTEP = env.TIMESTEP
        self.KF = env.KF
        self.KM = env.KM
        self.P_COEFF_POSITION = np.array([1, 1, 1])
        self.I_COEFF_POSITION = np.array([1, 1, 1])
        self.D_COEFF_POSITION = np.array([1, 1, 1])
        self.P_COEFF_ATTITUDE = np.array([1, 1, 1])
        self.I_COEFF_ATTITUDE = np.array([1, 1, 1])
        self.D_COEFF_ATTITUDE = np.array([1, 1, 1])
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        self.control_counter = 0
        self.last_position_error = np.zeros(3)
        self.integral_position_error = np.zeros(3)
        self.last_attitude_error = np.zeros(3)
        self.integral_attitude_error = np.zeros(3)

    ####################################################################################################
    #### Compute the control action ####################################################################
    ####################################################################################################
    def computeControl(self, 
                        current_position, 
                        current_quaternion, 
                        current_velocity, 
                        current_angular_velocity,
                        target_position, 
                        target_velocity=np.zeros(3),
                        target_angular_velocity=np.zeros(3)
                        ):
        ####################################################################################################
        propellers_0_and_3_rpm = np.sqrt(self.GRAVITY/(4*self.KF)) # HOMEWORK CODE
        propellers_1_and_2_rpm = np.sqrt(self.GRAVITY/(4*self.KF)) # HOMEWORK CODE
        ####################################################################################################
        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm, propellers_1_and_2_rpm, propellers_0_and_3_rpm])




