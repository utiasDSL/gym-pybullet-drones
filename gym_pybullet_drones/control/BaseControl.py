import math
import numpy as np
import pybullet as p
from enum import Enum
from scipy.optimize import nnls
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary


######################################################################################################################################################
#### Base control class ##############################################################################################################################
######################################################################################################################################################
class BaseControl(object):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (BaseAviary)                 simulation environment #####################################
    ####################################################################################################
    def __init__(self, env: BaseAviary):
        #### Set general use constants #####################################################################
        self.DRONE_MODEL = env.DRONE_MODEL; self.GRAVITY = env.GRAVITY; self.KF = env.KF; self.KM = env.KM
        self.MAX_THRUST = env.MAX_THRUST; self.MAX_XY_TORQUE = env.MAX_XY_TORQUE; self.MAX_Z_TORQUE = env.MAX_Z_TORQUE
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        self.control_counter = 0

    ####################################################################################################
    #### Wrapper function to compute the control action from obs as returned by BaseAviary.step() ######
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - state ((20,1) array)             current state of the drone #################################
    #### - target_pos ((3,1) array)         desired position ###########################################
    #### - target_rpy ((3,1) array)         desired orientation as roll, pitch, yaw ####################
    #### - target_vel ((3,1) array)         desired velocity ###########################################
    #### - target_ang_vel ((3,1) array)     desired angular velocity ###################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    #### - pos_e ((3,1) array)              current XYZ position error #################################
    #### - yaw_e (float)                    current yaw error ##########################################
    ####################################################################################################
    def computeControlFromState(self, control_timestep, state, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        return self.computeControl(control_timestep=control_timestep,
                                    cur_pos=state[0:3], cur_quat=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16],
                                    target_pos=target_pos, target_vel=target_vel, target_ang_vel=target_ang_vel)

    ####################################################################################################
    #### Compute the control action for a single drone, to be implemented in a child class #############
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - cur_pos ((3,1) array)            current position ###########################################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - cur_vel ((3,1) array)            current velocity ###########################################
    #### - cur_ang_vel ((3,1) array)        current angular velocity ###################################
    #### - target_pos ((3,1) array)         desired position ###########################################
    #### - target_rpy ((3,1) array)         desired orientation as roll, pitch, yaw ####################
    #### - target_vel ((3,1) array)         desired velocity ###########################################
    #### - target_ang_vel ((3,1) array)     desired angular velocity ###################################
    ####################################################################################################
    def computeControl(self, control_timestep, cur_pos, cur_quat, cur_vel, cur_ang_vel,
                        target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        raise NotImplementedError

