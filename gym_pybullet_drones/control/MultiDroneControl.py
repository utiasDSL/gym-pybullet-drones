import math
import numpy as np
import pybullet as p
from enum import Enum
from scipy.optimize import nnls
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.SingleDroneEnv import DroneModel
from gym_pybullet_drones.envs.MultiDroneEnv import MultiDroneEnv
from gym_pybullet_drones.control.SingleDroneControl import ControlType


######################################################################################################################################################
#### Control class ###################################################################################################################################
######################################################################################################################################################
class MultiDroneControl(object):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    def __init__(self, env: MultiDroneEnv, control_type: ControlType=ControlType.PID):
        pass