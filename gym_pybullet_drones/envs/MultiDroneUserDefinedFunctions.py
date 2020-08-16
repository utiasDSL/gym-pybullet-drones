import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import gym
from gym import error, spaces, utils
from gym.utils import seeding


######################################################################################################################################################
#### A class for user-defined "reward" and "done" functions, state/obs normalization, etc. ###########################################################
######################################################################################################################################################
class MultiDroneUserDefinedFunctions(object):

    ####################################################################################################
    #### Initialization with a username string #########################################################
    ####################################################################################################
    def __init__(self, client, gui=False, user: str="Default"):
        self.CLIENT = client; self.GUI = gui; self.USER = user