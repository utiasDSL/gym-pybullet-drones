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
#### A class for user-defined "reward" and "done" functions ##########################################################################################
######################################################################################################################################################


class UserDefinedFunctions(object):

    ####################################################################################################
    #### Initialization with a username string #########################################################
    ####################################################################################################
    def __init__(self, user: str="Default"):
        self.USER = user

    ####################################################################################################
    #### Compute the current state's reward ############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - reward (Float)                   reward value ###############################################
    ####################################################################################################
    def rewardFunction(self, state):
        if self.USER == "Default":
            if state[2] > 0.8: return -1
            elif state[2] > 0.5: return 2000
            elif state[2] > 0.3: return 1000
            elif state[2] > 0.2: return 500
            elif state[2] > 0.15: return 100
            elif state[2] > 0.1: return 10
            else: return -1
        elif self.USER == "Custom": # Use with: >>> env = SingleDroneEnv(user="Custom")
            pass
        else: print("[ERROR] in rewardFunctionExample(), unknown user")

    ####################################################################################################
    #### Evaluate the current state's halting conditions ###############################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    #### - sim_time (Float)                 elapsed simulation time (in seconds) #######################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (Boolean)                   whether the halting conditions of the episode are met ######
    ####################################################################################################
    def doneFunction(self, state, sim_time):
        if self.USER == "Default":
            if np.abs(state[0])>=1 or np.abs(state[1])>=1 or state[2]>=1 \
                        or np.abs(state[7])>=np.pi/3 or np.abs(state[8])>=np.pi/3 \
                        or np.abs(state[10])>=1 or np.abs(state[11])>=1 or np.abs(state[12])>=1 \
                        or np.abs(state[13])>=10*np.pi or np.abs(state[14])>=10*np.pi or np.abs(state[15])>=20*np.pi \
                        or sim_time > 3: 
                done = True
            else: 
                done = False
            return done
        elif self.USER == "Custom": # Use with: >>> env = SingleDroneEnv(user="Custom")
            pass
        else: print("[ERROR] in doneFunction(), unknown user")



