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
class SingleDroneUserDefinedFunctions(object):

    ####################################################################################################
    #### Initialization with a username string #########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - client (PyBullet Client Id)      id of the pybullet client to connect to ####################
    #### - gui (Boolean)                    whether PyBullet's GUI is in use ###########################
    #### - user (String)                    identifier string to disambiguate between implementations ##
    ####################################################################################################
    def __init__(self, client, gui=False, user: str="Default"):
        self.CLIENT = client; self.GUI = gui; self.USER = user

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
        else: print("[ERROR] in rewardFunction(), unknown user")

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

    ####################################################################################################
    #### Normalize the 20 values in the simulation state to the [-1,1] range ###########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - state (20-by-1 array)            raw simulation state #######################################
    #### - step_counter (Integer)           current simulation step ####################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    ####################################################################################################
    def clipAndNormalizeState(self, state, step_counter):
        clipped_pos = np.clip(state[0:3], -1, 1)
        clipped_rp = np.clip(state[7:9], -np.pi/3, np.pi/3)
        clipped_vel = np.clip(state[10:13], -1, 1)
        clipped_ang_vel_rp = np.clip(state[13:15], -10*np.pi, 10*np.pi)
        clipped_ang_vel_y = np.clip(state[15], -20*np.pi, 20*np.pi)
        if self.GUI:
            if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it:", step_counter, "in clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of doneFunction()".format(state[0], state[1], state[2]))
            if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it:", step_counter, "in clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of doneFunction()".format(state[7], state[8]))
            if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it:", step_counter, "in clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of doneFunction()".format(state[10], state[11], state[12]))
            if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it:", step_counter, "in clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of doneFunction()".format(state[13], state[14], state[15]))
            if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it:", step_counter, "in clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of doneFunction()".format(state[13], state[14], state[15]))
        normalized_pos = clipped_pos
        normalized_rp = clipped_rp/(np.pi/3)
        normalized_y = state[9]/np.pi
        normalized_vel = clipped_vel
        normalized_ang_vel_rp = clipped_ang_vel_rp/(10*np.pi)
        normalized_ang_vel_y = clipped_ang_vel_y/(20*np.pi)
        return np.hstack([normalized_pos, state[3:7], normalized_rp, normalized_y, normalized_vel, normalized_ang_vel_rp, normalized_ang_vel_y, state[16:20] ]).reshape(20,)

    ####################################################################################################
    #### Add obstacles to the environment from .urdf files #############################################
    ####################################################################################################
    def addObstacles(self):
        if self.USER == "Default":
            p.loadURDF("samurai.urdf", physicsClientId=self.CLIENT)
            p.loadURDF("duck_vhacd.urdf", [-.5,-.5,.05], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("cube_no_rotation.urdf", [-.5,-2.5,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("sphere2.urdf", [0,2,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        elif self.USER == "Custom": # Use with: >>> env = SingleDroneEnv(user="Custom")
            pass
        else: print("[ERROR] in addObstacles(), unknown user")



