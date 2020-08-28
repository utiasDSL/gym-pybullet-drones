import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from enum import Enum


######################################################################################################################################################
#### Enumeration of ProblemSpecificFunctions implementations #########################################################################################
######################################################################################################################################################
class Problem(Enum):
    DEFAULT = 0              # Default "reward" and "done" functions, state normalization, obstacles (i.e. fly upwards from the origin)
    CUSTOM = 1               # Placeholder for custom "reward" and "done" functions, state normalization, obstacles, etc.
    MA_FLOCK = 2             # "reward" functions for multi-agent flocking task

######################################################################################################################################################
#### A class for problem-specific "reward" and "done" functions, state normalization, etc. ###########################################################
######################################################################################################################################################
class ProblemSpecificFunctions(object):

    ####################################################################################################
    #### Initialization with the desired problem #######################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - client (PyBullet client id)      id of the pybullet client to connect to ####################
    #### - gui (bool)                       whether PyBullet's GUI is in use ###########################
    #### - problem (Problem)                to disambiguate between function implementations ###########
    ####################################################################################################
    def __init__(self, client, gui=False, problem: Problem=Problem.DEFAULT):
        self.CLIENT = client; self.GUI = gui; self.PROBLEM = problem

    ####################################################################################################
    #### Compute the current state's reward ############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - normalized state ((20,1) array)  clipped and normalized simulation state ####################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - reward (float)                   reward value ###############################################
    ####################################################################################################
    def rewardFunction(self, obs):
        if self.PROBLEM==Problem.DEFAULT:
            if obs[2] > 0.8: return -1
            elif obs[2] > 0.5: return 2000
            elif obs[2] > 0.3: return 1000
            elif obs[2] > 0.2: return 500
            elif obs[2] > 0.15: return 100
            elif obs[2] > 0.1: return 10
            else: return -1
        elif self.PROBLEM==Problem.CUSTOM:
            pass
        elif self.PROBLEM==Problem.MA_FLOCK:
            n_drones = len(obs)     # obs here is dictionary of the form {"i":{"state": Box(20,), 
                                    # "neighbors": MultiBinary(NUM_DRONES)}}
            # parse velocity and position
            vel = np.zeros((1, n_drones, 3))
            pos = np.zeros((1, n_drones, 3))
            for i in range(n_drones):
                pos[0][i][0] = obs[str(i)]["state"][0]
                pos[0][i][1] = obs[str(i)]["state"][1]
                pos[0][i][2] = obs[str(i)]["state"][2]
                vel[0][i][0] = obs[str(i)]["state"][10]
                vel[0][i][1] = obs[str(i)]["state"][11]
                vel[0][i][2] = obs[str(i)]["state"][12]
            # compute metrics
            # velocity alignment
            ali = 0
            EPSILON = 1e-3  # avoid divide by zero
            linear_vel_norm = np.linalg.norm(vel, axis=2)
            for i in range(n_drones):
                for j in range(n_drones):
                    if j != i:
                        d = np.einsum('ij,ij->i', vel[:, i, :], vel[:, j, :])
                        ali += (d / (linear_vel_norm[:, i] + EPSILON) / (linear_vel_norm[:, j] + EPSILON))
            ali /= (n_drones * (n_drones - 1))
            # flocking speed
            cof_v = np.mean(vel, axis=1)  # centre of flock speed
            avg_flock_linear_speed = np.linalg.norm(cof_v, axis=-1)
            # spacing
            whole_flock_spacing = []
            for i in range(n_drones):
                flck_neighbor_pos = np.delete(pos, [i], 1)
                drone_neighbor_pos_diff = flck_neighbor_pos - np.reshape(pos[:, i, :], (pos[:, i, :].shape[0], 1, -1))
                drone_neighbor_dis = np.linalg.norm(drone_neighbor_pos_diff, axis=-1)
                drone_spacing = np.amin(drone_neighbor_dis, axis=-1)
                whole_flock_spacing.append(drone_spacing)
            whole_flock_spacing = np.stack(whole_flock_spacing, axis=-1)
            avg_flock_spacing = np.mean(whole_flock_spacing, axis=-1)
            var_flock_spacing = np.var(whole_flock_spacing, axis=-1)
            # flocking metrics
            FLOCK_SPACING_MIN = 1.0
            FLOCK_SPACING_MAX = 3.0
            if FLOCK_SPACING_MIN < avg_flock_spacing[0] < FLOCK_SPACING_MAX:
                avg_flock_spac_rew = 0.0
            else:
                avg_flock_spac_rew = min(math.fabs(avg_flock_spacing[0] - FLOCK_SPACING_MIN),
                                         math.fabs(avg_flock_spacing[0] - FLOCK_SPACING_MAX))
            reward = ali[0] + avg_flock_linear_speed[0] - avg_flock_spac_rew - var_flock_spacing[0]
            return reward
        else: print("[ERROR] in ProblemSpecificFunctions.rewardFunction(), unknown Problem")

    ####################################################################################################
    #### Evaluate the current state's stopping conditions ##############################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - normalized state ((20,1) array)  clipped and normalized simulation state ####################
    #### - sim_time (float)                 elapsed simulation time (in seconds) #######################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (bool)                      whether the stopping conditions of the episode are met #####
    ####################################################################################################
    def doneFunction(self, obs, sim_time):
        if self.PROBLEM==Problem.DEFAULT:
            if np.abs(obs[0])>=1 or np.abs(obs[1])>=1 or obs[2]>=1 \
                        or np.abs(obs[7])>=np.pi/3 or np.abs(obs[8])>=np.pi/3 \
                        or np.abs(obs[10])>=1 or np.abs(obs[11])>=1 or np.abs(obs[12])>=1 \
                        or np.abs(obs[13])>=10*np.pi or np.abs(obs[14])>=10*np.pi or np.abs(obs[15])>=20*np.pi \
                        or sim_time > 3: return True
            else: return False
        elif self.PROBLEM==Problem.CUSTOM:
            pass
        else: print("[ERROR] in ProblemSpecificFunctions.doneFunction(), unknown Problem")

    ####################################################################################################
    #### Normalize the 20 values in the simulation state to the [-1,1] range ###########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - state ((20,1) array)             raw simulation stat data  ##################################
    #### - step_counter (int)               current simulation step ####################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - normalized state ((20,1) array)  clipped and normalized simulation state ####################
    ####################################################################################################
    def clipAndNormalizeState(self, state, step_counter):
        if self.PROBLEM==Problem.DEFAULT:
            clipped_pos = np.clip(state[0:3], -1, 1)
            clipped_rp = np.clip(state[7:9], -np.pi/3, np.pi/3)
            clipped_vel = np.clip(state[10:13], -1, 1)
            clipped_ang_vel_rp = np.clip(state[13:15], -10*np.pi, 10*np.pi)
            clipped_ang_vel_y = np.clip(state[15], -20*np.pi, 20*np.pi)
            if self.GUI:
                if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it", step_counter, "in ProblemSpecificFunctions.clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of ProblemSpecificFunctions.doneFunction()".format(state[0], state[1], state[2]))
                if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it", step_counter, "in ProblemSpecificFunctions.clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of ProblemSpecificFunctions.doneFunction()".format(state[7], state[8]))
                if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it", step_counter, "in ProblemSpecificFunctions.clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of ProblemSpecificFunctions.doneFunction()".format(state[10], state[11], state[12]))
                if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it", step_counter, "in ProblemSpecificFunctions.clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of ProblemSpecificFunctions.doneFunction()".format(state[13], state[14], state[15]))
                if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it", step_counter, "in ProblemSpecificFunctions.clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of ProblemSpecificFunctions.doneFunction()".format(state[13], state[14], state[15]))
            normalized_pos = clipped_pos
            normalized_rp = clipped_rp/(np.pi/3)
            normalized_y = state[9]/np.pi
            normalized_vel = clipped_vel
            normalized_ang_vel_rp = clipped_ang_vel_rp/(10*np.pi)
            normalized_ang_vel_y = clipped_ang_vel_y/(20*np.pi)
            return np.hstack([normalized_pos, state[3:7], normalized_rp, normalized_y, normalized_vel, normalized_ang_vel_rp, normalized_ang_vel_y, state[16:20] ]).reshape(20,)
        elif self.PROBLEM==Problem.CUSTOM:
            pass
        else: print("[ERROR] in ProblemSpecificFunctions.clipAndNormalizeState(), unknown Problem")

    ####################################################################################################
    #### Add obstacles to the environment from .urdf files #############################################
    ####################################################################################################
    def addObstacles(self):
        if self.PROBLEM==Problem.DEFAULT:
            p.loadURDF("samurai.urdf", physicsClientId=self.CLIENT)
            p.loadURDF("duck_vhacd.urdf", [-.5,-.5,.05], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("cube_no_rotation.urdf", [-.5,-2.5,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("sphere2.urdf", [0,2,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        elif self.PROBLEM==Problem.CUSTOM:
            pass
        else: print("[ERROR] in ProblemSpecificFunctions.addObstacles(), unknown Problem")



