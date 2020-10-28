import numpy as np
from gym import spaces

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import BaseSingleAgentAviary


######################################################################################################################################################
#### Single drone environment class for reinforcement learning applications (in this implementation, taking off from the origin) #####################
######################################################################################################################################################
class TakeoffAviary(BaseSingleAgentAviary):

    ####################################################################################################
    #### Initialize the environment ####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - drone_model (DroneModel)         desired drone type (associated to an .urdf file) ###########
    #### - num_drones (int)                 desired number of drones in the aviary #####################
    #### - neighbourhood_radius (float)     used to compute the drones' adjacency matrix, in meters ####
    #### - initial_xyzs ((3,1) array)       initial XYZ position of the drones #########################
    #### - initial_rpys ((3,1) array)       initial orientations of the drones (radians) ###############
    #### - physics (Physics)                desired implementation of physics/dynamics #################
    #### - freq (int)                       the frequency (Hz) at which the physics engine advances ####
    #### - aggregate_phy_steps (int)        number of physics updates within one call of .step() #######
    #### - gui (bool)                       whether to use PyBullet's GUI ##############################
    #### - record (bool)                    whether to save a video of the simulation ##################
    #### - obstacles (bool)                 whether to add obstacles to the simulation #################
    #### - user_debug_gui (bool)            whether to draw the drones' axes and the GUI sliders #######
    #### ...
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1,
                    neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=5,
                    gui=False, record=False, obstacles=True, user_debug_gui=False, img_obs=False, dyn_input=False):
        super().__init__(drone_model=drone_model, num_drones=num_drones, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui,
                            img_obs=img_obs, dyn_input=dyn_input)

    ####################################################################################################
    #### Compute the current reward value(s) ###########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - reward (..)                      the reward(s) associated to the current obs/state ##########
    ####################################################################################################
    def _computeReward(self, obs):
        #
        if self.IMG_OBS: obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
        #
##### DEV
#########
        # obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
        # if obs[2]>0.75 and obs[2]<.9 and np.abs(obs[0])<0.5 and np.abs(obs[1])<0.5: return 1
        # if obs[2]>0.75 and obs[2]<.9: return 1
        # else: return 0
        # return -np.abs(0.75-obs[2])**2
        # return -np.linalg.norm(np.array([0,0,0.75])-obs[0:3])**2
        obs = self._getDroneStateVector(0); norm_obs = self._clipAndNormalizeState(obs)
        MAX_XYZ = 5
        target = np.array([0,0,1]) / MAX_XYZ
        return -10 * np.linalg.norm(target-norm_obs[0:3])**2 # \
                    # -1 * norm_obs[7]**2 -1 * norm_obs[8]**2 \
                    # -1 * norm_obs[15]**2 #\
                    # -1 * norm_obs[10]**2 -1 * norm_obs[11]**2 -1 * norm_obs[12]**2 \
                    # -1 * norm_obs[13]**2 -1 * norm_obs[14]**2 #\


    ####################################################################################################
    #### Compute the current done value(s) #############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (..)                        the done value(s) associated to the current obs/state ######
    ####################################################################################################
    def _computeDone(self, norm_obs):
        #
        if self.IMG_OBS: norm_obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
        #
##### DEV
#########
        # norm_obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
        # if np.abs(norm_obs[0])>=1 or np.abs(norm_obs[1])>=1 or np.abs(norm_obs[2])>=1 \
        #         or np.abs(norm_obs[7])>=1 or np.abs(norm_obs[8])>=1 \
        #         or self.step_counter/self.SIM_FREQ > 5: 
        #     return True
        # obs = self._getDroneStateVector(0); norm_obs = self._clipAndNormalizeState(obs)
        # if np.abs(norm_obs[0])>=1 or np.abs(norm_obs[1])>=1 or np.abs(norm_obs[2])>=1 \
        #         or self.step_counter/self.SIM_FREQ > 5: 
        #     return True
        if self.step_counter/self.SIM_FREQ > 5: return True
        else: return False

    ####################################################################################################
    #### Compute the current info dict(s) ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - info (..)                        the info dict(s) associated to the current obs/state #######
    ####################################################################################################
    def _computeInfo(self, obs):
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years

    ####################################################################################################
    #### Normalize the 20 values in the simulation state to the [-1,1] range ###########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - state ((20,1) array)             raw simulation state #######################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - normalized state ((20,1) array)  clipped and normalized simulation state ####################
    ####################################################################################################
    def _clipAndNormalizeState(self, state):
        MAX_XYZ = 5
        MAX_PITCH_ROLL = np.pi
        MAX_VEL = 10
        MAX_PITCH_ROLL_VEL = 10*np.pi
        MAX_YAW_VEL = 20*np.pi
        #
        clipped_pos = np.clip(state[0:3], -MAX_XYZ, MAX_XYZ)
        clipped_rp = np.clip(state[7:9], -MAX_PITCH_ROLL, MAX_PITCH_ROLL)
        clipped_vel = np.clip(state[10:13], -MAX_VEL, MAX_VEL)
        clipped_ang_vel_rp = np.clip(state[13:15], -MAX_PITCH_ROLL_VEL, MAX_PITCH_ROLL_VEL)
        clipped_ang_vel_y = np.clip(state[15], -MAX_YAW_VEL, MAX_YAW_VEL)
        #
        if self.GUI: self._clipAndNormalizeStateWarning(state, clipped_pos, clipped_rp, clipped_vel, clipped_ang_vel_rp, clipped_ang_vel_y)
        #
        normalized_pos = clipped_pos / MAX_XYZ
        normalized_rp = clipped_rp / MAX_PITCH_ROLL
        normalized_y = state[9] / np.pi
        normalized_vel = clipped_vel / MAX_VEL
        normalized_ang_vel_rp = clipped_ang_vel_rp / MAX_PITCH_ROLL_VEL
        normalized_ang_vel_y = clipped_ang_vel_y / MAX_YAW_VEL
        #
        return np.hstack([normalized_pos, state[3:7], normalized_rp, normalized_y, normalized_vel, normalized_ang_vel_rp, normalized_ang_vel_y, state[16:20] ]).reshape(20,)

    ####################################################################################################
    #### Print a warning if any of the 20 values in a state vector is out of the normalization range ###
    ####################################################################################################
    def _clipAndNormalizeStateWarning(self, state, clipped_pos, clipped_rp, clipped_vel, clipped_ang_vel_rp, clipped_ang_vel_y):
        if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it", self.step_counter, "in TakeoffAviary._clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of TakeoffAviary._computeDone()".format(state[0], state[1], state[2]))
        if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it", self.step_counter, "in TakeoffAviary._clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of TakeoffAviary._computeDone()".format(state[7], state[8]))
        if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it", self.step_counter, "in TakeoffAviary._clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of TakeoffAviary._computeDone()".format(state[10], state[11], state[12]))
        if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it", self.step_counter, "in TakeoffAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of TakeoffAviary._computeDone()".format(state[13], state[14], state[15]))
        if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it", self.step_counter, "in TakeoffAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of TakeoffAviary._computeDone()".format(state[13], state[14], state[15]))

