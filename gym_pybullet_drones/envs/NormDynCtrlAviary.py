import numpy as np
from scipy.optimize import nnls
from gym import spaces

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.DynCtrlAviary import DynCtrlAviary


######################################################################################################################################################
#### Multi-drone environment class for control applications with thrust and torques inputs ###########################################################
######################################################################################################################################################
class NormDynCtrlAviary(DynCtrlAviary):

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
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1,
                    neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                    gui=False, record=False, obstacles=False, user_debug_gui=True):
        super().__init__(drone_model=drone_model, num_drones=num_drones, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui)

    ####################################################################################################
    #### Return the action space of the environment, a Dict of Box(4,) with NUM_DRONES entries #########
    ####################################################################################################
    def _actionSpace(self):
        #act_lower_bound = np.array([0.,              -self.MAX_XY_TORQUE, -self.MAX_XY_TORQUE, -self.MAX_Z_TORQUE])
        #act_upper_bound = np.array([self.MAX_THRUST, self.MAX_XY_TORQUE,  self.MAX_XY_TORQUE,  self.MAX_Z_TORQUE])
        #### Action vector ######## Thrust        X Torque      Y Torque      Z Torque
        act_lower_bound = np.array([-1,           -1,           -1,           -1])
        act_upper_bound = np.array([1,            1,            1,            1])
        return spaces.Dict({ str(i): spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32) for i in range(self.NUM_DRONES) })

    ####################################################################################################
    #### Return the observation space of the environment, a Dict with NUM_DRONES entries of Dict of ####
    #### { Box(4,), MultiBinary(NUM_DRONES) } ##########################################################
    ####################################################################################################
    def _observationSpace(self):
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
        #obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
        #obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        #return spaces.Dict({ str(i): spaces.Dict ({"state": spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32),
        #                                            "neighbors": spaces.MultiBinary(self.NUM_DRONES) }) for i in range(self.NUM_DRONES) })
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
        obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
        obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])
        return spaces.Dict({ str(i): spaces.Dict ({"state": spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32),
                                                    "neighbors": spaces.MultiBinary(self.NUM_DRONES) }) for i in range(self.NUM_DRONES) })

    ####################################################################################################
    #### Return the current observation of the environment #############################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (dict)                       {"0":{"state": np.arr(20,),"neighbors": np.arr(NUM_DRONES)},
    ####                                    .. "NUM_DRONES-1": {..} } ##################################
    ####                                    for the "state"'s content see _observationSpace() ##########
    ####                                    "neighbors" is the drone's row of the adjacency matrix #####
    ####################################################################################################
    def _computeObs(self):
        #adjacency_mat = self._getAdjacencyMatrix()
        #return {str(i): {"state": self._getDroneStateVector(i), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }
        adjacency_mat = self._getAdjacencyMatrix()
        return {str(i): {"state": self._clipAndNormalizeState(self._getDroneStateVector(i)), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }


    ####################################################################################################
    #### Preprocess the action passed to step() ########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (dict of (4,1) array)     commanded thrust, x, y, and z torques for each drone #######
    ####################################################################################################
    #### Returns #######################################################################################
    #### - clip_action ((N_DRONES,4,1) arr) clipped RPMs commanded to the 4 motors of each drone #######
    ####################################################################################################
    def _preprocessAction(self, action):
        clipped_action = np.zeros((self.NUM_DRONES,4))
        for k, v in action.items():

            # clipped_action[int(k),:] = np.clip(np.array(self._normalizedActionToRPM(v)), 0, self.MAX_RPM)
            # _normalizedActionToDyn ?

            clipped_action[int(k),:] = self._nnlsRPM(thrust=v[0], x_torque=v[1], y_torque=v[2], z_torque=v[3])
        return clipped_action

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
        return -1

    ####################################################################################################
    #### Compute the current done value(s) #############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (..)                        the done value(s) associated to the current obs/state ######
    ####################################################################################################
    def _computeDone(self, obs):
        return False

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
        clipped_pos = np.clip(state[0:3], -1, 1)
        clipped_rp = np.clip(state[7:9], -np.pi/3, np.pi/3)
        clipped_vel = np.clip(state[10:13], -1, 1)
        clipped_ang_vel_rp = np.clip(state[13:15], -10*np.pi, 10*np.pi)
        clipped_ang_vel_y = np.clip(state[15], -20*np.pi, 20*np.pi)
        if self.GUI: self._clipAndNormalizeStateWarning(state, clipped_pos, clipped_rp, clipped_vel, clipped_ang_vel_rp, clipped_ang_vel_y)
        normalized_pos = clipped_pos
        normalized_rp = clipped_rp/(np.pi/3)
        normalized_y = state[9]/np.pi
        normalized_vel = clipped_vel
        normalized_ang_vel_rp = clipped_ang_vel_rp/(10*np.pi)
        normalized_ang_vel_y = clipped_ang_vel_y/(20*np.pi)
        return np.hstack([normalized_pos, state[3:7], normalized_rp, normalized_y, normalized_vel, normalized_ang_vel_rp, normalized_ang_vel_y, state[16:20] ]).reshape(20,)

    ####################################################################################################
    #### Print a warning if any of the 20 values in a state vector is out of the normalization range ###
    ####################################################################################################
    def _clipAndNormalizeStateWarning(self, state, clipped_pos, clipped_rp, clipped_vel, clipped_ang_vel_rp, clipped_ang_vel_y):
        if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it", self.step_counter, "in NormDynCtrlAviary._clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[0], state[1], state[2]))
        if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it", self.step_counter, "in NormDynCtrlAviary._clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[7], state[8]))
        if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it", self.step_counter, "in NormDynCtrlAviary._clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[10], state[11], state[12]))
        if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it", self.step_counter, "in NormDynCtrlAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[13], state[14], state[15]))
        if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it", self.step_counter, "in NormDynCtrlAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[13], state[14], state[15]))


