import math
import numpy as np
from gym import spaces
from ray.rllib.env.multi_agent_env import MultiAgentEnv, ENV_STATE

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary


######################################################################################################################################################
#### Multi-drone environment class for multi-agent reinforcement learning applications (in this implementation, flocking) ############################
######################################################################################################################################################
class FlockAviary(BaseAviary, MultiAgentEnv):

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
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=2,
                    neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                    gui=False, record=False, obstacles=False, user_debug_gui=True):
        if num_drones<2: print("[ERROR] in FlockAviary.__init__(), FlockAviary only accepts num_drones>2" ); exit()
        super().__init__(drone_model=drone_model, num_drones=num_drones, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui)

    ####################################################################################################
    #### Return the action space of the environment, a Dict of Box(4,) with NUM_DRONES entries #########
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([-1,           -1,           -1,           -1])
        act_upper_bound = np.array([1,            1,            1,            1])
        return spaces.Dict({ str(i): spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32) for i in range(self.NUM_DRONES) })

    ####################################################################################################
    #### Return the observation space of the environment, a Dict with NUM_DRONES entries of Dict of ####
    #### { Box(4,), MultiBinary(NUM_DRONES) } ##########################################################
    ####################################################################################################
    def _observationSpace(self):
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
        adjacency_mat = self._getAdjacencyMatrix()
        return {str(i): {"state": self._clipAndNormalizeState(self._getDroneStateVector(i)), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }

    ####################################################################################################
    #### Preprocess the action passed to step() ########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (dict of (4,1) array)     unclipped RPMs commanded to the 4 motors of each drone #####
    ####################################################################################################
    #### Returns #######################################################################################
    #### - clip_action ((N_DRONES,4,1) arr) clipped RPMs commanded to the 4 motors of each drone #######
    ####################################################################################################
    def _preprocessAction(self, action):
        clipped_action = np.zeros((self.NUM_DRONES,4))
        for k, v in action.items():
            clipped_action[int(k),:] = np.clip(np.array(self._normalizedActionToRPM(v)), 0, self.MAX_RPM)
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
        # obs here is dictionary of the form {"i":{"state": Box(20,), "neighbors": MultiBinary(NUM_DRONES)}}
        # parse velocity and position
        vel = np.zeros((1, self.NUM_DRONES, 3)); pos = np.zeros((1, self.NUM_DRONES, 3))
        for i in range(self.NUM_DRONES):
            pos[0,i,:] = obs[str(i)]["state"][0:3]
            vel[0,i,:] = obs[str(i)]["state"][10:13]
        # compute metrics
        # velocity alignment
        ali = 0
        EPSILON = 1e-3  # avoid divide by zero
        linear_vel_norm = np.linalg.norm(vel, axis=2)
        for i in range(self.NUM_DRONES):
            for j in range(self.NUM_DRONES):
                if j != i:
                    d = np.einsum('ij,ij->i', vel[:, i, :], vel[:, j, :])
                    ali += (d / (linear_vel_norm[:, i] + EPSILON) / (linear_vel_norm[:, j] + EPSILON))
        ali /= (self.NUM_DRONES * (self.NUM_DRONES - 1))
        # flocking speed
        cof_v = np.mean(vel, axis=1)  # center of flock speed
        avg_flock_linear_speed = np.linalg.norm(cof_v, axis=-1)
        # spacing
        whole_flock_spacing = []
        for i in range(self.NUM_DRONES):
            flck_neighbor_pos = np.delete(pos, [i], 1)
            drone_neighbor_pos_diff = flck_neighbor_pos - np.reshape(pos[:, i, :], (pos[:, i, :].shape[0], 1, -1))
            drone_neighbor_dis = np.linalg.norm(drone_neighbor_pos_diff, axis=-1)
            drone_spacing = np.amin(drone_neighbor_dis, axis=-1)
            whole_flock_spacing.append(drone_spacing)
        whole_flock_spacing = np.stack(whole_flock_spacing, axis=-1)
        avg_flock_spacing = np.mean(whole_flock_spacing, axis=-1)
        var_flock_spacing = np.var(whole_flock_spacing, axis=-1)
        # flocking metrics
        FLOCK_SPACING_MIN = 1.0; FLOCK_SPACING_MAX = 3.0
        if FLOCK_SPACING_MIN < avg_flock_spacing[0] < FLOCK_SPACING_MAX:
            avg_flock_spac_rew = 0.0
        else:
            avg_flock_spac_rew = min(math.fabs(avg_flock_spacing[0] - FLOCK_SPACING_MIN),
                                     math.fabs(avg_flock_spacing[0] - FLOCK_SPACING_MAX))
        reward = ali[0] + avg_flock_linear_speed[0] - avg_flock_spac_rew - var_flock_spacing[0]
        return {str(i): reward for i in range(self.NUM_DRONES) }

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
        done = {str(i): self._individualDone(obs[str(i)]["state"]) for i in range(self.NUM_DRONES)}
        done["__all__"] = True if True in done.values() else False
        return done

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
        return {str(i): {} for i in range(self.NUM_DRONES) }

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
        if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of FlockAviary._computeDone()".format(state[0], state[1], state[2]))
        if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of FlockAviary._computeDone()".format(state[7], state[8]))
        if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of FlockAviary._computeDone()".format(state[10], state[11], state[12]))
        if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of FlockAviary._computeDone()".format(state[13], state[14], state[15]))
        if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of FlockAviary._computeDone()".format(state[13], state[14], state[15]))

    ####################################################################################################
    ####  Compute the boolean done value of an individual drone ########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - norm_state ((20,1) array)        raw simulation stat data  ##################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - indiv. done (bool)               whether a drone's done is True based on its norm_state #####
    ####################################################################################################
    def _individualDone(self, norm_state):
        if np.abs(norm_state[0])>=1 or np.abs(norm_state[1])>=1 or norm_state[2]>=1 \
            or np.abs(norm_state[7])>=1 or np.abs(norm_state[8])>=1 \
            or np.abs(norm_state[10])>=1 or np.abs(norm_state[11])>=1 or np.abs(norm_state[12])>=1 \
            or np.abs(norm_state[13])>=1 or np.abs(norm_state[14])>=1 or np.abs(norm_state[15])>=1 \
            or self.step_counter/self.SIM_FREQ > 3: return True
        else: return False
