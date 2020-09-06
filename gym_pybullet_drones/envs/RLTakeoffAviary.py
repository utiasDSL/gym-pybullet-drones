import numpy as np
from gym import error, spaces, utils

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary

class RLTakeoffAviary(BaseAviary):

    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1, \
                        visibility_radius: float=np.inf, initial_xyzs=None, initial_rpys=None, \
                        physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1, \
                        gui=False, record=False, obstacles=False):
        if num_drones!=1: print("[ERROR] in RLTakeoffAviary.__init__(), RLTakeoffAviary only accepts num_drones=1" ); exit()
        super().__init__(drone_model=drone_model, visibility_radius=visibility_radius, \
            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq, aggregate_phy_steps=aggregate_phy_steps, \
            gui=gui, record=record, obstacles=obstacles) 

    ####################################################################################################
    #### 
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([-1,           -1,           -1,           -1])
        act_upper_bound = np.array([1,            1,            1,            1])
        return spaces.Box( low=act_lower_bound, high=act_upper_bound, dtype=np.float32 )
        
    def _observationSpace(self):
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
        obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
        obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])
        return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )

    def _computeObs(self):
        return self._clipAndNormalizeState(self._getDroneState(0))

    def _preprocessAction(self, action):
        rpm = self._normActionToRPM(action)
        return np.clip(np.array(rpm), 0, self.MAX_RPM)

    def _computeReward(self, obs):
        if obs[2] > 0.8: return -1
        elif obs[2] > 0.5: return 2000
        elif obs[2] > 0.3: return 1000
        elif obs[2] > 0.2: return 500
        elif obs[2] > 0.15: return 100
        elif obs[2] > 0.1: return 10
        else: return -1

    def _computeDone(self, norm_obs):
        if np.abs(norm_obs[0])>=1 or np.abs(norm_obs[1])>=1 or norm_obs[2]>=1 \
            or np.abs(norm_obs[7])>=1 or np.abs(norm_obs[8])>=1 \
            or np.abs(norm_obs[10])>=1 or np.abs(norm_obs[11])>=1 or np.abs(norm_obs[12])>=1 \
            or np.abs(norm_obs[13])>=1 or np.abs(norm_obs[14])>=1 or np.abs(norm_obs[15])>=1 \
            or self.step_counter/self.SIM_FREQ > 3: return True
        else: return False

    def _computeInfo(self, obs):
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years

    ######

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

    def _clipAndNormalizeStateWarning(self, state, clipped_pos, clipped_rp, clipped_vel, clipped_ang_vel_rp, clipped_ang_vel_y):
        if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it", self.step_counter, "in RLTakeoffAviary._clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[0], state[1], state[2]))
        if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it", self.step_counter, "in RLTakeoffAviary._clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[7], state[8]))
        if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it", self.step_counter, "in RLTakeoffAviary._clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[10], state[11], state[12]))
        if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it", self.step_counter, "in RLTakeoffAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[13], state[14], state[15]))
        if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it", self.step_counter, "in RLTakeoffAviary._clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of RLTakeoffAviary._computeDone()".format(state[13], state[14], state[15]))

