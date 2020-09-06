import numpy as np
from gym import error, spaces, utils

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary

class CtrlAviary(BaseAviary):

    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1, \
                        visibility_radius: float=np.inf, initial_xyzs=None, initial_rpys=None, \
                        physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1, \
                        gui=False, record=False, obstacles=False):
        super().__init__(drone_model=drone_model, num_drones=num_drones, visibility_radius=visibility_radius, \
            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq, aggregate_phy_steps=aggregate_phy_steps, \
            gui=gui, record=record, obstacles=obstacles) 

    ####################################################################################################
    #### 
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([0.,           0.,           0.,           0.])
        act_upper_bound = np.array([self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({ str(i): spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32) for i in range(self.NUM_DRONES) })
        
    def _observationSpace(self):
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
        obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
        obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({ str(i): spaces.Dict ({"state": spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32), \
                                                    "neighbors": spaces.MultiBinary(self.NUM_DRONES) }) for i in range(self.NUM_DRONES) })

    def _computeObs(self):
        adjacency_mat = self._getAdjacencyMatrix()
        return {str(i): {"state": self._getDroneState(i), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }

    def _preprocessAction(self, action):
        clipped_action = np.zeros((self.NUM_DRONES,4))
        for k, v in action.items(): 
            clipped_action[int(k),:] = np.clip(np.array(v), 0, self.MAX_RPM)
        return clipped_action

    def _computeReward(self, obs):
        return -1

    def _computeDone(self, obs):
        return False

    def _computeInfo(self, obs):
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years


