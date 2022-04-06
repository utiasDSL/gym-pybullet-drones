import numpy as np
import pybullet as p
from gym.spaces import Box, Dict
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.envs.multi_agent_rl import BaseMultiagentAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType
from numpy.random import uniform

MAX_XYZ = np.array([15, 15, 5])
MIN_XYZ = np.array([-15, -15, 0])

class NavigationAviary(BaseMultiagentAviary):
    def __init__(self,
        drone_model: DroneModel=DroneModel.CF2X,
        num_drones: int=2,
        neighbourhood_radius: float=np.inf,
        initial_xyzs=None,
        initial_rpys=None,
        physics: Physics=Physics.PYB,
        freq: int=240,
        aggregate_phy_steps: int=1,
        gui=False,
        record=False, 
        record_path=None,
        obs: ObservationType=ObservationType.KIN,
        act: ActionType=ActionType.RPM,
        episode_len_sec=5,
        ):

        self.debug = record
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         record=record, record_path=record_path,
                         obs=obs,
                         act=act,
                         episode_len_sec=episode_len_sec)

    def _observationSpace(self):
        if self.OBS_TYPE == ObservationType.RGBD:
            img_shape = (self.IMG_RES[1], self.IMG_RES[0], 4)
            return Dict({i: Box(low=0, high=255, shape=img_shape, dtype=np.uint8) for i in range(self.NUM_DRONES)})
        else: raise ValueError     

    def reset(self):
        obs = super().reset()
        return obs
    
    def step(self, actions):
        results = super().step(actions)
        return results

    def _computeReward(self):
        return {i: 0 for i in range(self.NUM_DRONES)}
    
    def _computeInfo(self):
        return {i: {"goal_reached":self.goal_reached} for i in range(self.NUM_DRONES)}
        
    def _clipAndNormalizeState(self, state):
        MAX_LIN_VEL_XY = 3 
        MAX_LIN_VEL_Z = 1

        MAX_PITCH_ROLL = np.pi # Full range

        clipped_pos_xyz = np.clip(state[:3], MIN_XYZ, MAX_XYZ)
        clipped_rp = np.clip(state[7:9], -MAX_PITCH_ROLL, MAX_PITCH_ROLL)
        clipped_vel_xy = np.clip(state[10:12], -MAX_LIN_VEL_XY, MAX_LIN_VEL_XY)
        clipped_vel_z = np.clip(state[12], -MAX_LIN_VEL_Z, MAX_LIN_VEL_Z)

        normalized_pos_xyz = clipped_pos_xyz / MAX_XYZ
        normalized_rp = clipped_rp / MAX_PITCH_ROLL
        normalized_y = state[9] / (2*np.pi) # TODO: why?
        normalized_vel_xy = clipped_vel_xy / MAX_LIN_VEL_XY
        normalized_vel_z = clipped_vel_z / MAX_LIN_VEL_XY
        normalized_ang_vel = state[13:16]/np.linalg.norm(state[13:16]) if np.linalg.norm(state[13:16]) != 0 else state[13:16]
        normalized_goal_xyz = state[-3:] / MAX_XYZ

        norm_and_clipped = np.hstack([normalized_pos_xyz,
                                      state[3:7],
                                      normalized_rp,
                                      normalized_y,
                                      normalized_vel_xy,
                                      normalized_vel_z,
                                      normalized_ang_vel,
                                      ((state[16:20] / self.HOVER_RPM) - 1), # rpms
                                      normalized_goal_xyz,
                                      ]).reshape(23,)

        return norm_and_clipped
