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
        obs: ObservationType=ObservationType.KIN,
        act: ActionType=ActionType.RPM,
        goal_reset: int=2,):

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
                         record=record, 
                         obs=obs,
                         act=act)
        self.goal_reset = goal_reset

    def _observationSpace(self):
        joint_obs_space = super()._observationSpace()
        goal_high   = np.array([np.inf]*3)
        goal_low    = -goal_high
        def aug_obs_space(obs_space):
            assert isinstance(obs_space, Box)
            low = np.concatenate([obs_space.low, goal_low])
            high = np.concatenate([obs_space.high, goal_high])
            return Box(low, high, dtype=obs_space.dtype)
        joint_obs_space = Dict({
            i: aug_obs_space(joint_obs_space[i]) for i in range(self.NUM_DRONES)})
        return joint_obs_space

    def _setGoals(self, from_pos):
        # self.goals = from_pos + 0.5 * (np.random.random((self.NUM_DRONES, 3))-0.5) # random goal
        self.goals = from_pos + np.array([0., 0., 0.3]) + 0.3 * uniform(-1, 1, (self.NUM_DRONES, 3)) # take off
        self.goals = np.clip(self.goals, MIN_XYZ, MAX_XYZ)
        self.success = np.zeros(self.NUM_DRONES, bool)
        self.distance_max = self.distance = np.linalg.norm(from_pos - self.goals, axis=1)      

    def reset(self):
        self._setGoals(self.INIT_XYZS)
        self.goal_reached = 0
        obs = super().reset()

        if self.debug: 
            for i in range(self.NUM_DRONES):
                vshape_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1., 0., 0., 1.])
                p.createMultiBody(baseMass=0, baseVisualShapeIndex=vshape_id, basePosition=self.goals[i]) 
            p.removeAllUserDebugItems()
            self.line_ids = [p.addUserDebugLine(pos, goal, [1., 0., 0.], lineWidth=2.) for pos, goal in zip(self.pos, self.goals)]

        return obs
    
    def step(self, actions):
        results = super().step(actions)
        if self.debug:
            for pos, goal, line_id in zip(self.pos, self.goals, self.line_ids):
                p.addUserDebugLine(pos, goal, [1., 0., 0.], lineWidth=2., replaceItemUniqueId=line_id)
        if self.success.all() and self.goal_reached < self.goal_reset:
            self.goal_reached += 1
            self._setGoals(self.goals)
            for i in range(self.NUM_DRONES):
                vshape_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1., 0., 0., 1.])
                p.createMultiBody(baseMass=0, baseVisualShapeIndex=vshape_id, basePosition=self.goals[i]) 
        return results
        
    def _computeObs(self):
        if self.OBS_TYPE == ObservationType.KIN:
            obs = {
                i: self._clipAndNormalizeState(np.concatenate([self._getDroneStateVector(i), self.goals[i]-self.pos[i]])) 
                for i in range(self.NUM_DRONES)
            }
        else: 
            raise ValueError
        return obs

    def _computeReward(self):
        distance = np.linalg.norm(self.pos - self.goals, axis=1)
        distance_reduction = self.distance - distance # normalize
        reward = {i: distance_reduction[i]
            for i in range(self.NUM_DRONES)}
        # reward = {i: -distance[i]**2 for i in range(self.NUM_DRONES)}
        self.distance = distance
        success = self.success | (distance < 0.1)
        reward += success & ~self.success
        self.success = success
        return reward - 0.01
    
    def _computeDone(self):
        bool_val = True if self.step_counter/self.SIM_FREQ >= self.EPISODE_LEN_SEC else False
        done = {i: bool_val for i in range(self.NUM_DRONES)}
        done["__all__"] = True if True in done.values() else False
        return done
    
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
