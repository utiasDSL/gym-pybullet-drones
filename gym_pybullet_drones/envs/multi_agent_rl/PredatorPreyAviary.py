import numpy as np
import pybullet as p
from gym.spaces import Box, Dict
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary, ActionType, ObservationType
from gym_pybullet_drones.envs.multi_agent_rl import BaseMultiagentAviary

MAX_XYZ = np.array([15, 15, 5])
MIN_XYZ = np.array([-15, -15, 0])

class PredatorPrey(BaseMultiagentAviary):
    def __init__(self,
        num_predators: int=3,
        num_preys: int=1,
        fov: float=np.pi/3,
        *,
        drone_model: DroneModel=DroneModel.CF2X,
        freq: int=240,
        aggregate_phy_steps: int=1,
        gui=False,
        obs: ObservationType=ObservationType.KIN,
        episode_len_sec=5,
        ):

        super().__init__(drone_model=drone_model,
                         num_drones=num_predators+num_preys,
                         physics=Physics.PYB,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         obs=obs,
                         act=ActionType.PID,
                         episode_len_sec=episode_len_sec)
        self.fov = fov

    def reset(self, init_xyzs=None, init_rpys=None):
        if init_xyzs is not None: self.INIT_XYZS = init_xyzs
        if init_rpys is not None: self.INIT_RPYS = init_rpys
        obs = super().reset()
        return obs
    
    def step(self, actions):
        clipped_action = []
        for i in range(self.NUM_DRONES):
            target_pos, target_rpy = actions[i][:3], actions[i][3:]
            rpm, _, _ = self.ctrl[i].computeControlFromState(
                control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP,
                state=self._getDroneStateVector(i),
                target_pos=target_pos,
                target_rpy=target_rpy
            )
            clipped_action.append(rpm)
        clipped_action = np.stack(clipped_action)

        for _ in range(self.AGGR_PHY_STEPS):
            #### Update and store the drones kinematic info for certain
            #### Between aggregate steps for certain types of update ###
            if self.AGGR_PHY_STEPS > 1 and self.PHYSICS in [Physics.DYN, Physics.PYB_GND, Physics.PYB_DRAG, Physics.PYB_DW, Physics.PYB_GND_DRAG_DW]:
                self._updateAndStoreKinematicInformation()
            #### Step the simulation using the desired physics update ##
            for i in range (self.NUM_DRONES):
                if self.PHYSICS == Physics.PYB:
                    self._physics(clipped_action[i, :], i)
                elif self.PHYSICS == Physics.DYN:
                    self._dynamics(clipped_action[i, :], i)
                elif self.PHYSICS == Physics.PYB_GND:
                    self._physics(clipped_action[i, :], i)
                    self._groundEffect(clipped_action[i, :], i)
                elif self.PHYSICS == Physics.PYB_DRAG:
                    self._physics(clipped_action[i, :], i)
                    self._drag(self.last_clipped_action[i, :], i)
                elif self.PHYSICS == Physics.PYB_DW:
                    self._physics(clipped_action[i, :], i)
                    self._downwash(i)
                elif self.PHYSICS == Physics.PYB_GND_DRAG_DW:
                    self._physics(clipped_action[i, :], i)
                    self._groundEffect(clipped_action[i, :], i)
                    self._drag(self.last_clipped_action[i, :], i)
                    self._downwash(i)
            #### PyBullet computes the new state, unless Physics.DYN ###
            if self.PHYSICS != Physics.DYN:
                p.stepSimulation(physicsClientId=self.CLIENT)
            #### Save the last applied action (e.g. to compute drag) ###
            self.last_clipped_action = clipped_action
        self._updateAndStoreKinematicInformation()
        obs = self._computeObs()
        reward = self._computeReward()
        done = self._computeDone()
        info = self._computeInfo()
        self.step_counter = self.step_counter + (1 * self.AGGR_PHY_STEPS)
        return obs, reward, done, info

    def _computeReward(self):
        rayFromPositions = self.pos[:-1]
        rayToPositions = np.tile(self.pos[-1], (self.NUM_DRONES-1, 1))
        assert rayFromPositions.shape==rayToPositions.shape
        d = rayToPositions-rayFromPositions
        roll, pitch, yaw = self.rpy[:-1].T
        ori = np.stack([
            np.cos(yaw)*np.cos(pitch),
            np.sin(yaw)*np.cos(pitch),
            np.sin(pitch)
        ]).T
        in_fov = (d * ori).sum(1) > np.cos(self.fov/2)
        id = np.array([hit[0] for hit in p.rayTestBatch(
            rayFromPositions=rayFromPositions,
            rayToPositions=rayToPositions
        )])
        reward = id & in_fov
        reward = {i: float(reward[i]) for i in range(self.NUM_DRONES-1)}
        return reward
