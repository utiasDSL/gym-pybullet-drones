import numpy as np
import pybullet as p
from gym.spaces import Box, Dict
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary, ActionType, ObservationType
from gym_pybullet_drones.envs.multi_agent_rl import BaseMultiagentAviary

class PredatorPreyAviary(BaseMultiagentAviary):
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

        self.fov = fov
        self.predators = list(range(num_predators))
        self.preys = list(range(num_predators, num_predators+num_preys))

        super().__init__(drone_model=drone_model,
                         num_drones=num_predators+num_preys,
                         physics=Physics.PYB,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         obs=obs,
                         act=ActionType.PID,
                         episode_len_sec=episode_len_sec)

    def reset(self, init_xyzs=None, init_rpys=None):
        if init_xyzs is not None: self.INIT_XYZS = init_xyzs
        if init_rpys is not None: self.INIT_RPYS = init_rpys
        obs = super().reset()
        return obs

    def step(self, actions):
        assert len(actions) == self.NUM_DRONES
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
        self.step_counter = self.step_counter + self.AGGR_PHY_STEPS
        return obs, reward, done, info

    def _computeObs(self):
        obs = super()._computeObs()
        obs = {i: np.concatenate([obs[i], obs[self.NUM_DRONES-1]]) for i in self.predators}
        return obs

    def _computeReward(self):
        rayFromPositions = self.pos[:len(self.predators)]
        rayToPositions = np.tile(self.pos[-1], (len(self.predators), 1))
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
        in_sight = id & in_fov
        reward = {}
        reward.update({i: float(in_sight[i]) for i in self.predators})
        reward.update({i: -float(in_sight.sum()) for i in self.preys})
        return reward
    
    def _actionSpace(self):
        low = np.array([-1, -1, -1, -1, -1, -1])
        high = np.array([1, 1, 1, 1, 1, 1])
        return Dict({i: Box(low, high, dtype=np.float32) for i in self.predators})
    
    def _observationSpace(self):
        low = -np.ones(24)
        high = np.ones(24)
        return Dict({i: Box(low, high, dtype=float) for i in self.predators})

class PredatorAviary(PredatorPreyAviary):
    def __init__(self, 
            num_predators: int = 3, 
            fov: float = np.pi / 3, 
            *, 
            drone_model: DroneModel = DroneModel.CF2X, 
            freq: int = 240, 
            aggregate_phy_steps: int = 1, 
            gui=False, 
            obs: ObservationType = ObservationType.KIN, 
            episode_len_sec=5):
        self.prey = num_predators
        super().__init__(num_predators, 1, fov, 
            drone_model=drone_model, freq=freq, 
            aggregate_phy_steps=aggregate_phy_steps, 
            gui=gui, obs=obs, episode_len_sec=episode_len_sec)

    def reset(self, init_xyzs=None, init_rpys=None):
        init_xyzs = self.INIT_XYZS
        init_xyzs[-1] = np.random.random(3)
        r = np.linalg.norm(init_xyzs[-1, :2])
        angles = np.linspace(0, np.pi*2, 7) + np.arctan2(init_xyzs[-1, 0], init_xyzs[-1, 1])
        self.waypoints = np.stack([
            r * np.cos(angles),
            r * np.sin(angles),
            init_xyzs[-1, 2] * np.ones_like(angles)
        ]).T
        self.waypoint_cnt = 0
        obs = super().reset(init_xyzs, init_rpys)
        print(self.waypoint_cnt)
        return {i: obs[i] for i in self.predators}

    def step(self, actions):
        target_pos = self.waypoints[self.waypoint_cnt]
        distance = np.linalg.norm(self.pos[-1]-target_pos) 
        if distance < 0.05:
            self.waypoint_cnt = (self.waypoint_cnt + 1) % len(self.waypoints)
            print(self.waypoint_cnt)

        target_rpy = self.rpy[-1]
        actions[self.prey] = np.concatenate([target_pos, target_rpy])
        return super().step(actions)

    def _computeReward(self):
        reward = super()._computeReward()
        return {i: reward[i] for i in self.predators}

if __name__ == "__main__":
    env = PredatorAviary()
    obs = env.reset()
    assert env.observation_space.contains(obs), [_.shape for _ in obs.values()]
    action = env.action_space.sample()
    assert env.action_space.contains(action)
    obs, _, _, _ = env.step(action)
    assert env.observation_space.contains(obs), obs.shape