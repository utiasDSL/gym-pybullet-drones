import math
import numpy as np
from gym import spaces
from ray.rllib.env.multi_agent_env import MultiAgentEnv, ENV_STATE
import pybullet as pb
import pybullet_data

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType
from gym_pybullet_drones.envs.multi_agent_rl.BaseMultiagentAviary import BaseMultiagentAviary

class ShootAndDefend(BaseMultiagentAviary):
    """
    Multi-agent RL problem: one agent shoots a ball, the other tries to block it.
    """

    ################################################################################

    def __init__(
        self,
        drone_model: DroneModel=DroneModel.CF2X,
        neighbourhood_radius: float=np.inf,
        initial_xyzs=None,
        initial_rpys=None,
        physics: Physics=Physics.PYB,
        freq: int=240,
        aggregate_phy_steps: int=1,
        gui=False,
        record=False, 
        obs: ObservationType=ObservationType.KIN,
        competition_dims: np.array=np.array([4, 4, 4]),
        shooter_dims: np.array=np.array([1, 4, 4]),
        defender_dims: np.array=np.array([1, 4, 4])
    ):
        """
        Initialization of a multi-agent RL environment.

        Using the generic multi-agent RL superclass.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        freq : int, optional
            The frequency (Hz) at which the physics engine steps.
        aggregate_phy_steps : int, optional
            The number of physics steps within one call to `BaseAviary.step()`.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation in folder `files/videos/`.
        obs : ObservationType, optional
            The type of observation space (kinematic information or vision)
        competition_dims: np.array, optional
            The length, width, and height of the competition space. One of the
            walls of the competition space is used as the goal plane.
            x --> length
            y --> width
            z --> height
        shooter_dims: np.array, optional
            The length, width, and height of the shooter's bounding volume.
            The shooter's bounding volume is attached to the plane opposite to
            the goal plane.
        defender_dims: np.array, optional
            The length, width, and height of the defender's bounding volume.
            The defender's bounding volume is attached to the plane adjacent to
            the goal plane.
        """

        # Competition space
        # X bounds: [-length/2, length/2]
        # Y bounds: [-width/2, width/2]
        # Z bounds: [0, height]
        num_drones = 2
        self.defender_id = 0
        self.shooter_id = 1
        self.ball_id = -1
        self.comp_length, self.comp_width, self.comp_height = competition_dims

        self.goal_corners = np.array(
            [
                [-self.comp_length/2, -self.comp_width/2, 0],
                [-self.comp_length/2, self.comp_width/2, 0],
                [-self.comp_length/2, self.comp_width/2, self.comp_height],
                [-self.comp_length/2, -self.comp_width/2, self.comp_height],
            ]
        )

        self.ball_launched = False

        super().__init__(
            drone_model=drone_model,
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
            act=ActionType.RPM
        )

    ################################################################################

    def _actionSpace(self):
        """
        Overloaded from the BaseMultiagentAviary class. The defender has the
        standard four actions (one for each motor), but the shooter has four
        motor RPM commands and a single shoot command. The shoot command
        releases a ball from the shooter's grasp with the shooter's velocity at
        release time as the ball's initial velocity.

        Returns
        -------
        dict[int, ndarray]
            A Dict() of Box() of size 1, 3, or 3, depending on the action type,
            indexed by drone Id in integer format.

        """
        defender_action_size = 4
        shooter_action_size = 5
        shooter_action_space = spaces.Box(
            low=[-1, -1, -1, -1, 0],
            high=np.ones(shooter_action_size),
            dtype=np.float32
        )
        defender_action_space = spaces.Box(
            low=-1*np.ones(defender_action_size),
            high=np.ones(defender_action_size),
            dtype=np.float32
        )
        action_space = spaces.Dict(
            {
                self.shooter_id: shooter_action_space,
                self.defender_id: defender_action_space
            }
        )
        return action_space

    ################################################################################

    def _computeReward(self):
        """Computes the current reward value(s).

        Returns
        -------
        dict[int, float]
            The reward value for each drone.

        """
        rewards = {}
        states = np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])
        rewards[0] = -1 * np.linalg.norm(np.array([0, 0, 1]) - states[0, 0:3])**2
        for i in range(1, self.NUM_DRONES):
            rewards[i] = -1 * np.linalg.norm(states[i-1, 2] - states[i, 2])**2
        return rewards

    ################################################################################
    
    def _computeDone(self):
        """
        Computes the current done value(s).

        Returns
        -------
        dict[int | "__all__", bool]
            Dictionary with the done value of each drone and 
            one additional boolean value for key "__all__".

        """
        bool_val = True if self.step_counter/self.SIM_FREQ > self.EPISODE_LEN_SEC else False
        done = {i: bool_val for i in range(self.NUM_DRONES)}
        done["__all__"] = True if True in done.values() else False
        return done

    ################################################################################
    
    def _computeInfo(self):
        """
        Computes the current info dict(s).

        Unused.

        Returns
        -------
        dict[int, dict[]]
            Dictionary of empty dictionaries.

        """
        return {i: {} for i in range(self.NUM_DRONES)}

    ################################################################################

    def _clipAndNormalizeState(self, state):
        """
        Normalizes a drone's state to the [-1,1] range.

        Parameters
        ----------
        state : ndarray
            (20,)-shaped array of floats containing the non-normalized state of a single drone.

        Returns
        -------
        ndarray
            (20,)-shaped array of floats containing the normalized state of a single drone.

        """
        MAX_LIN_VEL_XY = 3 
        MAX_LIN_VEL_Z = 1

        MAX_XY = MAX_LIN_VEL_XY*self.EPISODE_LEN_SEC
        MAX_Z = MAX_LIN_VEL_Z*self.EPISODE_LEN_SEC

        MAX_PITCH_ROLL = np.pi # Full range

        clipped_pos_xy = np.clip(state[0:2], -MAX_XY, MAX_XY)
        clipped_pos_z = np.clip(state[2], 0, MAX_Z)
        clipped_rp = np.clip(state[7:9], -MAX_PITCH_ROLL, MAX_PITCH_ROLL)
        clipped_vel_xy = np.clip(state[10:12], -MAX_LIN_VEL_XY, MAX_LIN_VEL_XY)
        clipped_vel_z = np.clip(state[12], -MAX_LIN_VEL_Z, MAX_LIN_VEL_Z)

        normalized_pos_xy = clipped_pos_xy / MAX_XY
        normalized_pos_z = clipped_pos_z / MAX_Z
        normalized_rp = clipped_rp / MAX_PITCH_ROLL
        normalized_y = state[9] / np.pi # No reason to clip
        normalized_vel_xy = clipped_vel_xy / MAX_LIN_VEL_XY
        normalized_vel_z = clipped_vel_z / MAX_LIN_VEL_XY
        normalized_ang_vel = state[13:16]/(np.linalg.norm(state[13:16]) + 1e-6)

        norm_and_clipped = np.hstack(
            [
                normalized_pos_xy,
                normalized_pos_z,
                state[3:7],
                normalized_rp,
                normalized_y,
                normalized_vel_xy,
                normalized_vel_z,
                normalized_ang_vel,
                state[16:20]
            ]
        ).reshape(20,)

        return norm_and_clipped
    
    ################################################################################
    
    def _clipAndNormalizeStateWarning(
        self,
        state,
        clipped_pos_xy,
        clipped_pos_z,
        clipped_rp,
        clipped_vel_xy,
        clipped_vel_z,
    ):
        """
        Debugging printouts associated to `_clipAndNormalizeState`.

        Print a warning if values in a state vector is out of the clipping range.
        
        """
        if not(clipped_pos_xy == np.array(state[0:2])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped xy position [{:.2f} {:.2f}]".format(state[0], state[1]))
        if not(clipped_pos_z == np.array(state[2])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped z position [{:.2f}]".format(state[2]))
        if not(clipped_rp == np.array(state[7:9])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped roll/pitch [{:.2f} {:.2f}]".format(state[7], state[8]))
        if not(clipped_vel_xy == np.array(state[10:12])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped xy velocity [{:.2f} {:.2f}]".format(state[10], state[11]))
        if not(clipped_vel_z == np.array(state[12])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped z velocity [{:.2f}]".format(state[12]))

    def _computeObs(self):
        """
        Kinematic states for each drone and the ball.
        """
        obs_12 = np.zeros((self.NUM_DRONES + 1,12))
        for i in range(self.NUM_DRONES):
            obs = self._clipAndNormalizeState(self._getDroneStateVector(i))
            obs_12[i, :] = np.hstack([obs[0:3], obs[7:10], obs[10:13], obs[13:16]]).reshape(12,)
        obs[-1, :] = self._getBallObs()
        obs_dict = {drone_id: obs for drone_id in self.DRONE_IDS}
        return 

    def _getBallObs(self):
        ball_obs = np.zeros(12)
        if not self.ball_launched:
            shooter_state = self._getDroneStateVector(self.shooter_)
            R_gs2b = pb.getMatrixFromQuaternion(shooter_state[3:7])
            ball_pos = 1.5*R_gs2b[:, 0]*self.L + shooter_state[0:3]
            shooter_obs = self._clipAndNormalizeState(shooter_state)
            ball_obs[0:3] = np.hstack(
                [shooter_obs[0:3], shooter_obs[7:10], np.zeros(3), np.zeros(3)]
            ).reshape(12,)
        else:
            ball_pos, ball_quat = pb.getBasePositionAndOrientation(self.ball_id, physicsClientId=self.CLIENT)
            ball_rpy = pb.getEulerFromQuaternion(ball_quat)
            ball_vel, ball_ang_v = pb.getBaseVelocity(self.ball_id, physicsClientId=self.CLIENT)
            ball_obs = np.hstack(
                ball_pos, ball_rpy, ball_vel, ball_ang_v
            ).reshape(12,)
        return ball_obs

    def _launchBall(self):
        R_gs2b = pb.getMatrixFromQuaternion(shooter_state[3:7])
        ball_pos = 1.5*R_gs2b[:, 0]*self.L + shooter_state[0:3]
        self.ball_id = pb.loadURDF("sphere2.urdf",
            ball_pos,
            pb.getQuaternionFromEuler([0,0,0]),
            globalScaling=0.0625,
            physicsClientId=self.CLIENT
        )
        pb.changeDynamics(self.ball_id, -1, mass=0.1)
        shooter_vel, _ = pb.getBaseVelocity(self.shooter_id, physicsClient=self.CLIENT)
        ball_force_unit = np.linalg.norm(shooter_vel)
        ball_force = 50*force_unit
        pb.applyExternalForce(
            self.ball_id,
            -1,
            forceObj=ball_force,
            posObj=[0, 0, 0],
            flags=pb.LINK_FRAME,
            physicsClientId=self.CLIENT
        )
