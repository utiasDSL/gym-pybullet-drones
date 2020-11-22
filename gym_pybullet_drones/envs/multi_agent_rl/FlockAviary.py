import math
import numpy as np
from gym import spaces
from ray.rllib.env.multi_agent_env import MultiAgentEnv, ENV_STATE

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType
from gym_pybullet_drones.envs.multi_agent_rl.BaseMultiagentAviary import BaseMultiagentAviary

class FlockAviary(BaseMultiagentAviary):
    """Multi-agent RL problem: flocking."""

    ################################################################################

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
                 act: ActionType=ActionType.RPM):
        """Initialization of a multi-agent RL environment.

        Using the generic multi-agent RL superclass.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
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
        act : ActionType, optional
            The type of action space (1 or 3D; RPMS, thurst and torques, or waypoint with PID control)

        """
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
                         act=act
                         )

    ################################################################################
    
    def _computeReward(self):
        """Computes the current reward value(s).

        Returns
        -------
        dict[int, float]
            The reward value for each drone.

        """
        rewards = {}
        states = np.array([self._getDroneStateVector(0) for i in range(self.NUM_DRONES)])
        rewards[0] = -1 * np.linalg.norm(np.array([0, 0, 1]) - states[0, 0:3])**2
        for i in range(1, self.NUM_DRONES):
            rewards[i] = -1 * np.linalg.norm(states[i-1, 2] - states[i, 2])**2
        return rewards
        """
        # obs here is dictionary of the form {"i":{"state": Box(20,), "neighbors": MultiBinary(NUM_DRONES)}}
        # parse velocity and position
        vel = np.zeros((1, self.NUM_DRONES, 3)); pos = np.zeros((1, self.NUM_DRONES, 3))
        for i in range(self.NUM_DRONES):
            pos[0,i,:] = obs[   i   ]["state"][0:3]
            vel[0,i,:] = obs[   i   ]["state"][10:13]
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
        return {   i   : reward for i in range(self.NUM_DRONES) }
        """

    ################################################################################
    
    def _computeDone(self):
        """Computes the current done value(s).

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
        """Computes the current info dict(s).

        Unused.

        Returns
        -------
        dict[int, dict[]]
            Dictionary of empty dictionaries.

        """
        return {i: {} for i in range(self.NUM_DRONES)}

    ################################################################################

    def _clipAndNormalizeState(self,
                               state
                               ):
        """Normalizes a drone's state to the [-1,1] range.

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
        MAX_PITCH_ROLL_VEL = 6*np.pi
        MAX_YAW_VEL = 3*np.pi

        clipped_pos_xy = np.clip(state[0:2], -MAX_XY, MAX_XY)
        clipped_pos_z = np.clip(state[2], 0, MAX_Z)
        clipped_rp = np.clip(state[7:9], -MAX_PITCH_ROLL, MAX_PITCH_ROLL)
        clipped_vel_xy = np.clip(state[10:12], -MAX_LIN_VEL_XY, MAX_LIN_VEL_XY)
        clipped_vel_z = np.clip(state[12], -MAX_LIN_VEL_Z, MAX_LIN_VEL_Z)

        clipped_ang_vel_rp = np.clip(state[13:15], -MAX_PITCH_ROLL_VEL, MAX_PITCH_ROLL_VEL)
        clipped_ang_vel_y = np.clip(state[15], -MAX_YAW_VEL, MAX_YAW_VEL)

        if self.GUI:
            self._clipAndNormalizeStateWarning(state,
                                               clipped_pos_xy,
                                               clipped_pos_z,
                                               clipped_rp,
                                               clipped_vel_xy,
                                               clipped_vel_z,
                                               clipped_ang_vel_rp,
                                               clipped_ang_vel_y
                                               )

        normalized_pos_xy = clipped_pos_xy / MAX_XY
        normalized_pos_z = clipped_pos_z / MAX_Z
        normalized_rp = clipped_rp / MAX_PITCH_ROLL
        normalized_y = state[9] / np.pi # No reason to clip
        normalized_vel_xy = clipped_vel_xy / MAX_LIN_VEL_XY
        normalized_vel_z = clipped_vel_z / MAX_LIN_VEL_XY
        normalized_ang_vel_rp = clipped_ang_vel_rp / MAX_PITCH_ROLL_VEL
        normalized_ang_vel_y = clipped_ang_vel_y / MAX_YAW_VEL

        norm_and_clipped = np.hstack([normalized_pos_xy,
                                      normalized_pos_z,
                                      state[3:7],
                                      normalized_rp,
                                      normalized_y,
                                      normalized_vel_xy,
                                      normalized_vel_z,
                                      normalized_ang_vel_rp,
                                      normalized_ang_vel_y,
                                      state[16:20]
                                      ]).reshape(20,)

        return norm_and_clipped

    ################################################################################

    def _clipAndNormalizeStateWarning(self,
                                      state,
                                      clipped_pos_xy,
                                      clipped_pos_z,
                                      clipped_rp,
                                      clipped_vel_xy,
                                      clipped_vel_z,
                                      clipped_ang_vel_rp,
                                      clipped_ang_vel_y
                                      ):
        """Debugging printouts associated to `_clipAndNormalizeState`.

        Print a warning if any of the 20 values in a state vector is out of the normalization range.
        
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
        if not(clipped_ang_vel_rp == np.array(state[13:15])).all():
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped angular velocity [{:.2f} {:.2f}]".format(state[13], state[14]))
        if not(clipped_ang_vel_y == np.array(state[15])):
            print("[WARNING] it", self.step_counter, "in FlockAviary._clipAndNormalizeState(), clipped angular velocity [{:.2f}]".format(state[15]))
