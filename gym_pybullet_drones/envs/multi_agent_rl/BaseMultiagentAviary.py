import os
from datetime import datetime
import numpy as np
import pybullet as p
from gym import spaces
from ray.rllib.env.multi_agent_env import MultiAgentEnv, ENV_STATE

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType
from gym_pybullet_drones.utils.utils import nnlsRPM
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl

class BaseMultiagentAviary(BaseAviary, MultiAgentEnv):
    """Base multi-agent environment class for reinforcement learning."""
    
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
                 obs: ObservationType=ObservationType.KIN,
                 act: ActionType=ActionType.RPM,
                 episode_len_sec=5,
                 ):
        """Initialization of a generic multi-agent RL environment.

        Attributes `vision_attributes` and `dynamics_attributes` are selected
        based on the choice of `obs` and `act`; `obstacles` is set to True 
        and overridden with landmarks for vision applications; 
        `user_debug_gui` is set to False for performance.

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
            The type of action space (1 or 3D; RPMS, thurst and torques, waypoint or velocity with PID control; etc.)

        """
        if num_drones < 2:
            print("[ERROR] in BaseMultiagentAviary.__init__(), num_drones should be >= 2")
            exit()
        if act == ActionType.TUN:
            print("[ERROR] in BaseMultiagentAviary.__init__(), ActionType.TUN can only used with BaseSingleAgentAviary")
            exit()
        self._agent_ids = set(range(num_drones))
        vision_attributes = True if obs == ObservationType.RGB else False
        dynamics_attributes = True if act in [ActionType.DYN, ActionType.ONE_D_DYN] else False
        self.OBS_TYPE = obs
        self.ACT_TYPE = ActionType(act)
        self.EPISODE_LEN_SEC = episode_len_sec
        #### Create integrated controllers #########################
        if act in [ActionType.PID, ActionType.VEL, ActionType.ONE_D_PID]:
            os.environ['KMP_DUPLICATE_LIB_OK']='True'
            if drone_model in [DroneModel.CF2X, DroneModel.CF2P]:
                self.ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for i in range(num_drones)]
            elif drone_model == DroneModel.HB:
                self.ctrl = [SimplePIDControl(drone_model=DroneModel.HB) for i in range(num_drones)]
            else:
                print("[ERROR] in BaseMultiagentAviary.__init()__, no controller is available for the specified drone_model")
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         record=False,
                         obstacles=True, # Add obstacles for RGB observations and/or FlyThruGate
                         user_debug_gui=False, # Remove of RPM sliders from all single agent learning aviaries
                         vision_attributes=vision_attributes,
                         dynamics_attributes=dynamics_attributes
                         )
        #### Set a limit on the maximum target speed ###############
        if act == ActionType.VEL:
            self.SPEED_LIMIT = 0.03 * self.MAX_SPEED_KMH * (1000/3600)
        self.MAX_STEPS = self.EPISODE_LEN_SEC * self.SIM_FREQ
        self.cameras = [Camera()]

    def _addObstacles(self):
        """Add obstacles to the environment.

        Only if the observation is of type RGB, 4 landmarks are added.
        Overrides BaseAviary's method.

        """
        p.loadURDF("block.urdf",
                    [1, 0, .1],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    physicsClientId=self.CLIENT
                    )
        p.loadURDF("cube_small.urdf",
                    [0, 1, .1],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    physicsClientId=self.CLIENT
                    )
        p.loadURDF("duck_vhacd.urdf",
                    [-1, 0, .1],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    physicsClientId=self.CLIENT
                    )
        p.loadURDF("teddy_vhacd.urdf",
                    [0, -1, .1],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    physicsClientId=self.CLIENT
                    )
        

    ################################################################################

    def _actionSpace(self):
        """Returns the action space of the environment.

        Returns
        -------
        dict[int, ndarray]
            A Dict() of Box() of size 1, 3, or 3, depending on the action type,
            indexed by drone Id in integer format.

        """
        if self.ACT_TYPE in [ActionType.RPM, ActionType.DYN, ActionType.VEL]:
            size = 4
        elif self.ACT_TYPE==ActionType.PID:
            size = 3
        elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_DYN, ActionType.ONE_D_PID]:
            size = 1
        else:
            raise ValueError(f"Action type {self.ACT_TYPE} not supported.")
        return spaces.Dict({i: spaces.Box(low=-1*np.ones(size),
                                          high=np.ones(size),
                                          dtype=np.float32
                                          ) for i in range(self.NUM_DRONES)})

    ################################################################################

    def _preprocessAction(self, action):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Parameter `action` is processed differenly for each of the different
        action types: the input to n-th drone, `action[n]` can be of length
        1, 3, or 4, and represent RPMs, desired thrust and torques, or the next
        target position to reach using PID control.

        Parameter `action` is processed differenly for each of the different
        action types: `action` can be of length 1, 3, or 4 and represent 
        RPMs, desired thrust and torques, the next target position to reach 
        using PID control, a desired velocity vector, etc.

        Parameters
        ----------
        action : dict[int, ndarray]
            The input action for each drone, to be translated into RPMs.
        Returns
        -------
        ndarray
            (NUM_DRONES, 4)-shaped array of ints containing to clipped RPMs
            commanded to the 4 motors of each drone.

        """
        rpm = np.zeros((self.NUM_DRONES,4))
        for k, v in action.items():
            if self.ACT_TYPE == ActionType.RPM: 
                rpm[int(k),:] = self.HOVER_RPM * (1+0.05*v)
            elif self.ACT_TYPE == ActionType.DYN: 
                rpm[int(k),:] = nnlsRPM(thrust=(self.GRAVITY*(v[0]+1)),
                                        x_torque=(0.05*self.MAX_XY_TORQUE*v[1]),
                                        y_torque=(0.05*self.MAX_XY_TORQUE*v[2]),
                                        z_torque=(0.05*self.MAX_Z_TORQUE*v[3]),
                                        counter=self.step_counter,
                                        max_thrust=self.MAX_THRUST,
                                        max_xy_torque=self.MAX_XY_TORQUE,
                                        max_z_torque=self.MAX_Z_TORQUE,
                                        a=self.A,
                                        inv_a=self.INV_A,
                                        b_coeff=self.B_COEFF,
                                        gui=self.GUI
                                        )
            elif self.ACT_TYPE == ActionType.PID: 
                state = self._getDroneStateVector(int(k))
                rpm_k, _, _ = self.ctrl[int(k)].computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                        cur_pos=state[0:3],
                                                        cur_quat=state[3:7],
                                                        cur_vel=state[10:13],
                                                        cur_ang_vel=state[13:16],
                                                        target_pos=state[0:3]+0.1*v
                                                        )
                rpm[int(k),:] = rpm_k
            elif self.ACT_TYPE == ActionType.VEL:
                state = self._getDroneStateVector(int(k))
                if np.linalg.norm(v[0:3]) != 0:
                    v_unit_vector = v[0:3] / np.linalg.norm(v[0:3])
                else:
                    v_unit_vector = np.zeros(3)
                temp, _, _ = self.ctrl[int(k)].computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                        cur_pos=state[0:3],
                                                        cur_quat=state[3:7],
                                                        cur_vel=state[10:13],
                                                        cur_ang_vel=state[13:16],
                                                        target_pos=state[0:3], # same as the current position
                                                        target_rpy=np.array([0,0,state[9]]), # keep current yaw
                                                        target_vel=self.SPEED_LIMIT * np.abs(v[3]) * v_unit_vector # target the desired velocity vector
                                                        )
                rpm[int(k),:] = temp
            elif self.ACT_TYPE == ActionType.ONE_D_RPM: 
                rpm[int(k),:] = np.repeat(self.HOVER_RPM * (1+0.05*v), 4)
            elif self.ACT_TYPE == ActionType.ONE_D_DYN: 
                rpm[int(k),:] = nnlsRPM(thrust=(self.GRAVITY*(1+0.05*v[0])),
                                        x_torque=0,
                                        y_torque=0,
                                        z_torque=0,
                                        counter=self.step_counter,
                                        max_thrust=self.MAX_THRUST,
                                        max_xy_torque=self.MAX_XY_TORQUE,
                                        max_z_torque=self.MAX_Z_TORQUE,
                                        a=self.A,
                                        inv_a=self.INV_A,
                                        b_coeff=self.B_COEFF,
                                        gui=self.GUI
                                        )
            elif self.ACT_TYPE == ActionType.ONE_D_PID:
                state = self._getDroneStateVector(int(k))
                rpm, _, _ = self.ctrl[k].computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                        cur_pos=state[0:3],
                                                        cur_quat=state[3:7],
                                                        cur_vel=state[10:13],
                                                        cur_ang_vel=state[13:16],
                                                        target_pos=state[0:3]+0.1*np.array([0,0,v[0]])
                                                        )
                rpm[int(k),:] = rpm
            else:
                raise NotImplementedError(self.ACT_TYPE)
        return rpm

    ################################################################################

    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------
        dict[int, ndarray]
            A Dict with NUM_DRONES entries indexed by Id in integer format,
            each a Box() os shape (H,W,4) or (12,) depending on the observation type.

        """
        if self.OBS_TYPE == ObservationType.RGB:
            return spaces.Dict({i: spaces.Box(low=0,
                                              high=255,
                                              shape=(self.IMG_RES[1], self.IMG_RES[0], 4), dtype=np.uint8
                                              ) for i in range(self.NUM_DRONES)})
        elif self.OBS_TYPE == ObservationType.KIN:
            ############################################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
            dtype = np.float32
            obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1], dtype)
            obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1], dtype)          
            return spaces.Dict({i: spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=dtype)
                for i in range(self.NUM_DRONES)})
            ############################################################
            #### OBS SPACE OF SIZE 12
            # return spaces.Dict({i: spaces.Box(low=np.array([-1,-1,0, -1,-1,-1, -1,-1,-1, -1,-1,-1]),
            #                                   high=np.array([1,1,1, 1,1,1, 1,1,1, 1,1,1]),
            #                                   dtype=np.float32
            #                                   ) for i in range(self.NUM_DRONES)})
            ############################################################
        else:
            raise NotImplementedError(self.OBS_TYPE)
    
    ################################################################################

    def _computeObs(self):
        """Returns the current observation of the environment.

        Returns
        -------
        dict[int, ndarray]
            A Dict with NUM_DRONES entries indexed by Id in integer format,
            each a Box() os shape (H,W,4) or (12,) depending on the observation type.

        """
        if self.OBS_TYPE == ObservationType.RGB:
            if self.step_counter%self.IMG_CAPTURE_FREQ == 0: 
                for i in range(self.NUM_DRONES):
                    self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i, segmentation=False)
            return {i: self.rgb[i] for i in range(self.NUM_DRONES)}
        elif self.OBS_TYPE == ObservationType.KIN: 
            ############################################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            # return {   i   : self._clipAndNormalizeState(self._getDroneStateVector(i)) for i in range(self.NUM_DRONES) }
            ############################################################
            #### OBS SPACE OF SIZE 12
            obs_12 = np.zeros((self.NUM_DRONES,12))
            for i in range(self.NUM_DRONES):
                obs = self._clipAndNormalizeState(self._getDroneStateVector(i))
                obs_12[i, :] = np.hstack([obs[0:3], obs[7:10], obs[10:13], obs[13:16]]).reshape(12,)
            return {i: obs_12[i, :] for i in range(self.NUM_DRONES)}
            ############################################################
        else:
            raise NotImplementedError(self.OBS_TYPE)

    ################################################################################

    def _clipAndNormalizeState(self, state):
        """Normalizes a drone's state to the [-1,1] range.

        Parameters
        ----------
        state : ndarray
            Array containing the non-normalized state of a single drone.

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
        normalized_ang_vel = state[13:16]/np.linalg.norm(state[13:16]) if np.linalg.norm(state[13:16]) != 0 else state[13:16]

        norm_and_clipped = np.hstack([normalized_pos_xy,
                                      normalized_pos_z,
                                      state[3:7],
                                      normalized_rp,
                                      normalized_y,
                                      normalized_vel_xy,
                                      normalized_vel_z,
                                      normalized_ang_vel,
                                      state[16:20]
                                      ]).reshape(20,)

        return norm_and_clipped
    
    def _computeReward(self):
        return {i:0 for i in range(self.NUM_DRONES)}

    def _computeDone(self):
        bool_val = True if self.step_counter > self.MAX_STEPS else False
        done = {i: bool_val for i in range(self.NUM_DRONES)}
        done["__all__"] = True if True in done.values() else False
        return done
    
    def _computeInfo(self):
        return {i:{} for i in range(self.NUM_DRONES)}

    def add_camera(self, camera):
        self.cameras.append(camera)

    def render(self, *args, **kwargs):
        images = [cam._get_image(self.CLIENT) for cam in self.cameras]
        return images
    
class Camera:
    CAMERA_COUNT = 0
    def __init__(self, 
        width=640, height=480, fov=60,
        eye_pos=[-1.5, -1.5, 1.5], target_pos=[0, 0, 0.2], up=[0, 0, 1], name=None):
        if name is None:
            name = f"cam_{Camera.CAMERA_COUNT}"
            Camera.CAMERA_COUNT += 1
        self.name = name
        self.width = width
        self.height = height
        self.fov = fov
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=eye_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up
        )
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=fov,
            aspect=width/height,
            nearVal=0.1,
            farVal=1000.
        )

    def _get_image(self, client):
        w, h, rgb, dep, seg = p.getCameraImage(
            width=self.width, height=self.height,
            shadow=1,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            renderer=p.ER_TINY_RENDERER,
            flags=p.ER_NO_SEGMENTATION_MASK,
            physicsClientId=client
        )
        return rgb
