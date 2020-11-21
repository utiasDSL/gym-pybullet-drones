import os
from datetime import datetime
import numpy as np
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
                 record=False, 
                 obs: ObservationType=ObservationType.KIN,
                 act: ActionType=ActionType.RPM
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
            The type of action space (1 or 3D; RPMS, thurst and torques, or waypoint with PID control)

        """
        if num_drones < 2:
            print("[ERROR] in BaseMultiagentAviary.__init__(), num_drones should be >= 2" )
            exit()
        vision_attributes = True if obs == ObservationType.RGB else False
        dynamics_attributes = True if act in [ActionType.DYN, ActionType.ONE_D_DYN] else False
        self.OBS_TYPE = obs
        self.ACT_TYPE = act
        self.EPISODE_LEN_SEC = 5
        #### Create integrated controllers #########################
        if act in [ActionType.PID, ActionType.ONE_D_PID]:
            os.environ['KMP_DUPLICATE_LIB_OK']='True'
            if self.DRONE_MODEL in [DroneModel.CF2X, DroneModel.CF2P]:
                self.ctrl = [DSLPIDControl(BaseAviary(drone_model=DroneModel.CF2X)) for i in range(self.NUM_DRONES)]
            elif self.DRONE_MODEL == DroneModel.HB:
                self.ctrl = [SimplePIDControl(BaseAviary(drone_model=DroneModel.HB)) for i in range(self.NUM_DRONES)]
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
                         obstacles=True, # Add obstacles for RGB observations and/or FlyThruGate
                         user_debug_gui=False, # Remove of RPM sliders from all single agent learning aviaries
                         vision_attributes=vision_attributes,
                         dynamics_attributes=dynamics_attributes
                         )

    ################################################################################

    def _addObstacles(self):
        """Add obstacles to the environment.

        Only if the observation is of type RGB, 4 landmarks are added.
        Overrides BaseAviary's method.

        """
        if self.OBS_TYPE == ObservationType.RGB:
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
        else:
            pass

    ################################################################################

    def _actionSpace(self):
        """Returns the action space of the environment.

        Returns
        -------
        dict[int, ndarray]
            A Dict() of Box() of size 1, 3, or 3, depending on the action type,
            indexed by drone Id in integer format.

        """
        if self.ACT_TYPE==ActionType.RPM or self.ACT_TYPE==ActionType.DYN:
            size = 4
        elif self.ACT_TYPE==ActionType.PID:
            size = 3
        elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_DYN, ActionType.ONE_D_PID]:
            size = 1
        else:
            print("[ERROR] in BaseMultiagentAviary._actionSpace()")
            exit()
        return spaces.Dict({i: spaces.Box(low=-1*np.ones(size),
                                          high=np.ones(size),
                                          dtype=np.float32
                                          ) for i in range(self.NUM_DRONES)})

    ################################################################################

    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Parameter `action` is processed differenly for each of the different
        action types: the input to n-th drone, `action[n]` can be of length
        1, 3, or 4, and represent RPMs, desired thrust and torques, or the next
        target position to reach using PID control.

        Parameters
        ----------
        action : dict[str, ndarray]
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
                rpm[int(k),:] = np.array(self.HOVER_RPM * (1+0.05*v))
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
                rpm, _, _ = self.ctrl[k].computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                        cur_pos=state[0:3],
                                                        cur_quat=state[3:7],
                                                        cur_vel=state[10:13],
                                                        cur_ang_vel=state[13:16],
                                                        target_pos=state[0:3]+0.1*v
                                                        )
                rpm[int(k),:] = rpm
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
                print("[ERROR] in BaseMultiagentAviary._preprocessAction()")
                exit()
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
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
            # obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            # obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])          
            # return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )
            ############################################################
            #### OBS SPACE OF SIZE 12
            return spaces.Dict({i: spaces.Box(low=np.array([-1,-1,0, -1,-1,-1, -1,-1,-1, -1,-1,-1]),
                                              high=np.array([1,1,1, 1,1,1, 1,1,1, 1,1,1]),
                                              dtype=np.float32
                                              ) for i in range(self.NUM_DRONES)})
            ############################################################
        else:
            print("[ERROR] in BaseMultiagentAviary._observationSpace()")
    
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
                    self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i,
                                                                                 segmentation=False
                                                                                 )
                    #### Printing observation to PNG frames example ############
                    if self.RECORD:
                        self._exportImage(img_type=ImageType.RGB,
                                          img_input=self.rgb[i],
                                          path=self.ONBOARD_IMG_PATH+"drone_"+str(i),
                                          frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ)
                                          )
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
            print("[ERROR] in BaseMultiagentAviary._computeObs()")

    ################################################################################

    def _clipAndNormalizeState(self,
                               state
                               ):
        """Normalizes a drone's state to the [-1,1] range.

        Must be implemented in a subclass.

        Parameters
        ----------
        state : ndarray
            Array containing the non-normalized state of a single drone.

        """
        raise NotImplementedError
