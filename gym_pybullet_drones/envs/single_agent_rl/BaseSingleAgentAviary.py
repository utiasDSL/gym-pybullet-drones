import os
from enum import Enum
import numpy as np
from gym import spaces
import pybullet as p

from gym_pybullet_drones.envs.BaseAviary import BaseAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType
from gym_pybullet_drones.utils.utils import nnlsRPM
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl

class ActionType(Enum):
    """Action type enumeration class."""
    RPM = "rpm"                 # RPMS
    DYN = "dyn"                 # Desired thrust and torques
    PID = "pid"                 # PID control
    VEL = "vel"                 # Velocity input (using PID control)
    TUN = "tun"                 # Tune the coefficients of a PID controller
    ONE_D_RPM = "one_d_rpm"     # 1D (identical input to all motors) with RPMs
    ONE_D_DYN = "one_d_dyn"     # 1D (identical input to all motors) with desired thrust and torques
    ONE_D_PID = "one_d_pid"     # 1D (identical input to all motors) with PID control

################################################################################

class ObservationType(Enum):
    """Observation type enumeration class."""
    KIN = "kin"     # Kinematic information (pose, linear and angular velocities)
    RGB = "rgb"     # RGB camera capture in each drone's POV

################################################################################

class BaseSingleAgentAviary(BaseAviary):
    """Base single drone environment class for reinforcement learning."""
    
    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
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
        """Initialization of a generic single agent RL environment.

        Attribute `num_drones` is automatically set to 1; `vision_attributes`
        and `dynamics_attributes` are selected based on the choice of `obs`
        and `act`; `obstacles` is set to True and overridden with landmarks for
        vision applications; `user_debug_gui` is set to False for performance.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
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
        vision_attributes = True if obs == ObservationType.RGB else False
        dynamics_attributes = True if act in [ActionType.DYN, ActionType.ONE_D_DYN] else False
        self.OBS_TYPE = obs
        self.ACT_TYPE = act
        self.EPISODE_LEN_SEC = 5
        #### Create integrated controllers #########################
        if act in [ActionType.PID, ActionType.VEL, ActionType.TUN, ActionType.ONE_D_PID]:
            os.environ['KMP_DUPLICATE_LIB_OK']='True'
            if drone_model in [DroneModel.CF2X, DroneModel.CF2P]:
                self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
                if act == ActionType.TUN:
                    self.TUNED_P_POS = np.array([.4, .4, 1.25])
                    self.TUNED_I_POS = np.array([.05, .05, .05])
                    self.TUNED_D_POS = np.array([.2, .2, .5])
                    self.TUNED_P_ATT = np.array([70000., 70000., 60000.])
                    self.TUNED_I_ATT = np.array([.0, .0, 500.])
                    self.TUNED_D_ATT = np.array([20000., 20000., 12000.])
            elif drone_model == DroneModel.HB:
                self.ctrl = SimplePIDControl(drone_model=DroneModel.HB)
                if act == ActionType.TUN:
                    self.TUNED_P_POS = np.array([.1, .1, .2])
                    self.TUNED_I_POS = np.array([.0001, .0001, .0001])
                    self.TUNED_D_POS = np.array([.3, .3, .4])
                    self.TUNED_P_ATT = np.array([.3, .3, .05])
                    self.TUNED_I_ATT = np.array([.0001, .0001, .0001])
                    self.TUNED_D_ATT = np.array([.3, .3, .5])
            else:
                print("[ERROR] in BaseSingleAgentAviary.__init()__, no controller is available for the specified drone_model")
        super().__init__(drone_model=drone_model,
                         num_drones=1,
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
        #### Set a limit on the maximum target speed ###############
        if act == ActionType.VEL:
            self.SPEED_LIMIT = 0.03 * self.MAX_SPEED_KMH * (1000/3600)
        #### Try _trajectoryTrackingRPMs exists IFF ActionType.TUN #
        if act == ActionType.TUN and not (hasattr(self.__class__, '_trajectoryTrackingRPMs') and callable(getattr(self.__class__, '_trajectoryTrackingRPMs'))):
                print("[ERROR] in BaseSingleAgentAviary.__init__(), ActionType.TUN requires an implementation of _trajectoryTrackingRPMs in the instantiated subclass")
                exit()

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
        ndarray
            A Box() of size 1, 3, 4, or 6 depending on the action type.

        """
        if self.ACT_TYPE == ActionType.TUN:
            size = 6
        elif self.ACT_TYPE in [ActionType.RPM, ActionType.DYN, ActionType.VEL]:
            size = 4
        elif self.ACT_TYPE == ActionType.PID:
            size = 3
        elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_DYN, ActionType.ONE_D_PID]:
            size = 1
        else:
            print("[ERROR] in BaseSingleAgentAviary._actionSpace()")
            exit()
        return spaces.Box(low=-1*np.ones(size),
        # return spaces.Box(low=np.zeros(size),  # Alternative action space, see PR #32
                          high=np.ones(size),
                          dtype=np.float32
                          )

    ################################################################################

    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Parameter `action` is processed differenly for each of the different
        action types: `action` can be of length 1, 3, 4, or 6 and represent 
        RPMs, desired thrust and torques, the next target position to reach 
        using PID control, a desired velocity vector, new PID coefficients, etc.

        Parameters
        ----------
        action : ndarray
            The input action for each drone, to be translated into RPMs.

        Returns
        -------
        ndarray
            (4,)-shaped array of ints containing to clipped RPMs
            commanded to the 4 motors of each drone.

        """
        if self.ACT_TYPE == ActionType.TUN:
            self.ctrl.setPIDCoefficients(p_coeff_pos=(action[0]+1)*self.TUNED_P_POS,
                                         i_coeff_pos=(action[1]+1)*self.TUNED_I_POS,
                                         d_coeff_pos=(action[2]+1)*self.TUNED_D_POS,
                                         p_coeff_att=(action[3]+1)*self.TUNED_P_ATT,
                                         i_coeff_att=(action[4]+1)*self.TUNED_I_ATT,
                                         d_coeff_att=(action[5]+1)*self.TUNED_D_ATT
                                         )
            return self._trajectoryTrackingRPMs() 
        elif self.ACT_TYPE == ActionType.RPM:
            return np.array(self.HOVER_RPM * (1+0.05*action))
        elif self.ACT_TYPE == ActionType.DYN:
            return nnlsRPM(thrust=(self.GRAVITY*(action[0]+1)),
                           x_torque=(0.05*self.MAX_XY_TORQUE*action[1]),
                           y_torque=(0.05*self.MAX_XY_TORQUE*action[2]),
                           z_torque=(0.05*self.MAX_Z_TORQUE*action[3]),
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
            state = self._getDroneStateVector(0)
            rpm, _, _ = self.ctrl.computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                 cur_pos=state[0:3],
                                                 cur_quat=state[3:7],
                                                 cur_vel=state[10:13],
                                                 cur_ang_vel=state[13:16],
                                                 target_pos=state[0:3]+0.1*action
                                                 )
            return rpm
        elif self.ACT_TYPE == ActionType.VEL:
            state = self._getDroneStateVector(0)
            if np.linalg.norm(action[0:3]) != 0:
                v_unit_vector = action[0:3] / np.linalg.norm(action[0:3])
            else:
                v_unit_vector = np.zeros(3)
            rpm, _, _ = self.ctrl.computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                 cur_pos=state[0:3],
                                                 cur_quat=state[3:7],
                                                 cur_vel=state[10:13],
                                                 cur_ang_vel=state[13:16],
                                                 target_pos=state[0:3], # same as the current position
                                                 target_rpy=np.array([0,0,state[9]]), # keep current yaw
                                                 target_vel=self.SPEED_LIMIT * np.abs(action[3]) * v_unit_vector # target the desired velocity vector
                                                 )
            return rpm
        elif self.ACT_TYPE == ActionType.ONE_D_RPM:
            return np.repeat(self.HOVER_RPM * (1+0.05*action), 4)
        elif self.ACT_TYPE == ActionType.ONE_D_DYN:
            return nnlsRPM(thrust=(self.GRAVITY*(1+0.05*action[0])),
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
            state = self._getDroneStateVector(0)
            rpm, _, _ = self.ctrl.computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                 cur_pos=state[0:3],
                                                 cur_quat=state[3:7],
                                                 cur_vel=state[10:13],
                                                 cur_ang_vel=state[13:16],
                                                 target_pos=state[0:3]+0.1*np.array([0,0,action[0]])
                                                 )
            return rpm
        else:
            print("[ERROR] in BaseSingleAgentAviary._preprocessAction()")

    ################################################################################

    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------
        ndarray
            A Box() of shape (H,W,4) or (12,) depending on the observation type.

        """
        if self.OBS_TYPE == ObservationType.RGB:
            return spaces.Box(low=0,
                              high=255,
                              shape=(self.IMG_RES[1], self.IMG_RES[0], 4),
                              dtype=np.uint8
                              )
        elif self.OBS_TYPE == ObservationType.KIN:
            ############################################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
            # obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            # obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])          
            # return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )
            ############################################################
            #### OBS SPACE OF SIZE 12
            return spaces.Box(low=np.array([-1,-1,0, -1,-1,-1, -1,-1,-1, -1,-1,-1]),
                              high=np.array([1,1,1, 1,1,1, 1,1,1, 1,1,1]),
                              dtype=np.float32
                              )
            ############################################################
        else:
            print("[ERROR] in BaseSingleAgentAviary._observationSpace()")
    
    ################################################################################

    def _computeObs(self):
        """Returns the current observation of the environment.

        Returns
        -------
        ndarray
            A Box() of shape (H,W,4) or (12,) depending on the observation type.

        """
        if self.OBS_TYPE == ObservationType.RGB:
            if self.step_counter%self.IMG_CAPTURE_FREQ == 0: 
                self.rgb[0], self.dep[0], self.seg[0] = self._getDroneImages(0,
                                                                             segmentation=False
                                                                             )
                #### Printing observation to PNG frames example ############
                if self.RECORD:
                    self._exportImage(img_type=ImageType.RGB,
                                      img_input=self.rgb[0],
                                      path=self.ONBOARD_IMG_PATH,
                                      frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ)
                                      )
            return self.rgb[0]
        elif self.OBS_TYPE == ObservationType.KIN: 
            obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
            ############################################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            # return obs
            ############################################################
            #### OBS SPACE OF SIZE 12
            ret = np.hstack([obs[0:3], obs[7:10], obs[10:13], obs[13:16]]).reshape(12,)
            return ret.astype('float32')
            ############################################################
        else:
            print("[ERROR] in BaseSingleAgentAviary._computeObs()")
    
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
