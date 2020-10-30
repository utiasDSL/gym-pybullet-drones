import os
from enum import Enum
import numpy as np
from scipy.optimize import nnls
from gym import spaces
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, ImageType, BaseAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

######################################################################################################################################################
#### Action type enumeration #########################################################################################################################
######################################################################################################################################################
class ActionType(Enum):
    RPM = "rpm"
    DYN = "dyn"
    PID = "pid"
    ONE_D_RPM = "one_d_rpm"
    ONE_D_DYN = "one_d_dyn"
    ONE_D_PID = "one_d_pid"

######################################################################################################################################################
#### Observation type enumeration #########################################################################################################################
######################################################################################################################################################
class ObservationType(Enum):
    KIN = "kin"
    RGB = "rgb"

######################################################################################################################################################
#### Single drone environment class for reinforcement learning applications (in this implementation, taking off from the origin) #####################
######################################################################################################################################################
class BaseSingleAgentAviary(BaseAviary):

    ####################################################################################################
    #### Initialize the environment ####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - drone_model (DroneModel)         desired drone type (associated to an .urdf file) ###########
    #### - num_drones (int)                 desired number of drones in the aviary #####################
    #### - neighbourhood_radius (float)     used to compute the drones' adjacency matrix, in meters ####
    #### - initial_xyzs ((3,1) array)       initial XYZ position of the drones #########################
    #### - initial_rpys ((3,1) array)       initial orientations of the drones (radians) ###############
    #### - physics (Physics)                desired implementation of physics/dynamics #################
    #### - freq (int)                       the frequency (Hz) at which the physics engine advances ####
    #### - aggregate_phy_steps (int)        number of physics updates within one call of .step() #######
    #### - gui (bool)                       whether to use PyBullet's GUI ##############################
    #### - record (bool)                    whether to save a video of the simulation ##################
    #### - obstacles (bool)                 whether to add obstacles to the simulation #################
    #### - user_debug_gui (bool)            whether to draw the drones' axes and the GUI sliders #######
    #### - 
    ####
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                    gui=False, record=False, 
                    obs: ObservationType=ObservationType.KIN,
                    act: ActionType=ActionType.RPM):
        self.EPISODE_LEN_SEC = 5
        self.OBS_TYPE = obs; self.ACT_TYPE = act
        #### Before super().__init__ to initialize the proper action/obs spaces ############################
        if self.OBS_TYPE==ObservationType.RGB:
            self.IMG_RES = np.array([64, 48]); self.IMG_FRAME_PER_SEC = 24; self.IMG_CAPTURE_FREQ = int(freq/self.IMG_FRAME_PER_SEC)
            self.rgb = np.zeros(((1, self.IMG_RES[1], self.IMG_RES[0], 4))); self.dep = np.ones(((1, self.IMG_RES[1], self.IMG_RES[0]))); self.seg = np.zeros(((1, self.IMG_RES[1], self.IMG_RES[0])))
            if self.IMG_CAPTURE_FREQ%aggregate_phy_steps!=0: print("[ERROR] in BaseSingleAgentAviary.__init__(), aggregate_phy_steps incompatible with the desired video capture frame rate ({:f}Hz)".format(self.IMG_FRAME_PER_SEC)); exit()
        super().__init__(drone_model=drone_model, num_drones=1, initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, 
                            freq=freq, aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, 
                            obstacles=True,          # Add obstacles for RGB observations and/or FlyThruGate
                            user_debug_gui=False)    # Remove of RPM sliders from all single agent learning aviaries
        #### After super().__init__ to use the proper constants ############################################
        if self.ACT_TYPE==ActionType.DYN or self.ACT_TYPE==ActionType.ONE_D_DYN:
            if self.DRONE_MODEL==DroneModel.CF2X: self.A = np.array([ [1, 1, 1, 1], [.5, .5, -.5, -.5], [-.5, .5, .5, -.5], [-1, 1, -1, 1] ])
            elif self.DRONE_MODEL in [DroneModel.CF2P, DroneModel.HB]: self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ])
            self.INV_A = np.linalg.inv(self.A); self.B_COEFF = np.array([1/self.KF, 1/(self.KF*self.L), 1/(self.KF*self.L), 1/self.KM])
        if self.ACT_TYPE==ActionType.PID or self.ACT_TYPE==ActionType.ONE_D_PID:
            os.environ['KMP_DUPLICATE_LIB_OK']='True'
            if self.DRONE_MODEL in [DroneModel.CF2X, DroneModel.CF2P]: self.ctrl = DSLPIDControl(CtrlAviary(drone_model=DroneModel.CF2X))
            elif self.DRONE_MODEL==DroneModel.HB: self.ctrl = SimplePIDControl(CtrlAviary(drone_model=DroneModel.HB))

    ####################################################################################################
    #### Add obstacles to the environment from .urdf files #############################################
    ####################################################################################################
    def _addObstacles(self):
        if self.OBS_TYPE==ObservationType.RGB:
            p.loadURDF("block.urdf", [1,0,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("cube_small.urdf", [0,1,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("duck_vhacd.urdf", [-1,0,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("teddy_vhacd.urdf", [0,-1,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        else: pass

    ####################################################################################################
    #### Return the action space of the environment, a Box(4,) #########################################
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([-1,           -1,           -1,           -1])
        act_upper_bound = np.array([1,            1,            1,            1])
        if self.ACT_TYPE==ActionType.RPM or self.ACT_TYPE==ActionType.DYN: return spaces.Box( low=act_lower_bound, high=act_upper_bound, dtype=np.float32 )
        elif self.ACT_TYPE==ActionType.PID: return spaces.Box( low=-1*np.ones(3), high=np.ones(3), dtype=np.float32 )
        elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_DYN, ActionType.ONE_D_PID]: return spaces.Box( low=np.array([-1]), high=np.array([1]), dtype=np.float32 )
        else: print("[ERROR] in BaseSingleAgentAviary._actionSpace()")

    ####################################################################################################
    #### Preprocess the action passed to step() ########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action ((4,1) array)             unclipped RPMs commanded to the 4 motors of the drone ######
    ####################################################################################################
    #### Returns #######################################################################################
    #### - clipped_action ((4,1) array)     clipped RPMs commanded to the 4 motors of the drone ########
    ####################################################################################################
    def _preprocessAction(self, action):
        if self.ACT_TYPE==ActionType.RPM: return np.array(self.HOVER_RPM * (1+0.05*action))
        elif self.ACT_TYPE==ActionType.DYN: return self._nnlsRPM(thrust=(self.GRAVITY*(action[0]+1)), x_torque=(0.05*self.MAX_XY_TORQUE*action[1]), y_torque=(0.05*self.MAX_XY_TORQUE*action[2]), z_torque=(0.05*self.MAX_Z_TORQUE*action[3]))
        elif self.ACT_TYPE==ActionType.PID: 
            state = self._getDroneStateVector(0)
            rpm, _, _ = self.ctrl.computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                cur_pos=state[0:3], cur_quat=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16],
                                                target_pos=state[0:3]+1*action )
            return rpm
        elif self.ACT_TYPE==ActionType.ONE_D_RPM: return np.repeat(self.HOVER_RPM * (1+0.05*action), 4)
        elif self.ACT_TYPE==ActionType.ONE_D_DYN: return self._nnlsRPM(thrust=(self.GRAVITY*(1+0.05*action[0])), x_torque=0, y_torque=0, z_torque=0)
        elif self.ACT_TYPE==ActionType.ONE_D_PID:
            state = self._getDroneStateVector(0)
            rpm, _, _ = self.ctrl.computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                cur_pos=state[0:3], cur_quat=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16],
                                                target_pos=state[0:3]+1*np.array([0,0,action[0]]) )
            return rpm
        else: print("[ERROR] in BaseSingleAgentAviary._preprocessAction()")

    ####################################################################################################
    #### Return the observation space of the environment, a Box(20,) ###################################
    ####################################################################################################
    def _observationSpace(self):
        if self.OBS_TYPE==ObservationType.RGB: return spaces.Box(low=0, high=255, shape=(self.IMG_RES[1], self.IMG_RES[0], 4), dtype=np.uint8)
        elif self.OBS_TYPE==ObservationType.KIN:
            ###################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
            # obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            # obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])          
            # return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )
            ###################################
            #### OBS SPACE OF SIZE 12
            return spaces.Box( low=np.array([-1,-1,0, -1,-1,-1, -1,-1,-1, -1,-1,-1]), high=np.array([1,1,1, 1,1,1, 1,1,1, 1,1,1]), dtype=np.float32 )
            ###################################
        else: print("[ERROR] in BaseSingleAgentAviary._observationSpace()")

    ####################################################################################################
    #### Return the current observation of the environment #############################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (20,) array                  for its content see _observationSpace() ####################
    ####################################################################################################
    def _computeObs(self):
        if self.OBS_TYPE==ObservationType.RGB:
            if self.step_counter%self.IMG_CAPTURE_FREQ==0: 
                self.rgb[0], self.dep[0], self.seg[0] = self._getDroneImages(0, segmentation=False)
                # DEBUG ONLY, REMOVE TO IMPROVE RENDERING PERFORMANCE
                #### Printing observation to PNG frames example ####################################################
                if self.GUI:
                    path = os.path.dirname(os.path.abspath(__file__))+"/../../../files/test/"; os.makedirs(os.path.dirname(path), exist_ok=True)
                    self._exportImage(img_type=ImageType.RGB, img_input=self.rgb[0], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
                ####################################################################################################
            return self.rgb[0]
        elif self.OBS_TYPE==ObservationType.KIN: 
            obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
            ###################################
            #### OBS OF SIZE 20 (WITH QUATERNION AND RPMS)
            # return obs
            ###################################
            #### OBS SPACE OF SIZE 12
            return np.hstack([ obs[0:3], obs[7:10], obs[10:13], obs[13:16] ]).reshape(12,)
            ###################################
        else: print("[ERROR] in BaseSingleAgentAviary._observationSpace()")

    ####################################################################################################
    #### Normalize the 20 values in the simulation state to the [-1,1] range ###########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - state ((20,1) array)             raw simulation state #######################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - ...
    ####################################################################################################
    def _clipAndNormalizeState(self, state):
        pass

    ####################################################################################################
    #### Non-negative Least Squares (NNLS) RPM from desired thrust and torques  ########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - thrust (float)                   desired thrust along the local z-axis ######################
    #### - x_torque (float)                 desired x-axis torque ######################################
    #### - y_torque (float)                 desired y-axis torque ######################################
    #### - z_torque (float)                 desired z-axis torque ######################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _nnlsRPM(self, thrust, x_torque, y_torque, z_torque):
        #### Check the feasibility of thrust and torques ###################################################
        if self.GUI and (thrust<0 or thrust>self.MAX_THRUST): print("[WARNING] it", self.step_counter, "in DynCtrlAviary._nnlsRPM(), unfeasible thrust {:.2f} outside range [0, {:.2f}]".format(thrust, self.MAX_THRUST))
        if self.GUI and np.abs(x_torque)>self.MAX_XY_TORQUE: print("[WARNING] it", self.step_counter, "in DynCtrlAviary._nnlsRPM(), unfeasible roll torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if self.GUI and np.abs(y_torque)>self.MAX_XY_TORQUE: print("[WARNING] it", self.step_counter, "in DynCtrlAviary._nnlsRPM(), unfeasible pitch torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if self.GUI and np.abs(z_torque)>self.MAX_Z_TORQUE: print("[WARNING] it", self.step_counter, "in DynCtrlAviary._nnlsRPM(), unfeasible yaw torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -self.MAX_Z_TORQUE, self.MAX_Z_TORQUE))
        B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), self.B_COEFF)
        sq_rpm = np.dot(self.INV_A, B)
        #### Use NNLS if any of the desired angular velocities is negative #################################
        if np.min(sq_rpm)<0:
            sol, res = nnls(self.A, B, maxiter=3*self.A.shape[1])
            if self.GUI:
                print("[WARNING] it", self.step_counter, "in DynCtrlAviary._nnlsRPM(), unfeasible squared rotor speeds, using NNLS")
                print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                        "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
                print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                        "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                        "\t\tResidual: {:.2f}".format(res) )
            sq_rpm = sol
        return np.sqrt(sq_rpm)


