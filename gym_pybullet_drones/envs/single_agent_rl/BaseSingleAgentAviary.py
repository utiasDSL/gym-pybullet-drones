import os
import numpy as np
from scipy.optimize import nnls
from gym import spaces
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, ImageType, BaseAviary


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
    #### - img_obs (bool)                   ...
    #### - dyn_input (bool)                   ...
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1,
                    neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                    gui=False, record=False, obstacles=True, user_debug_gui=True, img_obs=False, dyn_input=False):
        if num_drones!=1: print("[ERROR] in BaseSingleAgentAviary.__init__(), BaseSingleAgentAviary only accepts num_drones=1"); exit()
        self.IMG_OBS = img_obs
        self.DYN_IN = dyn_input
        if self.IMG_OBS:
            self.IMG_RES = np.array([64, 48]); self.IMG_FRAME_PER_SEC = 24; self.IMG_CAPTURE_FREQ = int(freq/self.IMG_FRAME_PER_SEC)
            self.rgb = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0], 4))); self.dep = np.ones(((num_drones, self.IMG_RES[1], self.IMG_RES[0]))); self.seg = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0])))
            if self.IMG_CAPTURE_FREQ%aggregate_phy_steps!=0: print("[ERROR] in BaseSingleAgentAviary.__init__(), aggregate_phy_steps incompatible with the desired video capture frame rate ({:f}Hz)".format(self.IMG_FRAME_PER_SEC)); exit()
        super().__init__(drone_model=drone_model, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui)
        if self.DYN_IN:
            if self.DRONE_MODEL==DroneModel.CF2X: self.A = np.array([ [1, 1, 1, 1], [.5, .5, -.5, -.5], [-.5, .5, .5, -.5], [-1, 1, -1, 1] ])
            elif self.DRONE_MODEL in [DroneModel.CF2P, DroneModel.HB]: self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ])
            self.INV_A = np.linalg.inv(self.A); self.B_COEFF = np.array([1/self.KF, 1/(self.KF*self.L), 1/(self.KF*self.L), 1/self.KM])
        

    ####################################################################################################
    #### Add obstacles to the environment from .urdf files #############################################
    ####################################################################################################
    def _addObstacles(self):
        if self.IMG_OBS: 
            p.loadURDF("block.urdf", [1,0,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("cube_small.urdf", [0,1,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("duck_vhacd.urdf", [-1,0,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("teddy_vhacd.urdf", [0,-1,.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Return the action space of the environment, a Box(4,) #########################################
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([-1,           -1,           -1,           -1])
        act_upper_bound = np.array([1,            1,            1,            1])
        # return spaces.Box( low=act_lower_bound, high=act_upper_bound, dtype=np.float32 )
##### REMOVE
############ 
        return spaces.Box( low=np.array([-1]), high=np.array([1]), dtype=np.float32 )

    ####################################################################################################
    #### Return the observation space of the environment, a Box(20,) ###################################
    ####################################################################################################
    def _observationSpace(self):
        if self.IMG_OBS: return spaces.Box(low=0, high=255, shape=(self.IMG_RES[1], self.IMG_RES[0], 4), dtype=np.uint8)
        else:
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
            obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])          
            # return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )
##### REMOVE
############
            return spaces.Box( low=np.array([0,-1]), high=np.array([1,1]), dtype=np.float32 )
            # return spaces.Box( low=np.array([-1,-1,0, -1,-1,-1, -1,-1,-1, -1,-1,-1]), high=np.array([1,1,1, 1,1,1, 1,1,1, 1,1,1]), dtype=np.float32 )

    ####################################################################################################
    #### Return the current observation of the environment #############################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (20,) array                  for its content see _observationSpace() ####################
    ####################################################################################################
    def _computeObs(self):
        if self.IMG_OBS:
            if self.step_counter%self.IMG_CAPTURE_FREQ==0: 
                self.rgb[0], self.dep[0], self.seg[0] = self._getDroneImages(0, segmentation=False)
                # DEBUG ONLY, REMOVE TO IMPROVE RENDERING PERFORMANCE
                #### Printing observation to PNG frames example ####################################################
                if self.GUI:
                    path = os.path.dirname(os.path.abspath(__file__))+"/../../../files/test/"; os.makedirs(os.path.dirname(path), exist_ok=True)
                    self._exportImage(img_type=ImageType.RGB, img_input=self.rgb[0], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
                ####################################################################################################
            return self.rgb[0]
        else: 
            obs = self._clipAndNormalizeState(self._getDroneStateVector(0))
            # return obs
##### REMOVE
############            
            return np.hstack([ obs[2], obs[12] ])
            # return np.hstack([ obs[0:3], obs[7:10], obs[10:13], obs[13:15] ])

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
        if self.DYN_IN:
            return self._nnlsRPM(thrust=self.MAX_THRUST*(action[0]+1)/2, x_torque=self.MAX_XY_TORQUE*action[1], y_torque=self.MAX_XY_TORQUE*action[2], z_torque=self.MAX_Z_TORQUE*action[3])
##### REMOVE
############ 
            # return self._nnlsRPM(thrust=self.MAX_THRUST*(action[0]+1)/2, x_torque=0, y_torque=0, z_torque=0) # DEBUG ONLY
        else:
            rpm = self._normalizedActionToRPM(action)
            # return np.clip(np.array(rpm), 0, self.MAX_RPM)
##### REMOVE
############    
            return np.repeat(self.HOVER_RPM+action*self.HOVER_RPM/20, 4) # DEBUG ONLY, single RPM
            # return np.repeat(self.HOVER_RPM+action*self.HOVER_RPM/20, 1) # DEBUG ONLY, 4 RPMs

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
