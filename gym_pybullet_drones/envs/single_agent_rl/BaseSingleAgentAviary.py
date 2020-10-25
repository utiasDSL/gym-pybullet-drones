import os
import numpy as np
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
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1,
                    neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                    physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                    gui=False, record=False, obstacles=True, user_debug_gui=True, img_obs=False):
        if num_drones!=1: print("[ERROR] in BaseSingleAgentAviary.__init__(), BaseSingleAgentAviary only accepts num_drones=1"); exit()
        self.IMG_OBS = img_obs
        if self.IMG_OBS:
            self.IMG_RES = np.array([64, 48]); self.IMG_FRAME_PER_SEC = 24; self.IMG_CAPTURE_FREQ = int(freq/self.IMG_FRAME_PER_SEC)
            self.rgb = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0], 4))); self.dep = np.ones(((num_drones, self.IMG_RES[1], self.IMG_RES[0]))); self.seg = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0])))
            if self.IMG_CAPTURE_FREQ%aggregate_phy_steps!=0: print("[ERROR] in BaseSingleAgentAviary.__init__(), aggregate_phy_steps incompatible with the desired video capture frame rate ({:f}Hz)".format(self.IMG_FRAME_PER_SEC)); exit()
        super().__init__(drone_model=drone_model, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui)

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
        return spaces.Box( low=act_lower_bound, high=act_upper_bound, dtype=np.float32 )

    ####################################################################################################
    #### Return the observation space of the environment, a Box(20,) ###################################
    ####################################################################################################
    def _observationSpace(self):
        if self.IMG_OBS: return spaces.Box(low=0, high=255, shape=(self.IMG_RES[1], self.IMG_RES[0], 4), dtype=np.uint8)
        else:
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
            obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])
            return spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )

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
                # DEBUG ONLY
                #### Printing observation to PNG frames example ####################################################
                if self.GUI:
                    path = os.path.dirname(os.path.abspath(__file__))+"/../../../files/test/"; os.makedirs(os.path.dirname(path), exist_ok=True)
                    self._exportImage(img_type=ImageType.RGB, img_input=self.rgb[0], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
                #
            return self.rgb[0]
        else: return self._clipAndNormalizeState(self._getDroneStateVector(0))

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
        rpm = self._normalizedActionToRPM(action)
        return np.clip(np.array(rpm), 0, self.MAX_RPM)

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


