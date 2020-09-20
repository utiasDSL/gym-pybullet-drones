import os
import numpy as np
from gym import spaces

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, ImageType, BaseAviary


######################################################################################################################################################
#### Multi-drone environment class for control applications using visual input #######################################################################
######################################################################################################################################################
class VisionCtrlAviary(BaseAviary):

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
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1,
                        neighbourhood_radius: float=np.inf, initial_xyzs=None, initial_rpys=None,
                        physics: Physics=Physics.PYB, freq: int=240, aggregate_phy_steps: int=1,
                        gui=False, record=False, obstacles=False, user_debug_gui=True):
        self.IMG_RES = np.array([64, 48]); self.IMG_FRAME_PER_SEC = 24; self.IMG_CAPTURE_FREQ = int(freq/self.IMG_FRAME_PER_SEC)
        self.rgb = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0], 4))); self.dep = np.ones(((num_drones, self.IMG_RES[1], self.IMG_RES[0]))); self.seg = np.zeros(((num_drones, self.IMG_RES[1], self.IMG_RES[0])))
        if self.IMG_CAPTURE_FREQ%aggregate_phy_steps!=0: print("[ERROR] in VisionCtrlAviary.__init__(), aggregate_phy_steps incompatible with the desired video capture frame rate ({:f}Hz)".format(self.IMG_FRAME_PER_SEC)); exit()
        super().__init__(drone_model=drone_model, num_drones=num_drones, neighbourhood_radius=neighbourhood_radius,
                            initial_xyzs=initial_xyzs, initial_rpys=initial_rpys, physics=physics, freq=freq,
                            aggregate_phy_steps=aggregate_phy_steps, gui=gui, record=record, obstacles=obstacles, user_debug_gui=user_debug_gui)

    ####################################################################################################
    #### Return the action space of the environment, a Dict of Box(4,) with NUM_DRONES entries #########
    ####################################################################################################
    def _actionSpace(self):
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([0.,           0.,           0.,           0.])
        act_upper_bound = np.array([self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({ str(i): spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32) for i in range(self.NUM_DRONES) })

    ####################################################################################################
    #### Return the observation space of the environment, a Dict with NUM_DRONES entries of Dict of ####
    #### { Box(4,), MultiBinary(NUM_DRONES), Box(H,W,4), Box(H,W), Box(H,W) } ##########################
    ####################################################################################################
    def _observationSpace(self):
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
        obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
        obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({ str(i): spaces.Dict ({"state": spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32),
                                                    "neighbors": spaces.MultiBinary(self.NUM_DRONES),
                                                    "rgb": spaces.Box(low=0, high=255, shape=(self.IMG_RES[1], self.IMG_RES[0], 4), dtype=np.uint8),
                                                    "dep": spaces.Box(low=.01, high=1000., shape=(self.IMG_RES[1], self.IMG_RES[0]), dtype=np.float32),
                                                    "seg": spaces.Box(low=0, high=100, shape=(self.IMG_RES[1], self.IMG_RES[0]), dtype=np.int)
                                                     }) for i in range(self.NUM_DRONES) })

    ####################################################################################################
    #### Return the current observation of the environment #############################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (dict)                       {"0":{"state": np.arr(20,),"neighbors": np.arr(NUM_DRONES),
    ####                                    "rgb": np.arr(h,w,4),"dep": np.arr(h,w),"seg": np.arr(h,w)},
    ####                                    .. "NUM_DRONES-1": {..} } ##################################
    ####                                    for the "state"'s content see _observationSpace() ##########
    ####                                    "neighbors" is the drone's row of the adjacency matrix #####
    ####                                    "rgb" is a matrix of int8 with 4 channels ##################
    ####                                    "dep" is a matrix of floats with 1 channel #################
    ####                                    "seg" is a matrix of int's with 1 channel ##################
    ####################################################################################################
    def _computeObs(self):
        adjacency_mat = self._getAdjacencyMatrix()
        obs = {}
        for i in range(self.NUM_DRONES):
            if self.step_counter%self.IMG_CAPTURE_FREQ==0: self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i)
            obs[str(i)] = {"state": self._getDroneStateVector(i), "neighbors": adjacency_mat[i,:], \
                            "rgb": self.rgb[i], "dep": self.dep[i], "seg": self.seg[i] }
            #### Printing observation to PNG frames example ####################################################
            # path = os.path.dirname(os.path.abspath(__file__))+"/../../files/test/"; os.makedirs(os.path.dirname(path), exist_ok=True)
            # self._exportImage(img_type=ImageType.RGB, img_input=self.rgb[i], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
            ## self._exportImage(img_type=ImageType.BW, img_input=self.rgb[i], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
            ## self._exportImage(img_type=ImageType.DEP, img_input=self.dep[i], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
            ## self._exportImage(img_type=ImageType.SEG, img_input=self.seg[i], path=path, frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ))
        return obs

    ####################################################################################################
    #### Preprocess the action passed to step() ########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (dict of (4,1) array)     unclipped RPMs commanded to the 4 motors of each drone #####
    ####################################################################################################
    #### Returns #######################################################################################
    #### - clip_action ((N_DRONES,4,1) arr) clipped RPMs commanded to the 4 motors of each drone #######
    ####################################################################################################
    def _preprocessAction(self, action):
        clipped_action = np.zeros((self.NUM_DRONES,4))
        for k, v in action.items():
            clipped_action[int(k),:] = np.clip(np.array(v), 0, self.MAX_RPM)
        return clipped_action

    ####################################################################################################
    #### Compute the current reward value(s) ###########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - reward (..)                      the reward(s) associated to the current obs/state ##########
    ####################################################################################################
    def _computeReward(self, obs):
        return -1

    ####################################################################################################
    #### Compute the current done value(s) #############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (..)                        the done value(s) associated to the current obs/state ######
    ####################################################################################################
    def _computeDone(self, obs):
        return False

    ####################################################################################################
    #### Compute the current info dict(s) ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - info (..)                        the info dict(s) associated to the current obs/state #######
    ####################################################################################################
    def _computeInfo(self, obs):
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years


