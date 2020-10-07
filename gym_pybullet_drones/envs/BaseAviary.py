import os
import time
import collections
from datetime import datetime
from enum import Enum
import xml.etree.ElementTree as etxml
from PIL import Image
import pkgutil
egl = pkgutil.get_loader('eglRenderer')
import numpy as np
import pybullet as p
import pybullet_data
import gym


######################################################################################################################################################
#### Drone models enumeration ########################################################################################################################
######################################################################################################################################################
class DroneModel(Enum):
    CF2X = 0                 # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = 1                 # Bitcraze Craziflie 2.0 in the + configuration
    HB = 2                   # Generic quadrotor (with AscTec Hummingbird inertial properties)

    #### String representation of DroneModel ###########################################################
    def __str__(self): return self.name


######################################################################################################################################################
#### Physics implementations enumeration #############################################################################################################
######################################################################################################################################################
class Physics(Enum):
    PYB = 0                  # Base PyBullet physics update
    DYN = 1                  # Update with an explicit model of the dynamics
    PYB_GND = 2              # PyBullet physics update with ground effect
    PYB_DRAG = 3             # PyBullet physics update with drag
    PYB_DW = 4               # PyBullet physics update with downwash
    PYB_GND_DRAG_DW = 5      # PyBullet physics update with ground effect, drag, and downwash

    #### String representation of Physics ##############################################################
    def __str__(self): return self.name


######################################################################################################################################################
#### Camera capture image type enumeration ###########################################################################################################
######################################################################################################################################################
class ImageType(Enum):
    RGB = 0                  # Red, green, blue (and alpha)
    DEP = 1                  # Depth
    SEG = 2                  # Segmentation by object id
    BW = 3                   # Black and white


######################################################################################################################################################
#### Base multi-drone environment class ##############################################################################################################
######################################################################################################################################################
class BaseAviary(gym.Env):
    metadata = {'render.modes': ['human']}

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
        #### Constants #####################################################################################
        self.G = 9.8; self.RAD2DEG = 180/np.pi; self.DEG2RAD = np.pi/180
        self.SIM_FREQ = freq; self.TIMESTEP = 1./self.SIM_FREQ; self.AGGR_PHY_STEPS = aggregate_phy_steps
        #### Parameters ####################################################################################
        self.NUM_DRONES = num_drones; self.NEIGHBOURHOOD_RADIUS = neighbourhood_radius
        #### Options #######################################################################################
        self.DRONE_MODEL = drone_model; self.GUI = gui; self.RECORD = record; self.PHYSICS = physics
        self.OBSTACLES = obstacles; self.USER_DEBUG = user_debug_gui
        if self.DRONE_MODEL==DroneModel.CF2X: self.URDF = "cf2x.urdf"
        elif self.DRONE_MODEL==DroneModel.CF2P: self.URDF = "cf2p.urdf"
        elif self.DRONE_MODEL==DroneModel.HB: self.URDF = "hb.urdf"
        #### Load the drone properties from the .urdf file #################################################
        self.M, self.L, self.THRUST2WEIGHT_RATIO, self.J, self.J_INV, self.KF, self.KM, self.COLLISION_H, self.COLLISION_R, self.COLLISION_Z_OFFSET, self.MAX_SPEED_KMH, self.GND_EFF_COEFF, self.PROP_RADIUS, self.DRAG_COEFF, self.DW_COEFF_1, self.DW_COEFF_2, self.DW_COEFF_3 = self._parseURDFParameters()
        print("[INFO] BaseAviary.__init__() loaded parameters from the drone's .urdf:\n[INFO] m {:f}, L {:f},\n[INFO] ixx {:f}, iyy {:f}, izz {:f},\n[INFO] kf {:f}, km {:f},\n[INFO] t2w {:f}, max_speed_kmh {:f},\n[INFO] gnd_eff_coeff {:f}, prop_radius {:f},\n[INFO] drag_xy_coeff {:f}, drag_z_coeff {:f},\n[INFO] dw_coeff_1 {:f}, dw_coeff_2 {:f}, dw_coeff_3 {:f}".format(
                                                                        self.M, self.L, self.J[0,0], self.J[1,1], self.J[2,2], self.KF, self.KM, self.THRUST2WEIGHT_RATIO, self.MAX_SPEED_KMH, self.GND_EFF_COEFF, self.PROP_RADIUS, self.DRAG_COEFF[0], self.DRAG_COEFF[2], self.DW_COEFF_1, self.DW_COEFF_2, self.DW_COEFF_3) )
        #### Compute constants #############################################################################
        self.GRAVITY = self.G*self.M; self.HOVER_RPM = np.sqrt(self.GRAVITY/(4*self.KF))
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY)/(4*self.KF)); self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2); self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)
        self.GND_EFF_H_CLIP = 0.25 * self.PROP_RADIUS * np.sqrt( (15 * self.MAX_RPM**2 * self.KF * self.GND_EFF_COEFF) / self.MAX_THRUST )
        #### Connect to PyBullet ###########################################################################
        if self.GUI:
            #### With debug GUI ################################################################################
            self.CLIENT = p.connect(p.GUI) # p.connect(p.GUI, options="--opengl2")
            for i in [p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW]: p.configureDebugVisualizer(i, 0, physicsClientId=self.CLIENT)
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0,0,0], physicsClientId=self.CLIENT)
            ret = p.getDebugVisualizerCamera(physicsClientId=self.CLIENT); print("viewMatrix", ret[2]); print("projectionMatrix", ret[3])
            if self.USER_DEBUG:
                #### Add input sliders to the GUI ##################################################################
                self.SLIDERS = -1*np.ones(4)
                for i in range(4): self.SLIDERS[i] = p.addUserDebugParameter("Propeller "+str(i)+" RPM", 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
                self.INPUT_SWITCH = p.addUserDebugParameter("Use GUI RPM", 9999, -1, 0, physicsClientId=self.CLIENT)
        else:
            #### Without debug GUI #############################################################################
            self.CLIENT = p.connect(p.DIRECT)
            # p.setAdditionalSearchPath(pybullet_data.getDataPath()); plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin"); print("plugin=", plugin)
            if self.RECORD:
                #### Set the camera parameters to save frames in DIRECT mode #######################################
                self.VID_WIDTH=int(640); self.VID_HEIGHT=int(480); self.FRAME_PER_SEC = 24; self.CAPTURE_FREQ = int(self.SIM_FREQ/self.FRAME_PER_SEC)
                self.CAM_VIEW = p.computeViewMatrixFromYawPitchRoll(distance=3, yaw=-30, pitch=-30, roll=0, cameraTargetPosition=[0,0,0], upAxisIndex=2, physicsClientId=self.CLIENT)
                self.CAM_PRO = p.computeProjectionMatrixFOV(fov=60.0, aspect=self.VID_WIDTH/self.VID_HEIGHT, nearVal=0.1, farVal=1000.0)
        #### Set initial poses #############################################################################
        if initial_xyzs is None: self.INIT_XYZS = np.vstack([ np.array([x*4*self.L for x in range(self.NUM_DRONES)]), np.array([y*4*self.L for y in range(self.NUM_DRONES)]), np.ones(self.NUM_DRONES) * (self.COLLISION_H/2-self.COLLISION_Z_OFFSET+.1) ]).transpose().reshape(self.NUM_DRONES,3)
        elif np.array(initial_xyzs).shape==(self.NUM_DRONES,3): self.INIT_XYZS = initial_xyzs
        else: print("[ERROR] invalid initial_xyzs in BaseAviary.__init__(), try initial_xyzs.reshape(NUM_DRONES,3)")
        if initial_rpys is None: self.INIT_RPYS = np.zeros((self.NUM_DRONES,3))
        elif np.array(initial_rpys).shape==(self.NUM_DRONES,3): self.INIT_RPYS = initial_rpys
        else: print("[ERROR] invalid initial_rpys in BaseAviary.__init__(), try initial_rpys.reshape(NUM_DRONES,3)")
        #### Create action and observation spaces ##########################################################
        self.action_space = self._actionSpace()
        self.observation_space = self._observationSpace()
        #### Housekeeping ##################################################################################
        self._housekeeping()
        #### Update and store the drones kinematic information #############################################
        self._updateAndStoreKinematicInformation()
        #### Start video recording #########################################################################
        self._startVideoRecording()

    ####################################################################################################
    #### Reset the environment #########################################################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (..)                         initial observation, see _computeObs() in the child class ##
    ####################################################################################################
    def reset(self):
        p.resetSimulation(physicsClientId=self.CLIENT)
        #### Housekeeping ##################################################################################
        self._housekeeping()
        #### Update and store the drones kinematic information #############################################
        self._updateAndStoreKinematicInformation()
        #### Start video recording #########################################################################
        self._startVideoRecording()
        #### Return the initial observation ################################################################
        return self._computeObs()

    ####################################################################################################
    #### Advance the environment by one simulation step ################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (..)                      to motors' speed, see _preprocessAction() in the child class
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (..)                         observation, see _computeObs() in the child class ##########
    #### - reward (..)                      reward value, see _computeReward() in the child class ######
    #### - done (..)                        whether the current episode is over, see computeDone() #####
    #### - info (Dict)                      currently unused, see _computeInfo() in the child class ####
    ####################################################################################################
    def step(self, action):
        #### Save a video frame in PNG format if RECORD=True and GUI=False #################################
        if self.RECORD and not self.GUI and self.step_counter%self.CAPTURE_FREQ==0:
            [w, h, rgb, dep, seg] = p.getCameraImage(width=self.VID_WIDTH, height=self.VID_HEIGHT, shadow=1, viewMatrix=self.CAM_VIEW,
                projectionMatrix=self.CAM_PRO, renderer=p.ER_TINY_RENDERER, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, physicsClientId=self.CLIENT)
            (Image.fromarray(np.reshape(rgb, (h, w, 4)), 'RGBA')).save(self.IMG_PATH+"frame_"+str(self.FRAME_NUM)+".png")
            #### Save the depth or segmentation view instead ###################################################
            # dep = ((dep-np.min(dep)) * 255 / (np.max(dep)-np.min(dep))).astype('uint8'); (Image.fromarray(np.reshape(dep, (h, w)))).save(self.IMG_PATH+"frame_"+str(self.FRAME_NUM)+".png")
            # seg = ((seg-np.min(seg)) * 255 / (np.max(seg)-np.min(seg))).astype('uint8'); (Image.fromarray(np.reshape(seg, (h, w)))).save(self.IMG_PATH+"frame_"+str(self.FRAME_NUM)+".png")
            self.FRAME_NUM += 1
        #### Read the GUI's input parameters ###############################################################
        if self.GUI and self.USER_DEBUG:
            current_input_switch = p.readUserDebugParameter(self.INPUT_SWITCH, physicsClientId=self.CLIENT)
            if current_input_switch>self.last_input_switch:
                self.last_input_switch = current_input_switch
                self.USE_GUI_RPM = True if self.USE_GUI_RPM==False else False
        if self.USE_GUI_RPM:
            for i in range(4): self.gui_input[i] = p.readUserDebugParameter(int(self.SLIDERS[i]), physicsClientId=self.CLIENT)
            clipped_action = np.tile(self.gui_input,(self.NUM_DRONES,1))
            if self.step_counter%(self.SIM_FREQ/2)==0: self.GUI_INPUT_TEXT = [ p.addUserDebugText("Using GUI RPM", textPosition=[0,0,0], textColorRGB=[1,0,0], lifeTime=1, textSize=2, parentObjectUniqueId=self.DRONE_IDS[i], parentLinkIndex=-1, replaceItemUniqueId=int(self.GUI_INPUT_TEXT[i]), physicsClientId=self.CLIENT) for i in range(self.NUM_DRONES) ]
        #### Save, preprocess, and clip the action to the maximum RPM ######################################
        else: self._saveLastAction(action); clipped_action = np.reshape(self._preprocessAction(action), (self.NUM_DRONES,4))
        #### Repeat for as many as the aggregate physics steps/dynamics updates ############################
        for _ in range(self.AGGR_PHY_STEPS):
            #### Re-update and store the drones kinematic info to use DYN or compute the aerodynamics effects ##
            if self.AGGR_PHY_STEPS>1 and self.PHYSICS in [Physics.DYN, Physics.PYB_GND, Physics.PYB_DRAG, Physics.PYB_DW, Physics.PYB_GND_DRAG_DW]:
                self._updateAndStoreKinematicInformation()
            #### Step the simulation using the desired physics update ##########################################
            for i in range (self.NUM_DRONES):
                if self.PHYSICS==Physics.PYB: self._physics(clipped_action[i,:], i)
                elif self.PHYSICS==Physics.DYN: self._dynamics(clipped_action[i,:], i)
                elif self.PHYSICS==Physics.PYB_GND: self._physics(clipped_action[i,:], i); self._groundEffect(clipped_action[i,:], i)
                elif self.PHYSICS==Physics.PYB_DRAG: self._physics(clipped_action[i,:], i); self._drag(self.last_clipped_action[i,:], i)
                elif self.PHYSICS==Physics.PYB_DW: self._physics(clipped_action[i,:], i); self._downwash(i)
                elif self.PHYSICS==Physics.PYB_GND_DRAG_DW: self._physics(clipped_action[i,:], i); self._groundEffect(clipped_action[i,:], i); self._drag(self.last_clipped_action[i,:], i); self._downwash(i)
            #### Let PyBullet compute the new state, unless using Physics.DYN ##################################
            if self.PHYSICS!=Physics.DYN: p.stepSimulation(physicsClientId=self.CLIENT)
            #### Save the last applied action to compute drag in the next step #################################
            if self.PHYSICS in [Physics.PYB_DRAG, Physics.PYB_GND_DRAG_DW]: self.last_clipped_action = clipped_action
        #### Update and store the drones kinematic information #############################################
        self._updateAndStoreKinematicInformation()
        #### Prepare the return values #####################################################################
        obs = self._computeObs()
        reward = self._computeReward(obs)
        done = self._computeDone(obs)
        info = self._computeInfo(obs)
        #### Advance the step counter ######################################################################
        self.step_counter = self.step_counter + (1 * self.AGGR_PHY_STEPS)
        return obs, reward, done, info

    ####################################################################################################
    #### Print a textual output of the environment #####################################################
    ####################################################################################################
    def render(self, mode='human', close=False):
        if self.first_render_call and not self.GUI:
            print("[WARNING] BaseAviary.render() is implemented as text-only, re-initialize the environment using Aviary(gui=True) to use PyBullet's graphical interface")
            self.first_render_call = False
        print("\n[INFO] BaseAviary.render() ——— it {:04d}".format(self.step_counter),
            "——— wall-clock time {:.1f}s,".format(time.time()-self.RESET_TIME),
            "simulation time {:.1f}s@{:d}Hz ({:.2f}x)".format(self.step_counter*self.TIMESTEP, self.SIM_FREQ, (self.step_counter*self.TIMESTEP)/(time.time()-self.RESET_TIME)))
        for i in range (self.NUM_DRONES):
            print("[INFO] BaseAviary.render() ——— drone {:d}".format(i),
            "——— x {:+06.2f}, y {:+06.2f}, z {:+06.2f}".format(self.pos[i,0], self.pos[i,1], self.pos[i,2]),
            "——— velocity {:+06.2f}, {:+06.2f}, {:+06.2f}".format(self.vel[i,0], self.vel[i,1], self.vel[i,2]),
            "——— roll {:+06.2f}, pitch {:+06.2f}, yaw {:+06.2f}".format(self.rpy[i,0]*self.RAD2DEG, self.rpy[i,1]*self.RAD2DEG, self.rpy[i,2]*self.RAD2DEG),
            "——— angular velocities {:+06.2f}, {:+06.2f}, {:+06.2f} ——— ".format(self.ang_v[i,0]*self.RAD2DEG, self.ang_v[i,1]*self.RAD2DEG, self.ang_v[i,2]*self.RAD2DEG))

    ####################################################################################################
    #### Close the environment #########################################################################
    ####################################################################################################
    def close(self):
        if self.RECORD and self.GUI: p.stopStateLogging(self.VIDEO_ID, physicsClientId=self.CLIENT)
        p.disconnect(physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Return the PyBullet Client Id #################################################################
    ####################################################################################################
    def getPyBulletClient(self):
        return self.CLIENT

    ####################################################################################################
    #### Return the Drone Id ###########################################################################
    ####################################################################################################
    def getDroneIds(self):
        return self.DRONE_IDS

    ####################################################################################################
    #### Housekeeping of variables and PyBullet's parameters/objects in the reset() function ###########
    ####################################################################################################
    def _housekeeping(self):
        #### Initialize/reset counters and zero-valued variables ###########################################
        self.RESET_TIME = time.time(); self.step_counter = 0; self.first_render_call = True
        self.X_AX = -1*np.ones(self.NUM_DRONES); self.Y_AX = -1*np.ones(self.NUM_DRONES); self.Z_AX = -1*np.ones(self.NUM_DRONES);
        self.GUI_INPUT_TEXT = -1*np.ones(self.NUM_DRONES); self.USE_GUI_RPM=False; self.last_input_switch = 0
        self.last_action = -1*np.ones((self.NUM_DRONES,4))
        self.last_clipped_action = np.zeros((self.NUM_DRONES,4)); self.gui_input = np.zeros(4)
        self.no_pybullet_dyn_accs = np.zeros((self.NUM_DRONES,3))
        #### Initialize the drones kinemaatic information ##################################################
        self.pos = np.zeros((self.NUM_DRONES,3)); self.quat = np.zeros((self.NUM_DRONES,4)); self.rpy = np.zeros((self.NUM_DRONES,3))
        self.vel = np.zeros((self.NUM_DRONES,3)); self.ang_v = np.zeros((self.NUM_DRONES,3))
        #### Set PyBullet's parameters #####################################################################
        p.setGravity(0, 0, -self.G, physicsClientId=self.CLIENT)
        p.setRealTimeSimulation(0, physicsClientId=self.CLIENT)
        p.setTimeStep(self.TIMESTEP, physicsClientId=self.CLIENT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.CLIENT)
        #### Load ground plane, drone and obstacles models #################################################
        self.PLANE_ID = p.loadURDF("plane.urdf", physicsClientId=self.CLIENT)
        self.DRONE_IDS = np.array([p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/"+self.URDF, self.INIT_XYZS[i,:], p.getQuaternionFromEuler(self.INIT_RPYS[i,:]), physicsClientId=self.CLIENT) for i in range(self.NUM_DRONES)])
        for i in range(self.NUM_DRONES):
            #### Show the frame of reference of the drone, note thet it can severly slow down the GUI ##########
            if self.GUI and self.USER_DEBUG: self._showDroneLocalAxes(i)
            #### Disable collisions between drones' and the ground plane, e.g., to start a drone at [0,0,0] ####
            # p.setCollisionFilterPair(bodyUniqueIdA=self.PLANE_ID, bodyUniqueIdB=self.DRONE_IDS[i], linkIndexA=-1, linkIndexB=-1, enableCollision=0, physicsClientId=self.CLIENT)
        if self.OBSTACLES: self._addObstacles()

    ####################################################################################################
    #### Update and store the drones kinemaatic information ############################################
    ####################################################################################################
    def _updateAndStoreKinematicInformation(self):
        for i in range (self.NUM_DRONES):
            self.pos[i], self.quat[i] = p.getBasePositionAndOrientation(self.DRONE_IDS[i], physicsClientId=self.CLIENT)
            self.rpy[i] = p.getEulerFromQuaternion(self.quat[i])
            self.vel[i], self.ang_v[i] = p.getBaseVelocity(self.DRONE_IDS[i], physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Start saving the video output as .mp4, if GUI is True,  or .png, otherwise ####################
    ####################################################################################################
    def _startVideoRecording(self):
        if self.RECORD and self.GUI: self.VIDEO_ID = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=os.path.dirname(os.path.abspath(__file__))+"/../../files/video-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".mp4", physicsClientId=self.CLIENT)
        if self.RECORD and not self.GUI: self.FRAME_NUM = 0; self.IMG_PATH = os.path.dirname(os.path.abspath(__file__))+"/../../files/video-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+"/"; os.makedirs(os.path.dirname(self.IMG_PATH), exist_ok=True)

    ####################################################################################################
    #### Return the state vector of the nth drone ######################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - state ((20,) array)              the state vector of the nth drone ##########################
    ####################################################################################################
    def _getDroneStateVector(self, nth_drone):
        state = np.hstack([self.pos[nth_drone,:], self.quat[nth_drone,:], self.rpy[nth_drone,:], self.vel[nth_drone,:], self.ang_v[nth_drone,:], self.last_action[nth_drone,:]])
        return state.reshape(20,)

    ####################################################################################################
    #### Return camera captures from the nth drone POV #################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    #### - segmentation (bool)              whehter to compute the compute the segmentation mask #######
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rgb ((h,w,4) array)              RBG(A) image captured from the nth_drone's POV #############
    #### - dep ((h,w) array)                depth image captured from the nth_drone's POV ##############
    #### - seg ((h,w) array)                segmentation image captured from the nth_drone's POV #######
    ####################################################################################################
    def _getDroneImages(self, nth_drone, segmentation: bool=True):
        if self.IMG_RES is None: print("[ERROR] in BaseAviary._getDroneImages(), remember to set self.IMG_RES to np.array([width, height])"); exit()
        rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[nth_drone,:])).reshape(3,3)
        #### Set target point and camera view and projection matrices ######################################
        target = np.dot(rot_mat,np.array([1000,0,0])) + np.array(self.pos[nth_drone,:])
        DRONE_CAM_VIEW = p.computeViewMatrix(cameraEyePosition=self.pos[nth_drone,:]+np.array([0,0,self.L]), cameraTargetPosition=target, cameraUpVector=[0,0,1], physicsClientId=self.CLIENT)
        DRONE_CAM_PRO =  p.computeProjectionMatrixFOV(fov=60.0, aspect=1.0, nearVal=self.L, farVal=1000.0)
        SEG_FLAG = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX if segmentation else p.ER_NO_SEGMENTATION_MASK
        [w, h, rgb, dep, seg] = p.getCameraImage(width=self.IMG_RES[0], height=self.IMG_RES[1], shadow=1, viewMatrix=DRONE_CAM_VIEW, projectionMatrix=DRONE_CAM_PRO,
                flags=SEG_FLAG, physicsClientId=self.CLIENT)
        rgb = np.reshape(rgb, (h, w, 4)); dep = np.reshape(dep, (h, w)); seg = np.reshape(seg, (h, w))
        return rgb, dep, seg

    ####################################################################################################
    #### Save an image returned by _getDroneImages() as a PNG file #####################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - img_type (ImageType)             image type: RGBA, depth, segmentation, b&w (from RGB) ######
    #### - img_input ((h,w,?) array)        matrix with 1 (depth/seg) or 4 (RGBA) channels #############
    #### - path (str)                       where to save the images as PNGs ###########################
    #### - frame_num (int)                  number to append to the frame's filename ###################
    ####################################################################################################
    def _exportImage(self, img_type: ImageType, img_input, path: str, frame_num: int=0):
        if img_type==ImageType.RGB: (Image.fromarray(img_input.astype('uint8'), 'RGBA')).save(path+"frame_"+str(frame_num)+".png")
        elif img_type==ImageType.DEP: temp = ((img_input-np.min(img_input)) * 255 / (np.max(img_input)-np.min(img_input))).astype('uint8')
        elif img_type==ImageType.SEG: temp = ((img_input-np.min(img_input)) * 255 / (np.max(img_input)-np.min(img_input))).astype('uint8')
        elif img_type==ImageType.BW: temp = (np.sum(img_input[:,:,0:2], axis=2) / 3).astype('uint8')
        else: print("[ERROR] in BaseAviary._exportImage(), unknown ImageType"); exit()
        if img_type!=ImageType.RGB: (Image.fromarray(temp)).save(path+"frame_"+str(frame_num)+".png")

    ####################################################################################################
    #### Compute the adjacency matrix of a multi-drone system using NEIGHBOURHOOD_RADIUS ##################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - adj_mat ((NUM_DRONES,NUM_DRONES) array)    adj_mat[i,j]=1 if i,j are neighbors, 0 otherwise #
    ####################################################################################################
    def _getAdjacencyMatrix(self):
        adjacency_mat = np.identity(self.NUM_DRONES)
        for i in range(self.NUM_DRONES-1):
            for j in range(self.NUM_DRONES-i-1):
                if np.linalg.norm(self.pos[i,:]-self.pos[j+i+1,:])<self.NEIGHBOURHOOD_RADIUS: adjacency_mat[i,j+i+1] = adjacency_mat[j+i+1,i] = 1
        return adjacency_mat

    ####################################################################################################
    #### Base PyBullet physics implementation ##########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm ((4,1) array)                RPM values of the 4 motors #################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _physics(self, rpm, nth_drone):
        forces = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM
        z_torque = (-torques[0] + torques[1] - torques[2] + torques[3])
        for i in range(4): p.applyExternalForce(self.DRONE_IDS[nth_drone], i, forceObj=[0,0,forces[i]], posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        p.applyExternalTorque(self.DRONE_IDS[nth_drone], 4, torqueObj=[0,0,z_torque], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### PyBullet implementation of ground effect, from (Shi et al., 2019) #############################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm ((4,1) array)                RPM values of the 4 motors #################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _groundEffect(self, rpm, nth_drone):
        #### Kinematic information of all links (propellers and center of mass) ############################
        link_states = np.array(p.getLinkStates(self.DRONE_IDS[nth_drone], linkIndices=[0,1,2,3,4], computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=self.CLIENT))
        #### Simple, per-propeller ground effects ##########################################################
        prop_heights = np.array([link_states[0,0][2], link_states[1,0][2], link_states[2,0][2], link_states[3,0][2]])
        prop_heights = np.clip(prop_heights, self.GND_EFF_H_CLIP, np.inf)
        gnd_effects = np.array(rpm**2) * self.KF * self.GND_EFF_COEFF * (self.PROP_RADIUS/(4 * prop_heights))**2
        if np.abs(self.rpy[nth_drone,0])<np.pi/2 and np.abs(self.rpy[nth_drone,1])<np.pi/2:
            for i in range(4): p.applyExternalForce(self.DRONE_IDS[nth_drone], i, forceObj=[0,0,gnd_effects[i]], posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        #### TODO: a more realistic model would account for the drone's attitude and its z-axis velocity in the world frame

    ####################################################################################################
    #### PyBullet implementation of drag, from (Forster, 2015) #########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm ((4,1) array)                RPM values of the 4 motors #################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _drag(self, rpm, nth_drone):
        #### Rotation matrix of the base ###################################################################
        base_rot = np.array(p.getMatrixFromQuaternion(self.quat[nth_drone,:])).reshape(3,3)
        #### Simple draft model applied to the base/center of mass #########################################
        drag_factors = -1 * self.DRAG_COEFF * np.sum(np.array(2*np.pi*rpm/60))
        drag = np.dot(base_rot, drag_factors*np.array(self.vel[nth_drone,:]))
        p.applyExternalForce(self.DRONE_IDS[nth_drone], 4, forceObj=drag, posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### PyBullet implementation of ground effect, SiQi Zhou's modelling ###############################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _downwash(self, nth_drone):
        for i in range(self.NUM_DRONES):
            delta_z = self.pos[i,2]-self.pos[nth_drone,2]; delta_xy = np.linalg.norm(np.array(self.pos[i,0:2])-np.array(self.pos[nth_drone,0:2]))
            if delta_z>0 and delta_xy<10: # Ignore drones more than 10 meters away
                alpha = self.DW_COEFF_1 * (self.PROP_RADIUS/(4*delta_z))**2; beta = self.DW_COEFF_2 * delta_z + self.DW_COEFF_3
                downwash = [0, 0,  - alpha * np.exp(-.5*(delta_xy/beta)**2)]
                p.applyExternalForce(self.DRONE_IDS[nth_drone], 4, forceObj=downwash, posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Explicit dynamics implementation from github.com/utiasDSL/dsl__projects__benchmark ############
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm ((4,1) array)                RPM values of the 4 motors #################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _dynamics(self, rpm, nth_drone):
        #### Current state #################################################################################
        pos = self.pos[nth_drone,:]; quat = self.quat[nth_drone,:]; rpy = self.rpy[nth_drone,:]
        vel = self.vel[nth_drone,:]; ang_v = self.ang_v[nth_drone,:]
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        #### Compute forces and torques ####################################################################
        forces = np.array(rpm**2) * self.KF
        thrust = np.array([0, 0, np.sum(forces)])
        thrust_world_frame = np.dot(rotation,thrust)
        force_world_frame = thrust_world_frame - np.array([0, 0, self.GRAVITY])
        z_torques = np.array(rpm**2)*self.KM
        z_torque = (-z_torques[0] + z_torques[1] - z_torques[2] + z_torques[3])
        if self.DRONE_MODEL==DroneModel.CF2X:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3]) * (self.L/np.sqrt(2))
            y_torque = (- forces[0] + forces[1] + forces[2] - forces[3]) * (self.L/np.sqrt(2))
        elif self.DRONE_MODEL==DroneModel.CF2P or self.DRONE_MODEL==DroneModel.HB:
            x_torque = (forces[1] - forces[3]) * self.L
            y_torque = (-forces[0] + forces[2]) * self.L
        torques = np.array([x_torque, y_torque, z_torque])
        torques = torques - np.cross(ang_v, np.dot(self.J, ang_v))
        ang_vel_deriv = np.dot(self.J_INV, torques)
        self.no_pybullet_dyn_accs[nth_drone] = force_world_frame / self.M
        #### Update state ##################################################################################
        vel = vel + self.TIMESTEP * self.no_pybullet_dyn_accs[nth_drone]
        ang_v = ang_v + self.TIMESTEP * ang_vel_deriv
        pos = pos + self.TIMESTEP * vel
        rpy = rpy + self.TIMESTEP * ang_v
        #### Set PyBullet's state ##########################################################################
        p.resetBasePositionAndOrientation(self.DRONE_IDS[nth_drone], pos, p.getQuaternionFromEuler(rpy), physicsClientId=self.CLIENT)
        p.resetBaseVelocity(self.DRONE_IDS[nth_drone], vel, ang_v, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Denormalize the [-1,1] range to the [0, MAX RPM] range ########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action ((4,1) array)             normalized [-1,1] actions applied to the 4 motors ##########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _normalizedActionToRPM(self, action):
        if np.any(np.abs(action))>1: print("\n[ERROR] it", self.step_counter, "in BaseAviary._normalizedActionToRPM(), out-of-bound action")
        return np.where(action <= 0, (action+1)*self.HOVER_RPM, action*self.MAX_RPM) # Non-linear mapping: -1 -> 0, 0 -> HOVER_RPM, 1 -> MAX_RPM

    ####################################################################################################
    #### Save an action into self.last_action disambiguating between array and dict inputs #############
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action ((4,1) array or dict)     an array or a dict of arrays to be stored in last_action ###
    ####################################################################################################
    def _saveLastAction(self, action):
        if isinstance(action, collections.Mapping):
            for k, v in action.items(): self.last_action[int(k),:] = v
        else: self.last_action = np.reshape(action, (self.NUM_DRONES, 4))

    ####################################################################################################
    #### Draw the local frame of the nth drone #########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _showDroneLocalAxes(self, nth_drone):
        if self.GUI:
            AXIS_LENGTH = 2*self.L
            self.X_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[AXIS_LENGTH,0,0], lineColorRGB=[1,0,0], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.X_AX[nth_drone]), physicsClientId=self.CLIENT)
            self.Y_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,AXIS_LENGTH,0], lineColorRGB=[0,1,0], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.Y_AX[nth_drone]), physicsClientId=self.CLIENT)
            self.Z_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,0,AXIS_LENGTH], lineColorRGB=[0,0,1], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.Z_AX[nth_drone]), physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Add obstacles to the environment from .urdf files #############################################
    ####################################################################################################
    def _addObstacles(self):
        p.loadURDF("samurai.urdf", physicsClientId=self.CLIENT)
        p.loadURDF("duck_vhacd.urdf", [-.5,-.5,.05], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        p.loadURDF("cube_no_rotation.urdf", [-.5,-2.5,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        p.loadURDF("sphere2.urdf", [0,2,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Load parameters from the .urdf file ###########################################################
    ####################################################################################################
    def _parseURDFParameters(self):
        URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__))+"/../assets/"+self.URDF).getroot()
        M = float(URDF_TREE[1][0][1].attrib['value']); L = float(URDF_TREE[0].attrib['arm']); THRUST2WEIGHT_RATIO = float(URDF_TREE[0].attrib['thrust2weight'])
        IXX = float(URDF_TREE[1][0][2].attrib['ixx']); IYY = float(URDF_TREE[1][0][2].attrib['iyy']); IZZ = float(URDF_TREE[1][0][2].attrib['izz'])
        J = np.diag([IXX, IYY, IZZ]); J_INV = np.linalg.inv(J); KF = float(URDF_TREE[0].attrib['kf']); KM = float(URDF_TREE[0].attrib['km'])
        COLLISION_H = float(URDF_TREE[1][2][1][0].attrib['length']); COLLISION_R = float(URDF_TREE[1][2][1][0].attrib['radius'])
        COLLISION_SHAPE_OFFSETS = [float(s) for s in URDF_TREE[1][2][0].attrib['xyz'].split(' ')]; COLLISION_Z_OFFSET = COLLISION_SHAPE_OFFSETS[2]
        MAX_SPEED_KMH = float(URDF_TREE[0].attrib['max_speed_kmh']); GND_EFF_COEFF = float(URDF_TREE[0].attrib['gnd_eff_coeff']); PROP_RADIUS = float(URDF_TREE[0].attrib['prop_radius'])
        DRAG_COEFF_XY = float(URDF_TREE[0].attrib['drag_coeff_xy']); DRAG_COEFF_Z = float(URDF_TREE[0].attrib['drag_coeff_z']); DRAG_COEFF = np.array([DRAG_COEFF_XY, DRAG_COEFF_XY, DRAG_COEFF_Z])
        DW_COEFF_1 = float(URDF_TREE[0].attrib['dw_coeff_1']); DW_COEFF_2 = float(URDF_TREE[0].attrib['dw_coeff_2']); DW_COEFF_3 = float(URDF_TREE[0].attrib['dw_coeff_3'])
        return M, L, THRUST2WEIGHT_RATIO, J, J_INV, KF, KM, COLLISION_H, COLLISION_R, COLLISION_Z_OFFSET, MAX_SPEED_KMH, GND_EFF_COEFF, PROP_RADIUS, DRAG_COEFF, DW_COEFF_1, DW_COEFF_2, DW_COEFF_3

    ####################################################################################################
    #### Return the action space of the environment, to be implemented in a child class ################
    ####################################################################################################
    def _actionSpace(self):
        raise NotImplementedError

    ####################################################################################################
    #### Return the observation space of the environment, to be implemented in a child class ###########
    ####################################################################################################
    def _observationSpace(self):
        raise NotImplementedError

    ####################################################################################################
    #### Return the current observation of the environment, to be implemented in a child class #########
    ####################################################################################################
    def _computeObs(self):
        raise NotImplementedError

    ####################################################################################################
    #### Preprocess the action passed to .step() into motors' RPMs, to be implemented in a child class #
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (..)                      input to the environment's .step() function ################
    ####################################################################################################
    def _preprocessAction(self, action):
        raise NotImplementedError

    ####################################################################################################
    #### Compute the current reward value(s), to be implemented in a child class #######################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    def _computeReward(self, obs):
        raise NotImplementedError

    ####################################################################################################
    #### Compute the current done value(s), to be implemented in a child class #########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    def _computeDone(self, obs):
        raise NotImplementedError

    ####################################################################################################
    #### Compute the current info dict(s), to be implemented in a child class ##########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - obs (..)                         the return of _computeObs() ################################
    ####################################################################################################
    def _computeInfo(self, obs):
        raise NotImplementedError





