import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import pybullet_data
import gym
import xml.etree.ElementTree as etxml
from gym import error, spaces, utils
from gym.utils import seeding
from enum import Enum
from datetime import datetime

from gym_pybullet_drones.envs.SingleDroneUserDefinedFunctions import SingleDroneUserDefinedFunctions


######################################################################################################################################################
#### Drone models enumeration ########################################################################################################################
######################################################################################################################################################
class DroneModel(Enum):
    HB = 0                   # Generic quadrotor (with AscTec Hummingbird intertial properties)
    CF2X = 1                 # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = 2                 # Bitcraze Craziflie 2.0 in the + configuration


######################################################################################################################################################
#### Single drone environment class ##################################################################################################################
######################################################################################################################################################
class SingleDroneEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    ####################################################################################################
    #### Initialize the environment ####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - drone_model (DroneModel)         the type of drone to use (associated to an .urdf file) #####
    #### - initial_xyz (3-by-1 list)        initial XYZ position of the drone ##########################
    #### - initial_rpy (3-by-1 list)        initial roll, pitch, and yaw of the drone (radians) ########
    #### - pybullet (Boolean)               whether to use PyBullet's physics engine ###################
    #### - aero_effects (Boolean)           simple drag and ground effect in PyBullet ##################
    #### - normalized_spaces (Boolean)      whether to use normalized OpenAI Gym spaces ################
    #### - freq (Integer)                   the freqeuency (Hz) at which the simulation steps ##########
    #### - gui (Boolean)                    whether to use PyBullet's GUI ##############################
    #### - obstacles (Boolean)              whether to add obstacles in the simulation #################
    #### - record (Boolean)                 whether to save the simulation as an .mp4 video ############
    #### - user (String)                    passed to __init__() of SingleDroneUserDefinedFunctions ####
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, initial_xyz=[0.,0.,.1], initial_rpy=[0.,0.,0.], pybullet=True, aero_effects=False, normalized_spaces=True, freq=240, gui=False, obstacles=False, record=False, user="Default"):
        super(SingleDroneEnv, self).__init__()
        self.G = 9.8; self.RAD2DEG = 180/np.pi; self.DEG2RAD = np.pi/180
        self.DRONE_MODEL = drone_model; self.INIT_XYZ = initial_xyz; self.INIT_RPY = initial_rpy 
        self.PYBULLET = pybullet; self.AERO_EFFECTS = aero_effects; self.NORM_SPACES = normalized_spaces
        self.SIM_FREQ = freq; self.TIMESTEP = 1./self.SIM_FREQ; self.GUI=gui; self.OBSTACLES = obstacles; self.RECORD = record; self.USER = user
        if self.DRONE_MODEL == DroneModel.HB: self.URDF = "hb.urdf"
        elif self.DRONE_MODEL == DroneModel.CF2X: self.URDF = "cf2x.urdf"
        elif self.DRONE_MODEL == DroneModel.CF2P: self.URDF = "cf2p.urdf"
        if self.AERO_EFFECTS and not self.PYBULLET: print("[WARNING] SingleDroneEnv was initalized with pybullet=False, aerodynamic effects will not be computed")
        ####################################################################################################
        #### Connect to PyBullet ###########################################################################
        ####################################################################################################
        if self.GUI: 
            self.CLIENT = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0, physicsClientId=self.CLIENT)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0, physicsClientId=self.CLIENT)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0, physicsClientId=self.CLIENT)
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0,0,0], physicsClientId=self.CLIENT)
        else: self.CLIENT = p.connect(p.DIRECT)        
        ####################################################################################################
        #### Load the drone properties from the .urdf file #################################################
        ####################################################################################################
        URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__))+"/../assets/"+self.URDF).getroot()
        self.M = float(URDF_TREE[1][0][1].attrib['value']); self.L = float(URDF_TREE[0].attrib['arm']); self.THRUST2WEIGHT_RATIO = float(URDF_TREE[0].attrib['thrust2weight'])
        self.IXX = float(URDF_TREE[1][0][2].attrib['ixx']); self.IYY = float(URDF_TREE[1][0][2].attrib['iyy']); self.IZZ = float(URDF_TREE[1][0][2].attrib['izz'])
        self.KF = float(URDF_TREE[0].attrib['kf']); self.KM = float(URDF_TREE[0].attrib['km'])
        self.COLLISION_H = float(URDF_TREE[1][2][1][0].attrib['length']); self.COLLISION_R = float(URDF_TREE[1][2][1][0].attrib['radius'])
        COLLISION_SHAPE_OFFSETS = [float(s) for s in URDF_TREE[1][2][0].attrib['xyz'].split(' ')]; self.COLLISION_Z_OFFSET = COLLISION_SHAPE_OFFSETS[2]
        self.GND_EFF_COEFF = float(URDF_TREE[0].attrib['gnd_eff_coeff']); self.PROP_RADIUS = float(URDF_TREE[0].attrib['prop_radius'])
        self.DRAG_COEFF_XY = float(URDF_TREE[0].attrib['drag_coeff_xy']); self.DRAG_COEFF_Z = float(URDF_TREE[0].attrib['drag_coeff_z'])
        self.DRAG_COEFF = np.array([self.DRAG_COEFF_XY, self.DRAG_COEFF_XY, self.DRAG_COEFF_Z])
        if not self.PYBULLET: self.J = np.diag([self.IXX, self.IYY, self.IZZ]); self.J_INV = np.linalg.inv(self.J)
        print("[INFO] SingleDroneEnv.__init__() loaded from the drone's .urdf file the following physical parameters (m, L, ixx, iyy, izz, kf, km, t2w):", self.M, self.L, self.IXX, self.IYY, self.IZZ, self.KF, self.KM, self.THRUST2WEIGHT_RATIO)
        if self.AERO_EFFECTS: print("[INFO] SingleDroneEnv.__init__() loaded from the drone's .urdf file the following aerodynamic parameters (gnd_eff_coeff, prop_radius, drag_xy_coeff, drag_z_coeff):", self.GND_EFF_COEFF, self.PROP_RADIUS, self.DRAG_COEFF_XY, self.DRAG_COEFF_Z)
        ####################################################################################################
        #### Compute constants #############################################################################
        ####################################################################################################
        self.GRAVITY = self.G*self.M
        self.HOVER_RPM = np.sqrt((self.M*self.G)/(4*self.KF))
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY)/(4*self.KF))
        self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2)
        self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)   
        ####################################################################################################
        #### Create action and observation spaces ##########################################################
        ####################################################################################################
        if self.NORM_SPACES:
            self.action_space = spaces.Box(    low=np.array([-1,-1,-1,-1]), high=np.array([1,1,1,1]), dtype=np.float32)
            #### Observations ################################       X       Y       Z       Q1      Q2      Q3      Q4      R       P       Y       VX      VY      VZ      WR      WP      WY      P0      P1      P2      P3
            self.observation_space = spaces.Box(low=np.array([      -1,     -1,     0,      -1,     -1,     -1,     -1,     -1,     -1,     -1,     -1,      -1,    -1,     -1,     -1,     -1,     -1,     -1,     -1,     -1]), \
                                               high=np.array([       1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,       1,     1,      1,      1,      1,      1,      1,      1,      1]), dtype=np.float32)
        else:
            self.action_space = spaces.Box(    low=np.array([0.,0.,0.,0.]), high=np.array([self.MAX_RPM,self.MAX_RPM,self.MAX_RPM,self.MAX_RPM]), dtype=np.float32)
            #### Observations ################################       X       Y       Z       Q1      Q2      Q3      Q4      R       P       Y       VX      VY      VZ      WR      WP      WY          P0              P1              P2              P3
            self.observation_space = spaces.Box(low=np.array([      -np.inf,-np.inf,0.,     -1.,    -1.,    -1.,    -1.,    -np.pi, -np.pi, -np.pi, -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,    0.,             0.,             0.,             0.]), \
                                              high=np.array([        np.inf, np.inf, np.inf, 1.,     1.,     1.,     1.,     np.pi,  np.pi,  np.pi,  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf,    self.MAX_RPM,   self.MAX_RPM,   self.MAX_RPM,   self.MAX_RPM]), dtype=np.float32)
        ####################################################################################################
        #### Add input sliders to the GUI ##################################################################
        ####################################################################################################
        self.P0_SLIDER = p.addUserDebugParameter('Propeller 0 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P1_SLIDER = p.addUserDebugParameter('Propeller 1 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P2_SLIDER = p.addUserDebugParameter('Propeller 2 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P3_SLIDER = p.addUserDebugParameter('Propeller 3 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.INPUT_SWITCH = p.addUserDebugParameter('Use GUI RPM', 9999., -1., 0, physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Housekeeping ##################################################################################
        ####################################################################################################
        self._housekeeping()

    ####################################################################################################
    #### Reset the environment #########################################################################
    ####################################################################################################
    def reset(self):
        p.resetSimulation(physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Housekeeping ##################################################################################
        ####################################################################################################
        self._housekeeping()
        if self.RECORD: self.VIDEO_ID = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=os.path.dirname(os.path.abspath(__file__))+"/../../files/saves/video-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".mp4", physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Return the initial observation ################################################################
        ####################################################################################################
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        state = np.hstack([pos, quat, rpy, vel, ang_v, self.last_action])
        if self.NORM_SPACES: state = self.USER_DEFINED_FUNCTIONS.clipAndNormalizeState(state, self.step_counter)
        return state.reshape(20,)

    ####################################################################################################
    #### Advance the environment by one simulation step ################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (Gym's Box(4,))           motors' speed (normalized or unnormalized) #################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - state (Gym's Box(20,))           observation vector (normalized or unnormalized) ############
    #### - reward (Float)                   reward value of the current state ##########################
    #### - done (Boolean)                   whether the current episode is over/meets halt conditions ##
    #### - info (Dict)                      additional custom information ##############################
    ####################################################################################################
    def step(self, action):
        self.step_counter += 1
        ####################################################################################################
        #### Read the GUI's input parameters ###############################################################
        ####################################################################################################
        if self.GUI: 
            current_input_switch = p.readUserDebugParameter(self.INPUT_SWITCH, physicsClientId=self.CLIENT)
            if current_input_switch > self.last_input_switch:
                self.last_input_switch = current_input_switch
                self.USE_GUI_RPM = True if self.USE_GUI_RPM==False else False
        if self.USE_GUI_RPM:
            p0, p1 = p.readUserDebugParameter(self.P0_SLIDER, physicsClientId=self.CLIENT), p.readUserDebugParameter(self.P1_SLIDER, physicsClientId=self.CLIENT)
            p2, p3 = p.readUserDebugParameter(self.P2_SLIDER, physicsClientId=self.CLIENT), p.readUserDebugParameter(self.P3_SLIDER, physicsClientId=self.CLIENT)
            clipped_rpm = np.array([p0,p1,p2,p3])
            if self.step_counter%(self.SIM_FREQ/2)==0:
                self.GUI_INPUT_TEXT = p.addUserDebugText("Using GUI RPM",textPosition=[0,0,0],textColorRGB=[1,0,0],lifeTime=1,textSize=2,parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.GUI_INPUT_TEXT,physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Denormalize (if necessary) and clip the action to the maximum RPM #############################
        ####################################################################################################
        else:
            rpm = self._normActionToRPM(action) if self.NORM_SPACES else np.array(action)
            clipped_rpm = np.clip(np.array(rpm), 0, self.MAX_RPM)
            if self.GUI and not(clipped_rpm==np.array(rpm)).all():
                print("\n[WARNING] it:", self.step_counter, "in step(), out-of-range rotor speeds [{:.0f} {:.0f} {:.0f} {:.0f}], max RPM: {:.0f}".format(rpm[0], rpm[1], rpm[2], rpm[3], self.MAX_RPM)) 
        self.last_action = np.array(action)
        ####################################################################################################
        #### Step the simulation using PyBullet's physics engine ###########################################
        ####################################################################################################
        if self.PYBULLET:
            self._physics(clipped_rpm)
            if self.AERO_EFFECTS: self._simpleAerodynamicEffects(clipped_rpm)
            p.stepSimulation(physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Step the simulation with a custom dynamics implementation #####################################
        ####################################################################################################
        else: self._noPyBulletDynamics(clipped_rpm)
        ####################################################################################################
        #### Prepare the return values #####################################################################
        ####################################################################################################
        self._showDroneFrame()
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        state = np.hstack([pos, quat, rpy, vel, ang_v, self.last_action])
        if self.NORM_SPACES: state = self.USER_DEFINED_FUNCTIONS.clipAndNormalizeState(state, self.step_counter)
        reward = self.USER_DEFINED_FUNCTIONS.rewardFunction(state)
        done = self.USER_DEFINED_FUNCTIONS.doneFunction(state, self.step_counter/self.SIM_FREQ) 
        info = {"answer": 42}
        return state.reshape(20,), reward, done, info

    ####################################################################################################
    #### Print a textual output of the environment #####################################################
    ####################################################################################################
    def render(self, mode='human', close=False):
        if self.first_render_call and not self.GUI: 
            self.first_render_call = False
            print("[WARNING] render() is implemented as text-only, re-initialize the environment using singleDroneEnv(gui=True) to use PyBullet's graphical interface")
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        print("——— step number {:d}".format(self.step_counter), 
            "——— wall-clock time {:.1f}".format(time.time()-self.RESET_TIME), 
            "——— simulation time {:.1f}@{:d}Hz ({:.2f}x)".format(self.step_counter*self.TIMESTEP, self.SIM_FREQ, (self.step_counter*self.TIMESTEP)/(time.time()-self.RESET_TIME)), 
            "——— X {:.2f} Y {:.2f} Z {:.2f} vel. {:.2f}".format(pos[0],pos[1],pos[2], np.linalg.norm(vel)), 
            "——— roll {:.2f} pitch {:.2f} yaw {:.2f}".format(rpy[0]*self.RAD2DEG, rpy[1]*self.RAD2DEG, rpy[2]*self.RAD2DEG),
            "——— ang. vel. {:.2f} {:.2f} {:.2f} ——— ".format(ang_v[0]*self.RAD2DEG, ang_v[1]*self.RAD2DEG, ang_v[2]*self.RAD2DEG), end="\r")

    ####################################################################################################
    #### Close the environment #########################################################################
    ####################################################################################################
    def close(self):
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        print("——— step number {:d}".format(self.step_counter), 
            "——— wall-clock time {:.1f}".format(time.time()-self.RESET_TIME), 
            "——— simulation time {:.1f}@{:d}Hz ({:.2f}x)".format(self.step_counter*self.TIMESTEP, self.SIM_FREQ, (self.step_counter*self.TIMESTEP)/(time.time()-self.RESET_TIME)), 
            "——— X {:.2f} Y {:.2f} Z {:.2f} vel. {:.2f}".format(pos[0],pos[1],pos[2], np.linalg.norm(vel)), 
            "——— roll {:.2f} pitch {:.2f} yaw {:.2f}".format(rpy[0]*self.RAD2DEG, rpy[1]*self.RAD2DEG, rpy[2]*self.RAD2DEG),
            "——— ang. vel. {:.2f} {:.2f} {:.2f} ——— ".format(ang_v[0]*self.RAD2DEG, ang_v[1]*self.RAD2DEG, ang_v[2]*self.RAD2DEG))
        if self.RECORD: p.stopStateLogging(self.VIDEO_ID, physicsClientId=self.CLIENT)
        p.disconnect(physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Return the PyBullet Client Id #################################################################
    ####################################################################################################
    def getPyBulletClient(self):
        return self.CLIENT

    ####################################################################################################
    #### Return the Drone Id ###########################################################################
    ####################################################################################################
    def getDroneId(self):
        return self.DRONE_ID


######################################################################################################################################################
#### Internals #######################################################################################################################################
######################################################################################################################################################

    ####################################################################################################
    #### Denormalize the [-1,1] range to the [0, MAX RPM] range ########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (4-by-1 array)            normalized [-1,1] actions applied to the 4 motors ##########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4-by-1 array)               RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _normActionToRPM(self, action): 
        if np.any(np.abs(action)) > 1: print("\n[ERROR] it:", self.step_counter, "in _normActionToRPM(), out-of-bound action")
        return np.where(action <= 0, (action+1)*self.HOVER_RPM, action*self.MAX_RPM)

    ####################################################################################################
    #### Normalize the [0, MAX RPM] range to the [-1,1] range ##########################################
    ####################################################################################################    
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - action (4-by-1 array)            normalized action to apply to the 4 motors #################
    ####################################################################################################
    # def _rpmToNormAction(self, rpm): 
    #     if np.any(rpm) < 0: print("\n[ERROR] it:", self.step_counter, "in _rpmToNormAction(), negative RPM")
    #     return np.where(rpm <= self.HOVER_RPM, (rpm/self.HOVER_RPM)-1, rpm/self.MAX_RPM)

    ####################################################################################################
    #### Housekeeping shared by the __init__() and reset() functions ###################################
    ####################################################################################################
    def _housekeeping(self):
        ####################################################################################################
        #### Initialize/reset counters and zero-valued variables ###########################################
        ####################################################################################################
        self.RESET_TIME = time.time(); self.step_counter = 0; self.first_render_call = True
        self.X_AX = -1; self.Y_AX = -1; self.Z_AX = -1; 
        self.GUI_INPUT_TEXT = -1; self.USE_GUI_RPM=False; self.last_input_switch = 0
        self.USER_DEFINED_FUNCTIONS = SingleDroneUserDefinedFunctions(self.CLIENT, self.GUI, self.USER)
        self.last_action = -1*np.ones(4) if self.NORM_SPACES else np.zeros(4)
        if not self.PYBULLET: self.no_pybullet_acc = np.zeros(3)
        ####################################################################################################
        #### Set PyBullet's parameters #####################################################################
        ####################################################################################################
        p.setGravity(0, 0, -self.G, physicsClientId=self.CLIENT)
        p.setRealTimeSimulation(0, physicsClientId=self.CLIENT)
        p.setTimeStep(self.TIMESTEP, physicsClientId=self.CLIENT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Load ground plane, drone and obstacles models #################################################
        ####################################################################################################
        p.loadURDF("plane.urdf", physicsClientId=self.CLIENT)
        self.DRONE_ID = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/"+self.URDF,self.INIT_XYZ, p.getQuaternionFromEuler(self.INIT_RPY), physicsClientId=self.CLIENT)
        if self.OBSTACLES:
            self.USER_DEFINED_FUNCTIONS.addObstacles()

    ####################################################################################################
    #### Draw the local frame of the drone #############################################################
    ####################################################################################################
    def _showDroneFrame(self):
        AXIS_LENGTH = 2*self.L
        self.X_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[AXIS_LENGTH,0,0],lineColorRGB=[1,0,0],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.X_AX,physicsClientId=self.CLIENT)
        self.Y_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,AXIS_LENGTH,0],lineColorRGB=[0,1,0],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.Y_AX,physicsClientId=self.CLIENT)
        self.Z_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,AXIS_LENGTH],lineColorRGB=[0,0,1],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.Z_AX,physicsClientId=self.CLIENT)
    
    ####################################################################################################
    #### PyBullet physics implementation ###############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _physics(self, rpm):
        forces = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM
        z_torque = (-torques[0] + torques[1] - torques[2] + torques[3])
        for i in range(4): p.applyExternalForce(self.DRONE_ID, i, forceObj=[0,0,forces[i]], posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        p.applyExternalTorque(self.DRONE_ID, 4, torqueObj=[0,0,z_torque], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### PyBullet implementation of drag and ground effect #############################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _simpleAerodynamicEffects(self, rpm):
        ####################################################################################################
        #### Kinematic information of the base and all links (propellers and center of mass) ###############
        ####################################################################################################
        base_pos, base_quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        base_rpy = p.getEulerFromQuaternion(base_quat)
        base_rot = np.array(p.getMatrixFromQuaternion(base_quat)).reshape(3,3)
        base_vel, base_ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        link_states = np.array(p.getLinkStates(self.DRONE_ID, linkIndices=[0,1,2,3,4], computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=self.CLIENT))
        ####################################################################################################
        #### Simple, per-propeller ground effects, from (Shi et al., 2019) #################################
        ####################################################################################################       
        prop_heights = np.array([link_states[0,0][2], link_states[1,0][2], link_states[2,0][2], link_states[3,0][2]])
        gnd_effects = np.array(rpm**2) * self.KF * self.GND_EFF_COEFF * (self.PROP_RADIUS/(4 * prop_heights))**2
        gnd_effects = np.clip(gnd_effects, 0, self.MAX_THRUST/10)
        if np.abs(base_rpy[0])<np.pi/2 and np.abs(base_rpy[1])<np.pi/2:
            for i in range(4): p.applyExternalForce(self.DRONE_ID, i, forceObj=[0,0,gnd_effects[i]], posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        #### TODO: a more realistic model would account for the drone's attitude and its z-axis velocity in the world frame 
        ####################################################################################################
        #### Simple draft model applied to the base/center of mass, from (Forster, 2015) ###################
        ####################################################################################################
        drag_factors = -1 * self.DRAG_COEFF * np.sum(np.array(2*np.pi*rpm/60))
        drag = np.dot(base_rot,drag_factors*np.array(base_vel))
        p.applyExternalForce(self.DRONE_ID, 4, forceObj=drag, posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)       

    ####################################################################################################
    #### Alternative PyBullet physics implementation ###################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    # def _physicsWithExplicitTorques(self, rpm):
    #     forces = np.array(rpm**2)*self.KF
    #     torques = np.array(rpm**2)*self.KM
    #     z_torque = (torques[0] - torques[1] + torques[2] - torques[3])
    #     thrust = np.array([0,0,np.sum(forces)])
    #     if self.DRONE_MODEL==DroneModel.HB or self.DRONE_MODEL==DroneModel.CF2P:
    #         x_torque = (forces[1] - forces[3])*self.L
    #         y_torque = (-forces[0] + forces[2])*self.L
    #     elif self.DRONE_MODEL==DroneModel.CF2X:
    #         x_torque = (forces[0] + forces[1] - forces[2] - forces[3])*self.L/np.sqrt(2)
    #         y_torque = (- forces[0] + forces[1] + forces[2] - forces[3])*self.L/np.sqrt(2)
    #     torques = np.array([x_torque,y_torque,z_torque])
    #     p.applyExternalForce(self.DRONE_ID, 4, forceObj=thrust, posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
    #     p.applyExternalTorque(self.DRONE_ID, 4, torqueObj=torques, flags=p.LINK_FRAME, physicsClientId=self.CLIENT)

    ####################################################################################################
    #### Custom dynamics implementation ################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _noPyBulletDynamics(self, rpm):
        ####################################################################################################
        #### Based on: github.com/utiasDSL/dsl__projects__benchmark/tree/gym-wrapper/python_sim ############
        ####################################################################################################
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        forces = np.array(rpm**2)*self.KF
        thrust = np.array([0, 0, np.sum(forces)])
        thrust_world_frame = np.dot(rotation,thrust)
        force_world_frame = thrust_world_frame - np.array([0, 0, self.GRAVITY])
        z_torques = np.array(rpm**2)*self.KM
        z_torque = (-z_torques[0] + z_torques[1] - z_torques[2] + z_torques[3])
        if self.DRONE_MODEL==DroneModel.HB or self.DRONE_MODEL==DroneModel.CF2P:
            x_torque = (forces[1] - forces[3])*self.L
            y_torque = (-forces[0] + forces[2])*self.L
        elif self.DRONE_MODEL==DroneModel.CF2X:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3])*self.L/np.sqrt(2)
            y_torque = (- forces[0] + forces[1] + forces[2] - forces[3])*self.L/np.sqrt(2)
        torques = np.array([x_torque, y_torque, z_torque])
        torques = torques - np.cross(ang_v, np.dot(self.J, ang_v))
        ang_vel_deriv = np.dot(self.J_INV, torques)
        self.no_pybullet_acc = force_world_frame / self.M
        vel = vel + self.TIMESTEP * self.no_pybullet_acc
        ang_v = ang_v + self.TIMESTEP * ang_vel_deriv
        pos = pos + self.TIMESTEP * vel
        rpy = rpy + self.TIMESTEP * ang_v
        p.resetBasePositionAndOrientation(self.DRONE_ID, pos, p.getQuaternionFromEuler(rpy), physicsClientId=self.CLIENT)
        p.resetBaseVelocity(self.DRONE_ID, vel, ang_v, physicsClientId=self.CLIENT)

        