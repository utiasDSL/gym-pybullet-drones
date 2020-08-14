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

from gym_pybullet_drones.envs.UserDefinedFunctions import UserDefinedFunctions






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
    #### - pybullet (Boolean)               whether to use PyBullet's physics engine ###################
    #### - aero_effects (Boolean)           simple drag and ground effect in PyBullet ##################
    #### - normalized_spaces (Boolean)      whether to use normalized OpenAI Gym spaces ################
    #### - freq (Integer)                   the freqeuency (Hz) at which the simulation steps ##########
    #### - gui (Boolean)                    whether to use PyBullet's GUI ##############################
    #### - obstacles (Boolean)              whether to add obstacles in the simulation #################
    #### - record (Boolean)                 whether to save the simulation as an .mp4 video ############
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, pybullet=True, aero_effects=False, normalized_spaces=True, freq=240, gui=False, obstacles=False, record=False):
        super(SingleDroneEnv, self).__init__()
        self.G = 9.8; self.RAD2DEG = 180/np.pi; self.DEG2RAD = np.pi/180
        self.DRONE_MODEL = drone_model; self.PYBULLET = pybullet; self.AERO_EFFECTS = aero_effects; self.NORM_SPACES = normalized_spaces
        self.SIM_FREQ = freq; self.TIMESTEP = 1./self.SIM_FREQ; self.GUI=gui; self.OBSTACLES = obstacles; self.RECORD = record
        if self.AERO_EFFECTS and not self.PYBULLET: print("[WARNING] aerodynamic effect will not be computed because SingleDroneEnv was initalized with pybullet=False")
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
        if self.DRONE_MODEL == DroneModel.HB: URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__))+"/../assets/hb.urdf").getroot()
        elif self.DRONE_MODEL == DroneModel.CF2X: URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__))+"/../assets/cf2x.urdf").getroot()
        elif self.DRONE_MODEL == DroneModel.CF2P: URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__))+"/../assets/cf2+.urdf").getroot()
        self.M = float(URDF_TREE[1][0][1].attrib['value']); self.L = float(URDF_TREE[0].attrib['arm']); self.THRUST2WEIGHT_RATIO = float(URDF_TREE[0].attrib['thrust2weight'])
        self.IXX = float(URDF_TREE[1][0][2].attrib['ixx']); self.IYY = float(URDF_TREE[1][0][2].attrib['iyy']); self.IZZ = float(URDF_TREE[1][0][2].attrib['izz'])
        self.KF = float(URDF_TREE[0].attrib['kf']); self.KM = float(URDF_TREE[0].attrib['km'])
        if not self.PYBULLET: self.J = np.diag([self.IXX, self.IYY, self.IZZ]); self.J_INV = np.linalg.inv(self.J)
        print("[INFO] SingleDroneEnv.__init__() loaded from the drone's .urdf file the following physical parameters (m, L, ixx, iyy, izz, kf, km, t2w):", self.M, self.L, self.IXX, self.IYY, self.IZZ, self.KF, self.KM, self.THRUST2WEIGHT_RATIO)
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
            #### Observations ################################    X        Y        Z        Q1        Q2        Q3        Q4        R        P        Y        VX        VY        VZ        WR        WP        WY        P1        P2        P3        P4
            self.observation_space = spaces.Box(low=np.array([    -1,        -1,        0,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1]), \
                             high=np.array([         1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1]), dtype=np.float32)
        else:
            self.action_space = spaces.Box(    low=np.array([0.,0.,0.,0.]), high=np.array([self.MAX_RPM,self.MAX_RPM,self.MAX_RPM,self.MAX_RPM]), dtype=np.float32)
            #### Observations ################################    X        Y        Z        Q1    Q2    Q3    Q4    R        P        Y        VX        VY        VZ        WR        WP        WY        P1        P2        P3        P4
            self.observation_space = spaces.Box(low=np.array([    -np.inf,    -np.inf,    0.,        -1.,    -1.,    -1.,    -1.,    -np.pi,        -np.pi,        -np.pi,        -np.inf,    -np.inf,    -np.inf,    -np.inf,    -np.inf,    -np.inf,    0.,        0.,        0.,        0.]), \
                                 high=np.array([     np.inf,    np.inf,        np.inf,        1.,    1.,    1.,    1.,    np.pi,        np.pi,        np.pi,        np.inf,        np.inf,        np.inf,        np.inf,        np.inf,        np.inf,        self.MAX_RPM,    self.MAX_RPM,    self.MAX_RPM,    self.MAX_RPM]), dtype=np.float32)
        ####################################################################################################
        #### Add input sliders to the GUI ##################################################################
        ####################################################################################################
        self.P1_SLIDER = p.addUserDebugParameter('Propeller 1 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P2_SLIDER = p.addUserDebugParameter('Propeller 2 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P3_SLIDER = p.addUserDebugParameter('Propeller 3 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.P4_SLIDER = p.addUserDebugParameter('Propeller 4 RPM', 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
        self.INPUT_TYPE = p.addUserDebugParameter('[0:1][1:13/24)[2:12/34][3:14/23][4:!]', 0, 4, 0, physicsClientId=self.CLIENT)
        self.INPUT_SWITCH = p.addUserDebugParameter('Use GUI RPM (set to > 0.5)', 0, 1, 0, physicsClientId=self.CLIENT)
        self.DISTURBANCE_SLIDER_1 = p.addUserDebugParameter('Push down', 0, 0.5, 0, physicsClientId=self.CLIENT)
        self.DISTURBANCE_SLIDER_2 = p.addUserDebugParameter('Slap left', 0, 0.5, 0, physicsClientId=self.CLIENT)
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
        if self.RECORD: self.VIDEO_ID = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=os.path.dirname(os.path.abspath(__file__))+"/../../video-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".mp4", physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Return the initial observation ################################################################
        ####################################################################################################
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        if self.PYBULLET: vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        else: vel, ang_v = self.no_pybullet_vel, self.no_pybullet_ang_vel
        if self.NORM_SPACES: state = self._clipAndNormalizeState(np.hstack([pos, quat, rpy, vel, ang_v, self.last_action]))
        else: state = np.hstack([pos, quat, rpy, vel, ang_v, self.last_action])
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
            input_switch_val = p.readUserDebugParameter(self.INPUT_SWITCH, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,-p.readUserDebugParameter(self.DISTURBANCE_SLIDER_1, physicsClientId=self.CLIENT)], posObj=[self.L,0.,0.], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,p.readUserDebugParameter(self.DISTURBANCE_SLIDER_2, physicsClientId=self.CLIENT),0], posObj=[self.L,0.,0.], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        if self.GUI and input_switch_val > 0.5:
            input_type_val = p.readUserDebugParameter(self.INPUT_TYPE, physicsClientId=self.CLIENT)
            p1, p2 = p.readUserDebugParameter(self.P1_SLIDER, physicsClientId=self.CLIENT), p.readUserDebugParameter(self.P2_SLIDER, physicsClientId=self.CLIENT)
            p3, p4 = p.readUserDebugParameter(self.P3_SLIDER, physicsClientId=self.CLIENT), p.readUserDebugParameter(self.P4_SLIDER, physicsClientId=self.CLIENT)
            if input_type_val < 1: clipped_rpm = np.array([p1,p1,p1,p1])
            elif input_type_val < 2: clipped_rpm = np.array([p1,p2,p1,p2])
            elif input_type_val < 3: clipped_rpm = np.array([p1,p1,p3,p3])
            elif input_type_val < 4: clipped_rpm = np.array([p1,p2,p2,p1])
            elif input_type_val < 5: clipped_rpm = np.array([p1,p2,p3,p4])
        ####################################################################################################
        #### Denormalize (if necessary) and clip the action to the maximum RPM #############################
        ####################################################################################################
        else:
            if self.NORM_SPACES: rpm = self._normActionToRPM(action)
            else: rpm = np.array(action)
            clipped_rpm = np.clip(np.array(rpm), 0, self.MAX_RPM)
            if self.GUI and not(clipped_rpm==np.array(rpm)).all():
                print("\n[WARNING] it:", self.step_counter, "in step(), out-of-range rotor speeds [{:.0f} {:.0f} {:.0f} {:.0f}], max RPM: {:.0f}".format(rpm[0], rpm[1], rpm[2], rpm[3], self.MAX_RPM)) 
        self.last_action = np.array(action)
        ####################################################################################################
        #### Step the simulation using PyBullet's physics engine ###########################################
        ####################################################################################################
        if self.PYBULLET:
            self._physics(clipped_rpm)
            if self.AERO_EFFECTS: self._simpleAerodynamicEffects()
            p.stepSimulation(physicsClientId=self.CLIENT)
            vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        ####################################################################################################
        #### Step the simulation with a custom dynamics implementation #####################################
        ####################################################################################################
        else:
            self._noPyBulletDynamics(clipped_rpm)
            vel, ang_v = self.no_pybullet_vel, self.no_pybullet_ang_vel 
        self._showFrame()
        ####################################################################################################
        #### Prepare the return values #####################################################################
        ####################################################################################################
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        if self.NORM_SPACES: state = self._clipAndNormalizeState(np.hstack([pos, quat, rpy, vel, ang_v, self.last_action]))
        else: state = np.hstack([pos, quat, rpy, vel, ang_v, self.last_action])
        reward = self._computeReward(state)
        done = self._isDone(state) 
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
        if self.PYBULLET: vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        else: vel, ang_v = self.no_pybullet_vel, self.no_pybullet_ang_vel
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
        if self.PYBULLET: vel, ang_v = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        else: vel, ang_v = self.no_pybullet_vel, self.no_pybullet_ang_vel
        print("——— step number {:d}".format(self.step_counter), 
            "——— wall-clock time {:.1f}".format(time.time()-self.RESET_TIME), 
            "——— simulation time {:.1f}@{:d}Hz ({:.2f}x)".format(self.step_counter*self.TIMESTEP, self.SIM_FREQ, (self.step_counter*self.TIMESTEP)/(time.time()-self.RESET_TIME)), 
            "——— X {:.2f} Y {:.2f} Z {:.2f} vel. {:.2f}".format(pos[0],pos[1],pos[2], np.linalg.norm(vel)), 
            "——— roll {:.2f} pitch {:.2f} yaw {:.2f}".format(rpy[0]*self.RAD2DEG, rpy[1]*self.RAD2DEG, rpy[2]*self.RAD2DEG),
            "——— ang. vel. {:.2f} {:.2f} {:.2f} ——— ".format(ang_v[0]*self.RAD2DEG, ang_v[1]*self.RAD2DEG, ang_v[2]*self.RAD2DEG))
        if self.RECORD: p.stopStateLogging(self.VIDEO_ID, physicsClientId=self.CLIENT)
        p.disconnect(physicsClientId=self.CLIENT)






######################################################################################################################################################
#### Internals #######################################################################################################################################
######################################################################################################################################################


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

    ####################################################################################################
    #### Normalize the [0, MAX RPM] range to the [-1,1] range ##########################################
    ####################################################################################################    
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - action (4-by-1 array)            normalized action to apply to the 4 motors #################
    ####################################################################################################
    def _rpmToNormAction(self, rpm): ############################################# Currently unused ####
        if np.any(rpm) < 0: print("\n[ERROR] it:", self.step_counter, "in _rpmToNormAction(), negative RPM")
        return np.where(rpm <= self.HOVER_RPM, (rpm/self.HOVER_RPM)-1, rpm/self.MAX_RPM)

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
    #### Normalize the 20 values in the simulation state to the [-1,1] range ###########################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - state (20-by-1 array)            raw simulation state #######################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    ####################################################################################################
    def _clipAndNormalizeState(self, state):
        clipped_pos = np.clip(state[0:3], -1, 1)
        clipped_rp = np.clip(state[7:9], -np.pi/3, np.pi/3)
        clipped_vel = np.clip(state[10:13], -1, 1)
        clipped_ang_vel_rp = np.clip(state[13:15], -10*np.pi, 10*np.pi)
        clipped_ang_vel_y = np.clip(state[15], -20*np.pi, 20*np.pi)
        if self.GUI:
            if not(clipped_pos==np.array(state[0:3])).all(): print("[WARNING] it:", self.step_counter, "in _clipAndNormalizeState(), out-of-bound position [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of _isDone()".format(state[0], state[1], state[2]))
            if not(clipped_rp==np.array(state[7:9])).all(): print("[WARNING] it:", self.step_counter, "in _clipAndNormalizeState(), out-of-bound roll/pitch [{:.2f} {:.2f}], consider a more conservative implementation of _isDone()".format(state[7], state[8]))
            if not(clipped_vel==np.array(state[10:13])).all(): print("[WARNING] it:", self.step_counter, "in _clipAndNormalizeState(), out-of-bound velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of _isDone()".format(state[10], state[11], state[12]))
            if not(clipped_ang_vel_rp==np.array(state[13:15])).all(): print("[WARNING] it:", self.step_counter, "in _clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of _isDone()".format(state[13], state[14], state[15]))
            if not(clipped_ang_vel_y==np.array(state[15])): print("[WARNING] it:", self.step_counter, "in _clipAndNormalizeState(), out-of-bound angular velocity [{:.2f} {:.2f} {:.2f}], consider a more conservative implementation of _isDone()".format(state[13], state[14], state[15]))
        normalized_pos = clipped_pos
        normalized_rp = clipped_rp/(np.pi/3)
        normalized_y = state[9]/np.pi
        normalized_vel = clipped_vel
        normalized_ang_vel_rp = clipped_ang_vel_rp/(10*np.pi)
        normalized_ang_vel_y = clipped_ang_vel_y/(20*np.pi)
        return np.hstack([normalized_pos, state[3:7], normalized_rp, normalized_y, normalized_vel, normalized_ang_vel_rp, normalized_ang_vel_y, self.last_action]).reshape(20,)

    ####################################################################################################
    #### Compute the current state's reward ############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - reward (Float)                   reward value ###############################################
    ####################################################################################################
    def _computeReward(self, state):
        ####################################################################################################
        #### Customize the reward function #################################################################
        ####################################################################################################
        if state[2] > 0.8: reward = -1
        elif state[2] > 0.5: reward = 2000
        elif state[2] > 0.3: reward = 1000
        elif state[2] > 0.2: reward = 500
        elif state[2] > 0.15: reward = 100
        elif state[2] > 0.1: reward = 10
        else: reward = -1
        ####################################################################################################
        ####################################################################################################
        return reward

    ####################################################################################################
    #### Evaluate the current state's halting conditions ###############################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - norm. state (20-by-1 array)      clipped and normalized simulation state ####################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - done (Boolean)                   whether the halting conditions of the episode are met ######
    ####################################################################################################
    def _isDone(self, state):
        ####################################################################################################
        #### Customize the episodes' halting conditions ####################################################
        ####################################################################################################
        if np.abs(state[0])>=1 or np.abs(state[1])>=1 or state[2]>=1 \
                    or np.abs(state[7])>=np.pi/3 or np.abs(state[8])>=np.pi/3 \
                    or np.abs(state[10])>=1 or np.abs(state[11])>=1 or np.abs(state[12])>=1 \
                    or np.abs(state[13])>=10*np.pi or np.abs(state[14])>=10*np.pi or np.abs(state[15])>=20*np.pi \
                    or self.step_counter > 3*self.SIM_FREQ: 
            done = True
        else: 
            done = False
        ####################################################################################################
        ####################################################################################################
        return done

    ####################################################################################################
    #### Housekeeping shared by the __init__() and reset() functions ###################################
    ####################################################################################################
    def _housekeeping(self):
        ####################################################################################################
        #### Initialize/reset counters and zero-valued variables ###########################################
        ####################################################################################################
        self.step_counter = 0; self.RESET_TIME = time.time(); self.first_render_call = True
        self.X_AX = -1; self.Y_AX = -1; self.Z_AX = -1;
        if self.NORM_SPACES: self.last_action = -1*np.ones(4)
        else: self.last_action = np.zeros(4)
        if not self.PYBULLET: 
            self.no_pybullet_vel = np.zeros(3); self.no_pybullet_ang_vel = np.zeros(3); self.no_pybullet_acc = np.zeros(3)
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
        if self.DRONE_MODEL==DroneModel.HB:
            self.DRONE_ID = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/hb.urdf",[0,0,0.25], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        elif self.DRONE_MODEL==DroneModel.CF2X:
            self.DRONE_ID = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/cf2x.urdf",[0,0,0.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        elif self.DRONE_MODEL==DroneModel.CF2P:
            self.DRONE_ID = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/cf2+.urdf",[0,0,0.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
        if self.OBSTACLES:
            p.loadURDF("samurai.urdf", physicsClientId=self.CLIENT)
            p.loadURDF("duck_vhacd.urdf", [-.5,-.5,.05], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("cube_no_rotation.urdf", [-.5,-2.5,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)
            p.loadURDF("sphere2.urdf", [0,2,.5], p.getQuaternionFromEuler([0,0,0]), physicsClientId=self.CLIENT)

    ####################################################################################################
    #### PyBullet physics implementation ###############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _physics(self, rpm):
        forces = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM
        z_torque = (torques[0] - torques[1] + torques[2] - torques[3])
        if self.DRONE_MODEL==DroneModel.HB:
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[0]], posObj=[self.L,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[1]], posObj=[0,self.L,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[2]], posObj=[-self.L,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[3]], posObj=[0,-self.L,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        elif self.DRONE_MODEL==DroneModel.CF2X:
            dist = self.L/np.sqrt(2)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[0]], posObj=[dist,dist,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[1]], posObj=[-dist,dist,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[2]], posObj=[-dist,-dist,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[3]], posObj=[dist,-dist,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        elif self.DRONE_MODEL==DroneModel.CF2P:
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[0]], posObj=[self.L,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[1]], posObj=[0,self.L,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[2]], posObj=[-self.L,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
            p.applyExternalForce(self.DRONE_ID, -1, forceObj=[0,0,forces[3]], posObj=[0,-self.L,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        p.applyExternalTorque(self.DRONE_ID, -1, torqueObj=[0,0,z_torque], flags=p.WORLD_FRAME, physicsClientId=self.CLIENT) # Note: bug fix, WORLD_FRAME for LINK FRAME, see run_physics_test.py
   
    ####################################################################################################
    #### PyBullet implementation of drag and ground effect #############################################
    ####################################################################################################
    def _simpleAerodynamicEffects(self):
        pass

    ####################################################################################################
    #### Alternative PyBullet physics implementation ###################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _physicsWithExplicitTorques(self, rpm): ################################## Currently unused ####
        forces = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM
        z_torque = (torques[0] - torques[1] + torques[2] - torques[3])
        thrust = np.array([0,0,np.sum(forces)])
        if self.DRONE_MODEL==DroneModel.HB:
            x_torque = (forces[1] - forces[3])*self.L
            y_torque = (-forces[0] + forces[2])*self.L
        elif self.DRONE_MODEL==DroneModel.CF2X:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3])*self.L/np.sqrt(2)
            y_torque = (- forces[0] + forces[1] + forces[2] - forces[3])*self.L/np.sqrt(2)
        elif self.DRONE_MODEL==DroneModel.CF2P:
            x_torque = (forces[1] - forces[3])*self.L
            y_torque = (-forces[0] + forces[2])*self.L
        torques = np.array([x_torque,y_torque,z_torque])
        p.applyExternalForce(self.DRONE_ID, -1, forceObj=thrust, posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=self.CLIENT)
        p.applyExternalTorque(self.DRONE_ID, -1, torqueObj=torques, flags=p.WORLD_FRAME, physicsClientId=self.CLIENT) # Note: bug fix, WORLD_FRAME for LINK FRAME, see run_physics_test.py

    ####################################################################################################
    #### Custom dynamics implementation ################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm (4-by-1 array)               RPM values of the 4 motors #################################
    ####################################################################################################
    def _noPyBulletDynamics(self, rpm):
        ####################################################################################################
        #### Based on github.com/utiasDSL/dsl__projects__benchmark/tree/gym-wrapper/python_sim #############
        ####################################################################################################
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_ID, physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        forces = np.array(rpm**2)*self.KF
        thrust = np.array([0, 0, np.sum(forces)])
        thrust_world_frame = np.dot(rotation,thrust)
        force_world_frame = thrust_world_frame - np.array([0, 0, self.GRAVITY])
        z_torques = np.array(rpm**2)*self.KM
        z_torque = (z_torques[0] - z_torques[1] + z_torques[2] - z_torques[3])
        if self.DRONE_MODEL==DroneModel.HB:
            x_torque = (forces[1] - forces[3])*self.L
            y_torque = (-forces[0] + forces[2])*self.L
        elif self.DRONE_MODEL==DroneModel.CF2X:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3])*self.L/np.sqrt(2)
            y_torque = (- forces[0] + forces[1] + forces[2] - forces[3])*self.L/np.sqrt(2)
        elif self.DRONE_MODEL==DroneModel.CF2P:
            x_torque = (forces[1] - forces[3])*self.L
            y_torque = (-forces[0] + forces[2])*self.L
        torques = np.array([x_torque, y_torque, z_torque])
        torques = torques - np.cross(self.no_pybullet_ang_vel, np.dot(self.J, self.no_pybullet_ang_vel))
        ang_vel_deriv = np.dot(self.J_INV, torques)
        self.no_pybullet_acc = force_world_frame / self.M
        self.no_pybullet_vel = self.no_pybullet_vel + self.TIMESTEP * self.no_pybullet_acc
        self.no_pybullet_ang_vel = self.no_pybullet_ang_vel + self.TIMESTEP * ang_vel_deriv
        pos = pos + self.TIMESTEP * self.no_pybullet_vel
        rpy = rpy + self.TIMESTEP * self.no_pybullet_ang_vel
        p.resetBasePositionAndOrientation(self.DRONE_ID, pos, p.getQuaternionFromEuler(rpy))

    ####################################################################################################
    #### Draw the local frame of the drone #############################################################
    ####################################################################################################
    def _showFrame(self):
        if self.DRONE_MODEL==DroneModel.HB: LENGTH = 0.35
        elif self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P: LENGTH = 0.1
        self.X_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[LENGTH,0,0],lineColorRGB=[1,0,0],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.X_AX,physicsClientId=self.CLIENT)
        self.Y_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,LENGTH,0],lineColorRGB=[0,1,0],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.Y_AX,physicsClientId=self.CLIENT)
        self.Z_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,LENGTH],lineColorRGB=[0,0,1],parentObjectUniqueId=self.DRONE_ID,parentLinkIndex=-1,replaceItemUniqueId=self.Z_AX,physicsClientId=self.CLIENT)
        
