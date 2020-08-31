import os
import time
import curses
import pdb
import math
import numpy as np
import xml.etree.ElementTree as etxml
import pybullet as p
import pybullet_data
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from enum import Enum
from datetime import datetime

from gym_pybullet_drones.envs.RLFunctions import Problem, RLFunctions 


######################################################################################################################################################
#### Drone models enumeration ########################################################################################################################
######################################################################################################################################################
class DroneModel(Enum):
    CF2X = 0                 # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = 1                 # Bitcraze Craziflie 2.0 in the + configuration
    HB = 2                   # Generic quadrotor (with AscTec Hummingbird inertial properties)


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
    PYB_PM = 6               # Simpler PyBullet physics update with point-mass models
    PYB_KIN = 7              # Update with a desired kinematics input


######################################################################################################################################################
#### Multi-drone environment class ###################################################################################################################
######################################################################################################################################################
class Aviary(gym.Env):
    metadata = {'render.modes': ['human']}

    ####################################################################################################
    #### Initialize the environment ####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - drone_model (DroneModel)         desired drone type (associated to an .urdf file) ###########
    #### - num_drones (int)                 desired number of drones in the aviary #####################
    #### - visibility_radius (float)        used to compute the drones' adjacency matrix, in meters ####
    #### - initial_xyz ((3,1) array)        initial XYZ position of the drones #########################
    #### - initial_rpy ((3,1) array)        initial orientations of the drones (radians) ###############
    #### - physics (Physics)                desired implementation of physics/dynamics #################
    #### - normalized_spaces (bool)         whether to use normalized OpenAI Gym spaces ################
    #### - freq (int)                       the frequency (Hz) at which the simulation steps ###########
    #### - gui (bool)                       whether to use PyBullet's GUI ##############################
    #### - obstacles (bool)                 whether to add obstacles to the simulation #################
    #### - record (bool)                    whether to save the simulation as an .mp4 video ############
    #### - problem (Problem)                used to select reward, done, and normalization functions ###
    ####################################################################################################
    def __init__(self, drone_model: DroneModel=DroneModel.CF2X, num_drones: int=1, \
                        visibility_radius: float=np.inf, initial_xyzs=None, initial_rpys=None, \
                        physics: Physics=Physics.PYB, normalized_spaces=True, freq: int=240, \
                        gui=False, obstacles=False, record=False, problem: Problem=Problem.SA_TAKEOFF):
        super(Aviary, self).__init__()
        #### Parameters ####################################################################################
        self.DRONE_MODEL = drone_model; self.NUM_DRONES = num_drones; self.VISIBILITY_RADIUS = visibility_radius
        self.PHYSICS = physics; self.NORM_SPACES = normalized_spaces
        #### Constants #####################################################################################
        self.G = 9.8; self.RAD2DEG = 180/np.pi; self.DEG2RAD = np.pi/180
        self.SIM_FREQ = freq; self.TIMESTEP = 1./self.SIM_FREQ
        self.GUI = gui; self.OBSTACLES = obstacles; self.RECORD = record; self.PROBLEM = problem
        if self.DRONE_MODEL==DroneModel.CF2X: self.URDF = "cf2x.urdf"
        elif self.DRONE_MODEL==DroneModel.CF2P: self.URDF = "cf2p.urdf"
        elif self.DRONE_MODEL==DroneModel.HB: self.URDF = "hb.urdf"  
        #### Load the drone properties from the .urdf file #################################################
        self.M, self.L, self.THRUST2WEIGHT_RATIO, self.J, self.J_INV, self.KF, self.KM, self.COLLISION_H, self.COLLISION_R, self.COLLISION_Z_OFFSET, self.MAX_SPEED_KMH, self.GND_EFF_COEFF, self.PROP_RADIUS, self.DRAG_COEFF, self.DW_COEFF_1, self.DW_COEFF_2, self.DW_COEFF_3 = self._loadURDF()
        print("[INFO] Aviary.__init__() loaded parameters from the drone's .urdf:\n[INFO] m {:f}, L {:f},\n[INFO] ixx {:f}, iyy {:f}, izz {:f},\n[INFO] kf {:f}, km {:f},\n[INFO] t2w {:f}, max_speed_kmh {:f},\n[INFO] gnd_eff_coeff {:f}, prop_radius {:f},\n[INFO] drag_xy_coeff {:f}, drag_z_coeff {:f},\n[INFO] dw_coeff_1 {:f}, dw_coeff_2 {:f}, dw_coeff_3 {:f}".format(\
                                                                        self.M, self.L, self.J[0,0], self.J[1,1], self.J[2,2], self.KF, self.KM, self.THRUST2WEIGHT_RATIO, self.MAX_SPEED_KMH, self.GND_EFF_COEFF, self.PROP_RADIUS, self.DRAG_COEFF[0], self.DRAG_COEFF[2], self.DW_COEFF_1, self.DW_COEFF_2, self.DW_COEFF_3) )
        #### Compute constants #############################################################################
        self.GRAVITY = self.G*self.M; self.HOVER_RPM = np.sqrt(self.GRAVITY/(4*self.KF))
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY)/(4*self.KF)); self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2); self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)
        self.MAX_A = self.MAX_THRUST/self.M; self.MAX_PYB_KIN_V = self.MAX_SPEED_KMH*(1000/60**2)
        #### Connect to PyBullet ###########################################################################
        if self.GUI: 
            self.CLIENT = p.connect(p.GUI)
            for i in [p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW]: p.configureDebugVisualizer(i, 0, physicsClientId=self.CLIENT)
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0,0,0], physicsClientId=self.CLIENT)
            #### Add input sliders to the GUI ##################################################################
            self.SLIDERS = -1*np.ones(4)
            for i in range(4): self.SLIDERS[i] = p.addUserDebugParameter("Propeller "+str(i)+" RPM", 0, self.MAX_RPM, self.HOVER_RPM, physicsClientId=self.CLIENT)
            self.INPUT_SWITCH = p.addUserDebugParameter("Use GUI RPM", 9999, -1, 0, physicsClientId=self.CLIENT)
        else: self.CLIENT = p.connect(p.DIRECT)
        #### Set initial poses #############################################################################
        if initial_xyzs is None: self.INIT_XYZS = np.hstack([ np.array([x*4*self.L for x in range(self.NUM_DRONES)]), np.array([y*4*self.L for y in range(self.NUM_DRONES)]), np.ones(self.NUM_DRONES) * (self.COLLISION_H/2-self.COLLISION_Z_OFFSET+.1) ]).reshape(self.NUM_DRONES,3)
        elif np.array(initial_xyzs).shape==(self.NUM_DRONES,3): self.INIT_XYZS = initial_xyzs 
        else: print("[ERROR] invalid initial_xyzs in Aviary.__init__(), try initial_xyzs.reshape(NUM_DRONES,3)")
        if initial_rpys is None: self.INIT_RPYS = np.zeros((self.NUM_DRONES,3))
        elif np.array(initial_rpys).shape==(self.NUM_DRONES,3): self.INIT_RPYS = initial_rpys
        else: print("[ERROR] invalid initial_rpys in Aviary.__init__(), try initial_rpys.reshape(NUM_DRONES,3)")
        #### Create action and observation spaces ##########################################################
        if self.NORM_SPACES:
            #### Set bounds for normalized spaces ##############################################################
            act_lower_bound = np.array([-1,           -1,           -1,           -1])
            act_upper_bound = np.array([1,            1,            1,            1])
            obs_lower_bound = np.array([-1,      -1,      0,      -1,  -1,  -1,  -1,  -1,     -1,     -1,     -1,      -1,      -1,      -1,      -1,      -1,      -1,           -1,           -1,           -1])
            obs_upper_bound = np.array([1,       1,       1,      1,   1,   1,   1,   1,      1,      1,      1,       1,       1,       1,       1,       1,       1,            1,            1,            1])
        else:
            ##### Set bounds for unnormalized spaces ############################################################
            if self.PHYSICS in [Physics.PYB_PM, Physics.PYB_KIN]: 
                norm_upper_bound = self.MAX_A if self.PHYSICS==Physics.PYB_PM else self.MAX_PYB_KIN_V
                #### Action vector A ###### X             Y             Z             Norm
                act_lower_bound = np.array([-1,           -1,           -1,           0.])
                act_upper_bound = np.array([1,            1,            1,            norm_upper_bound])
            else: 
                #### Action vector B ###### P0            P1            P2            P3
                act_lower_bound = np.array([0.,           0.,           0.,           0.])
                act_upper_bound = np.array([self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WR       WP       WY       P0            P1            P2            P3
            obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
            obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        if self.NUM_DRONES==1:
            ##### Use simpler Box spaces for a single drone ####################################################
            self.action_space = spaces.Box( low=act_lower_bound, high=act_upper_bound, dtype=np.float32 )
            self.observation_space = spaces.Box( low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32 )
        else:
            ##### Use nested Dict spaces for multiple drones ###################################################
            self.action_space = spaces.Dict({ str(i): spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32) for i in range(self.NUM_DRONES) })
            self.observation_space = spaces.Dict({ str(i): spaces.Dict ({"state": spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32), \
                                                                        "neighbors": spaces.MultiBinary(self.NUM_DRONES) }) for i in range(self.NUM_DRONES) })
        
        #### Housekeeping ##################################################################################
        self._housekeeping()

    ####################################################################################################
    #### Reset the environment #########################################################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (..)                         initial observation, Dict() for multidrones, Box() for 1 ###
    ####################################################################################################
    def reset(self):
        p.resetSimulation(physicsClientId=self.CLIENT)
        #### Housekeeping ##################################################################################
        self._housekeeping()
        #### Start video recording #########################################################################
        if self.RECORD: self.VIDEO_ID = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=os.path.dirname(os.path.abspath(__file__))+"/../../files/video-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".mp4", physicsClientId=self.CLIENT)
        #### Return the initial observation ################################################################
        if self.NUM_DRONES==1: return self._getDroneState(0)
        else:
            adjacency_mat = self.getAdjacencyMatrix()
            return {str(i): {"state": self._getDroneState(i), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }

    ####################################################################################################
    #### Advance the environment by one simulation step ################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action (..)                      motors' speed (or commanded a, v), as Dict() or Box() ######
    ####################################################################################################
    #### Returns #######################################################################################
    #### - obs (..)                         initial observation, Dict() for multidrones, Box() for 1 ###
    #### - reward (..)                      reward value, Dict() for multidrones, float for 1 ##########
    #### - done (..)                        whether the current episode is over, Dict() or bool ########
    #### - info (Dict)                      currently unused ###########################################
    ####################################################################################################
    def step(self, action):
        self.step_counter += 1
        #### Read the GUI's input parameters ###############################################################
        if self.GUI: 
            current_input_switch = p.readUserDebugParameter(self.INPUT_SWITCH, physicsClientId=self.CLIENT)
            if current_input_switch>self.last_input_switch:
                self.last_input_switch = current_input_switch
                self.USE_GUI_RPM = True if self.USE_GUI_RPM==False else False
        if self.USE_GUI_RPM:
            for i in range(4): self.gui_input[i] = p.readUserDebugParameter(int(self.SLIDERS[i]), physicsClientId=self.CLIENT)
            clipped_action = np.tile(self.gui_input,(self.NUM_DRONES,1))
            if self.step_counter%(self.SIM_FREQ/2)==0: self.GUI_INPUT_TEXT = [ p.addUserDebugText("Using GUI RPM", textPosition=[0,0,0], textColorRGB=[1,0,0], lifeTime=1, textSize=2, parentObjectUniqueId=self.DRONE_IDS[i], parentLinkIndex=-1, replaceItemUniqueId=int(self.GUI_INPUT_TEXT[i]), physicsClientId=self.CLIENT) for i in range(self.NUM_DRONES) ]
        #### Denormalize (if necessary) and clip the action to the maximum RPM #############################
        else:   
            #### Transform 1-drone action into the Dict format of multiple drones to simplify the code #########
            if self.NUM_DRONES==1: action = {"0": action}
            clipped_action = np.zeros((self.NUM_DRONES,4))
            for k, v in action.items(): 
                self.last_action[int(k),:] = v
                #### Manage commanded acceleration #################################################################
                if self.PHYSICS==Physics.PYB_PM: 
                    acc = self._normActionToAcceleration(action) if self.NORM_SPACES else np.array(action)
                    clipped_action[int(k),:] = acc  # TODO implement clipping
                #### Manage commanded velocity #####################################################################
                elif self.PHYSICS==Physics.PYB_KIN:
                    vel = self._normActionToVelocity(action) if self.NORM_SPACES else np.array(action)
                    clipped_action[int(k),:] = vel  # TODO implement clipping
                #### Manage commanded RPM ##########################################################################
                else:
                    rpm = self._normActionToRPM(v) if self.NORM_SPACES else np.array(v)
                    clipped_action[int(k),:] = np.clip(np.array(rpm), 0, self.MAX_RPM)
        #### Step the simulation using the desired physics update ##########################################
        for i in range (self.NUM_DRONES):
            self._showDroneFrame(i)
            if self.PHYSICS==Physics.PYB: self._physics(clipped_action[i,:], i)
            elif self.PHYSICS==Physics.DYN: self._dynamics(clipped_action[i,:], i)
            elif self.PHYSICS==Physics.PYB_GND: self._physics(clipped_action[i,:], i); self._groundEffect(clipped_action[i,:], i)
            elif self.PHYSICS==Physics.PYB_DRAG: self._physics(clipped_action[i,:], i); self._drag(self.last_clipped_action[i,:], i)
            elif self.PHYSICS==Physics.PYB_DW: self._physics(clipped_action[i,:], i); self._downwash(i)
            elif self.PHYSICS==Physics.PYB_GND_DRAG_DW: self._physics(clipped_action[i,:], i); self._groundEffect(clipped_action[i,:], i); self._drag(self.last_clipped_action[i,:], i); self._downwash(i)
            elif self.PHYSICS==Physics.PYB_PM: self._pointMass(clipped_action[i,:], i)
            elif self.PHYSICS==Physics.PYB_KIN: self._kinematics(clipped_action[i,:], i)
        #### Let PyBullet compute the new state, unless using Physics.DYN ##################################
        if self.PHYSICS!=Physics.DYN: p.stepSimulation(physicsClientId=self.CLIENT)
        #### Save the last applied action to compute drag in the next step #################################
        if self.PHYSICS in [Physics.PYB_DRAG, Physics.PYB_GND_DRAG_DW]: self.last_clipped_action = clipped_action
        #### Prepare the return values #####################################################################
        if self.NUM_DRONES==1:
            obs = self._getDroneState(0)
            reward = self.RL_FUNCTIONS.rewardFunction(obs)
            done = self.RL_FUNCTIONS.doneFunction(obs, self.step_counter/self.SIM_FREQ)
        else:
            adjacency_mat = self.getAdjacencyMatrix()
            obs = {str(i): {"state": self._getDroneState(i), "neighbors": adjacency_mat[i,:] } for i in range(self.NUM_DRONES) }
            reward = 0.         # TODO,  self.RL_FUNCTIONS.rewardFunction(obs)
            done = False        # TODO,  self.RL_FUNCTIONS.doneFunction(obs, self.step_counter/self.SIM_FREQ)
        return obs, reward, done, {}

    ####################################################################################################
    #### Print a textual output of the environment #####################################################
    ####################################################################################################
    def render(self, mode='human', close=False):
        if self.first_render_call and not self.GUI: 
            print("[WARNING] Aviary.render() is implemented as text-only, re-initialize the environment using Aviary(gui=True) to use PyBullet's graphical interface")
            self.first_render_call = False
        print("\n[INFO] Aviary.render() ——— it {:04d}".format(self.step_counter), 
            "——— wall-clock time {:.1f}s,".format(time.time()-self.RESET_TIME), 
            "simulation time {:.1f}s@{:d}Hz ({:.2f}x)".format(self.step_counter*self.TIMESTEP, self.SIM_FREQ, (self.step_counter*self.TIMESTEP)/(time.time()-self.RESET_TIME)))
        for i in range (self.NUM_DRONES):
            pos, quat = p.getBasePositionAndOrientation(self.DRONE_IDS[i], physicsClientId=self.CLIENT)
            rpy = p.getEulerFromQuaternion(quat)
            vel, ang_v = p.getBaseVelocity(self.DRONE_IDS[i], physicsClientId=self.CLIENT)
            print("[INFO] Aviary.render() ——— drone {:d}".format(i), 
                "——— x {:+06.2f}, y {:+06.2f}, z {:+06.2f}".format(pos[0], pos[1], pos[2]), 
                "——— velocity {:+06.2f}, {:+06.2f}, {:+06.2f}".format(vel[0], vel[1], vel[2]), 
                "——— roll {:+06.2f}, pitch {:+06.2f}, yaw {:+06.2f}".format(rpy[0]*self.RAD2DEG, rpy[1]*self.RAD2DEG, rpy[2]*self.RAD2DEG),
                "——— angular velocities {:+06.2f}, {:+06.2f}, {:+06.2f} ——— ".format(ang_v[0]*self.RAD2DEG, ang_v[1]*self.RAD2DEG, ang_v[2]*self.RAD2DEG))

    ####################################################################################################
    #### Close the environment #########################################################################
    ####################################################################################################
    def close(self):
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
    def getDroneIds(self):
        return self.DRONE_IDS

    ####################################################################################################
    #### Compute the adjacency matrix of a multidrone system using VISIBILITY_RADIUS ###################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - adj_mat ((NUM_DRONES,NUM_DRONES) array)    adj_mat[i,j]=1 if i,j are neighbors, 0 otherwise #
    ####################################################################################################
    def getAdjacencyMatrix(self):
        adjacency_mat = np.identity(self.NUM_DRONES)
        for i in range(self.NUM_DRONES-1):
            for j in range(self.NUM_DRONES-i-1):
                pos_i, _ = p.getBasePositionAndOrientation(self.DRONE_IDS[i], physicsClientId=self.CLIENT)
                pos_j, _ = p.getBasePositionAndOrientation(self.DRONE_IDS[j+i+1], physicsClientId=self.CLIENT)
                if np.linalg.norm(np.array(pos_i)-np.array(pos_j))<self.VISIBILITY_RADIUS: adjacency_mat[i,j] = adjacency_mat[j,i] = 1
        return adjacency_mat

    ####################################################################################################
    #### Denormalize the [-1,1] range to the [0, MAX RPM] range ########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - action ((4,1) array)             normalized [-1,1] actions applied to the 4 motors ##########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _normActionToRPM(self, action): 
        if np.any(np.abs(action))>1: print("\n[ERROR] it", self.step_counter, "in Aviary._normActionToRPM(), out-of-bound action")
        return np.where(action <= 0, (action+1)*self.HOVER_RPM, action*self.MAX_RPM)

    ####################################################################################################
    #### Normalize the [0, MAX RPM] range to the [-1,1] range ##########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - rpm ((4,1) array)                RPM values of the 4 motors #################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - action ((4,1) array)             normalized action to apply to the 4 motors #################
    ####################################################################################################
    def _rpmToNormAction(self, rpm): 
        if np.any(rpm)<0: print("\n[ERROR] it", self.step_counter, "in Aviary._rpmToNormAction(), negative RPM")
        return np.where(rpm <= self.HOVER_RPM, (rpm/self.HOVER_RPM)-1, rpm/self.MAX_RPM)

    ####################################################################################################
    #### Load parameters from the .urdf file ###########################################################
    ####################################################################################################
    def _loadURDF(self):
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
    #### Housekeeping shared by the __init__() and reset() functions ###################################
    ####################################################################################################
    def _housekeeping(self):
        #### Initialize/reset counters and zero-valued variables ###########################################
        self.RESET_TIME = time.time(); self.step_counter = 0; self.first_render_call = True
        self.X_AX = -1*np.ones(self.NUM_DRONES); self.Y_AX = -1*np.ones(self.NUM_DRONES); self.Z_AX = -1*np.ones(self.NUM_DRONES);
        self.GUI_INPUT_TEXT = -1*np.ones(self.NUM_DRONES); self.USE_GUI_RPM=False; self.last_input_switch = 0
        self.RL_FUNCTIONS = RLFunctions(self.CLIENT, self.GUI, self.PROBLEM)
        self.last_action = -1*np.ones((self.NUM_DRONES,4)) if self.NORM_SPACES else np.zeros((self.NUM_DRONES,4))
        self.last_clipped_action = np.zeros((self.NUM_DRONES,4)); self.gui_input = np.zeros(4)
        self.no_pybullet_dyn_accs = np.zeros((self.NUM_DRONES,3)); 
        #### Set PyBullet's parameters #####################################################################
        p.setGravity(0, 0, -self.G, physicsClientId=self.CLIENT)
        p.setRealTimeSimulation(0, physicsClientId=self.CLIENT)
        p.setTimeStep(self.TIMESTEP, physicsClientId=self.CLIENT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.CLIENT)
        #### Load ground plane, drone and obstacles models #################################################
        p.loadURDF("plane.urdf", physicsClientId=self.CLIENT)        
        self.DRONE_IDS = np.array([p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/../assets/"+self.URDF, self.INIT_XYZS[i,:], p.getQuaternionFromEuler(self.INIT_RPYS[i,:]), physicsClientId=self.CLIENT) for i in range(self.NUM_DRONES)])
        if self.OBSTACLES: self.RL_FUNCTIONS.addObstacles()

    ####################################################################################################
    #### Return the state vector of the nth drone ######################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - state (Box (20,))                the state vector of the nth drone ##########################
    ####################################################################################################
    def _getDroneState(self, nth_drone):
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        state = np.hstack([pos, quat, rpy, vel, ang_v, self.last_action[nth_drone,:]])
        if self.NORM_SPACES: state = self.RL_FUNCTIONS.clipAndNormalizeState(state, self.step_counter)
        return state.reshape(20,)

    ####################################################################################################
    #### Draw the local frame of the nth drone #########################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _showDroneFrame(self, nth_drone):
        AXIS_LENGTH = 2*self.L
        self.X_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[AXIS_LENGTH,0,0], lineColorRGB=[1,0,0], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.X_AX[nth_drone]), physicsClientId=self.CLIENT)
        self.Y_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,AXIS_LENGTH,0], lineColorRGB=[0,1,0], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.Y_AX[nth_drone]), physicsClientId=self.CLIENT)
        self.Z_AX[nth_drone] = p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,0,AXIS_LENGTH], lineColorRGB=[0,0,1], parentObjectUniqueId=self.DRONE_IDS[nth_drone], parentLinkIndex=-1, replaceItemUniqueId=int(self.Z_AX[nth_drone]), physicsClientId=self.CLIENT)
    
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
        #### Kinematic information of the base and all links (propellers and center of mass) ###############
        base_pos, base_quat = p.getBasePositionAndOrientation(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        base_rpy = p.getEulerFromQuaternion(base_quat)
        link_states = np.array(p.getLinkStates(self.DRONE_IDS[nth_drone], linkIndices=[0,1,2,3,4], computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=self.CLIENT))
        #### Simple, per-propeller ground effects ##########################################################
        prop_heights = np.array([link_states[0,0][2], link_states[1,0][2], link_states[2,0][2], link_states[3,0][2]])
        gnd_effects = np.array(rpm**2) * self.KF * self.GND_EFF_COEFF * (self.PROP_RADIUS/(4 * prop_heights))**2
        gnd_effects = np.clip(gnd_effects, 0, self.MAX_THRUST/10)
        if np.abs(base_rpy[0])<np.pi/2 and np.abs(base_rpy[1])<np.pi/2:
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
        #### Kinematic information of the base #############################################################
        base_pos, base_quat = p.getBasePositionAndOrientation(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        base_rot = np.array(p.getMatrixFromQuaternion(base_quat)).reshape(3,3)
        base_vel, base_ang_v = p.getBaseVelocity(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        #### Simple draft model applied to the base/center of mass #########################################
        drag_factors = -1 * self.DRAG_COEFF * np.sum(np.array(2*np.pi*rpm/60))
        drag = np.dot(base_rot, drag_factors*np.array(base_vel))
        p.applyExternalForce(self.DRONE_IDS[nth_drone], 4, forceObj=drag, posObj=[0,0,0], flags=p.LINK_FRAME, physicsClientId=self.CLIENT) 

    ####################################################################################################
    #### PyBullet implementation of ground effect, SiQi Zhou's modelling ###############################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - nth_drone (int)                  order position of the drone in list self.DRONE_IDS #########
    ####################################################################################################
    def _downwash(self, nth_drone):
        # self.DW_COEFF_1 = 2267.18
        # self.DW_COEFF_2 = .16
        # self.DW_COEFF_3 = -.11
        pos, _ = p.getBasePositionAndOrientation(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        for i in range(self.NUM_DRONES):
            pos_other, _ = p.getBasePositionAndOrientation(self.DRONE_IDS[i], physicsClientId=self.CLIENT)
            delta_z = pos_other[2]-pos[2]; delta_xy = np.linalg.norm(np.array(pos_other[0:2])-np.array(pos[0:2]))
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
        pos, quat = p.getBasePositionAndOrientation(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        vel, ang_v = p.getBaseVelocity(self.DRONE_IDS[nth_drone], physicsClientId=self.CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        #### Compute forces and torques ####################################################################
        forces = np.array(rpm**2) * self.KF
        thrust = np.array([0, 0, np.sum(forces)])
        thrust_world_frame = np.dot(rotation,thrust)
        force_world_frame = thrust_world_frame - np.array([0, 0, self.GRAVITY])
        z_torques = np.array(rpm**2)*self.KM
        z_torque = (-z_torques[0] + z_torques[1] - z_torques[2] + z_torques[3])
        if self.DRONE_MODEL==DroneModel.CF2X:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3]) * self.L/np.sqrt(2)
            y_torque = (- forces[0] + forces[1] + forces[2] - forces[3]) * self.L/np.sqrt(2)
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
    #### Work in progress ##############################################################################
    ####################################################################################################
    def _normActionToAcceleration(self, action):
        pass # add checks

    def _accelerationToNormAction(self, acc):
        pass # add checks

    def _normActionToVelocity(self, action):
        pass # add checks

    def _velocityToNormAction(self, vel):
        pass # add checks

    def _pointMass(self, acc, nth_drone):
        pass # TODO

    def _kinematics(self, vel, nth_drone):
        pass # TODO




