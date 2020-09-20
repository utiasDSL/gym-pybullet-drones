import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary


class DSLPIDControl(BaseControl):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (BaseAviary)                 simulation environment #####################################
    ####################################################################################################
    def __init__(self, env: BaseAviary):
        super().__init__(env=env)
        if self.DRONE_MODEL!=DroneModel.CF2X and self.DRONE_MODEL!=DroneModel.CF2P: print("[ERROR] in DSLPIDControl.__init__(), DSLPIDControl requires DroneModel.CF2X or DroneModel.CF2P"); exit()
        self.P_COEFF_FOR = np.array([.4, .4, 1.25]); self.I_COEFF_FOR = np.array([.05, .05, .05]); self.D_COEFF_FOR = np.array([.2, .2, .5])
        self.P_COEFF_TOR = np.array([70000., 70000., 60000.]); self.I_COEFF_TOR = np.array([.0, .0, 500.]); self.D_COEFF_TOR = np.array([20000., 20000., 12000.])
        self.PWM2RPM_SCALE = 0.2685; self.PWM2RPM_CONST = 4070.3; self.MIN_PWM = 20000; self.MAX_PWM = 65535
        if self.DRONE_MODEL==DroneModel.CF2X: self.MIXER_MATRIX = np.array([ [.5, -.5,  -1], [.5, .5, 1], [-.5,  .5,  -1], [-.5, -.5, 1] ])
        elif self.DRONE_MODEL==DroneModel.CF2P: self.MIXER_MATRIX = np.array([ [0, -1,  -1], [+1, 0, 1], [0,  1,  -1], [-1, 0, 1] ])
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        super().reset()
        #### Initialized PID control variables #############################################################
        self.last_pos_e = np.zeros(3); self.integral_pos_e = np.zeros(3); self.last_rpy_e = np.zeros(3); self.integral_rpy_e = np.zeros(3)

    ####################################################################################################
    #### Compute the control action for a single drone #################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - cur_pos ((3,1) array)            current position ###########################################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - cur_vel ((3,1) array)            current velocity ###########################################
    #### - cur_ang_vel ((3,1) array)        current angular velocity ###################################
    #### - target_pos ((3,1) array)         desired position ###########################################
    #### - target_rpy ((3,1) array)         desired orientation as roll, pitch, yaw ####################
    #### - target_vel ((3,1) array)         desired velocity ###########################################
    #### - target_ang_vel ((3,1) array)     desired angular velocity ###################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    #### - pos_e ((3,1) array)              current XYZ position error #################################
    #### - yaw_e (float)                    current yaw error ##########################################
    ####################################################################################################
    def computeControl(self, control_timestep, cur_pos, cur_quat, cur_vel, cur_ang_vel,
                        target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        self.control_counter += 1
        thrust, computed_target_rpy, pos_e = self._dslPIDPositionControl(control_timestep, cur_pos, cur_quat, cur_vel, target_pos, target_rpy, target_vel)
        rpm = self._dslPIDAttitudeControl(control_timestep, thrust, cur_quat, cur_ang_vel, computed_target_rpy, target_ang_vel)
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2]-cur_rpy[2]

    ####################################################################################################
    #### DSL's CF2.x PID position control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step (inverse of freq.) at which control is computed ##
    #### - cur_pos ((3,1) array)            current position ###########################################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - cur_vel ((3,1) array)            current velocity ###########################################
    #### - target_pos ((3,1) array)         desired position ###########################################
    #### - target_rpy ((3,1) array)         desired orientation as roll, pitch, yaw ####################
    #### - target_vel ((3,1) array)         desired velocity ###########################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - thrust (float)                   thrust along the drone z-axis ##############################
    #### - target_rpy ((3,1) array)         target roll, pitch, and yaw ################################
    #### - yaw_e (float)                    current yaw error ##########################################
    ####################################################################################################
    def _dslPIDPositionControl(self, control_timestep, cur_pos, cur_quat, cur_vel, target_pos, target_rpy, target_vel):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3,3)
        pos_e = target_pos - cur_pos
        vel_e = target_vel - cur_vel
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.); self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)
        #### PID target thrust #############################################################################
        target_thrust = np.multiply(self.P_COEFF_FOR, pos_e) + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) + np.multiply(self.D_COEFF_FOR, vel_e) + np.array([0,0,self.GRAVITY])
        scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
        thrust = (math.sqrt(scalar_thrust / (4*self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        target_z_ax = target_thrust / np.linalg.norm(target_thrust)
        target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
        target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
        #### Target rotation ###############################################################################
        target_euler = (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)
        if np.any(np.abs(target_euler)>math.pi): print("\n[ERROR] ctrl it", self.control_counter, "in Control._dslPIDPositionControl(), values outside range [-pi,pi]")
        return thrust, target_euler, pos_e

    ####################################################################################################
    #### DSL's CF2.x PID attitude control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - thrust (float)                   desired thrust along the drone z-axis ######################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - cur_ang_vel ((3,1) array)        current angular velocity ###################################
    #### - target_euler ((3,1) array)       computed target Euler angles ###############################
    #### - target_ang_vel ((3,1) array)     desired angular velocity ###################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _dslPIDAttitudeControl(self, control_timestep, thrust, cur_quat, cur_ang_vel, target_euler, target_ang_vel):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3,3)
        target_quat = (Rotation.from_euler('XYZ', target_euler, degrees=False)).as_quat()
        w,x,y,z = target_quat
        target_rotation = (Rotation.from_quat([w,x,y,z])).as_matrix()
        rot_matrix_e = np.dot((target_rotation.transpose()),cur_rotation) - np.dot(cur_rotation.transpose(),target_rotation)
        rot_e = np.array([rot_matrix_e[2,1], rot_matrix_e[0,2], rot_matrix_e[1,0]])
        ang_vel_e = target_ang_vel - cur_ang_vel
        self.integral_rpy_e = self.integral_rpy_e - rot_e*control_timestep
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.); self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
        #### PID target torques ############################################################################
        target_torques = - np.multiply(self.P_COEFF_TOR, rot_e) + np.multiply(self.D_COEFF_TOR, ang_vel_e) + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
        target_torques = np.clip(target_torques, -3200, 3200)
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        return self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST

    ####################################################################################################
    #### Utility function interfacing 1, 2, or 3D use cases ############################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - thrust ((?,1) array)             a desired thrust input with 1, 2, or 4 components ##########
    ####################################################################################################
    #### Returns #######################################################################################
    #### - pwm ((4,1) array)                PWM values to apply to the 4 motors ########################
    ####################################################################################################
    def _one23DInterface(thrust):
        DIM = len(np.array(thrust)); pwm = np.clip((np.sqrt(np.array(thrust)/(self.KF*(4/DIM)))-self.PWM2RPM_CONST)/self.PWM2RPM_SCALE, self.MIN_PWM, self.MAX_PWM)
        if DIM in [1, 4]: return np.repeat(pwm, 4/DIM)
        elif DIM==2: return np.hstack([pwm, np.flip(pwm)])
        else: print("[ERROR] in one23DInterface()"); exit()