import math
import numpy as np
import pybullet as p

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary
from gym_pybullet_drones.utils.utils import nnlsRPM

class SimplePIDControl(BaseControl):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (BaseAviary)                 simulation environment #####################################
    ####################################################################################################
    def __init__(self,
                 env: BaseAviary
                 ):
        super().__init__(env=env)
        if self.DRONE_MODEL != DroneModel.HB:
            print("[ERROR] in SimplePIDControl.__init__(), DSLPIDControl requires DroneModel.HB")
            exit()
        self.P_COEFF_FOR = np.array([.1, .1, .2])
        self.I_COEFF_FOR = np.array([.0001, .0001, .0001])
        self.D_COEFF_FOR = np.array([.3, .3, .4])
        self.P_COEFF_TOR = np.array([.3, .3, .05])
        self.I_COEFF_TOR = np.array([.0001, .0001, .0001])
        self.D_COEFF_TOR = np.array([.3, .3, .5])
        self.MAX_ROLL_PITCH = np.pi/6
        self.MAX_THRUST = env.MAX_THRUST
        self.MAX_XY_TORQUE = env.MAX_XY_TORQUE
        self.MAX_Z_TORQUE = env.MAX_Z_TORQUE
        self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ])
        self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([1/self.KF, 1/(self.KF*env.L), 1/(self.KF*env.L), 1/self.KM])
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        super().reset()
        #### Initialized PID control variables #####################
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

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
    def computeControl(self,
                       control_timestep,
                       cur_pos,
                       cur_quat,
                       cur_vel,
                       cur_ang_vel,
                       target_pos,
                       target_rpy=np.zeros(3),
                       target_vel=np.zeros(3),
                       target_ang_vel=np.zeros(3)
                       ):
        self.control_counter += 1
        if target_rpy[2]!=0:
            print("\n[WARNING] ctrl it", self.control_counter, "in SimplePIDControl.computeControl(), desired yaw={:.0f}deg but locked to 0. for DroneModel.HB".format(target_rpy[2]*(180/np.pi)))
        thrust, computed_target_rpy, pos_e = self._simplePIDPositionControl(control_timestep,
                                                                            cur_pos,
                                                                            cur_quat,
                                                                            target_pos
                                                                            )
        rpm = self._simplePIDAttitudeControl(control_timestep,
                                             thrust,
                                             cur_quat,
                                             computed_target_rpy
                                             )
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]

    ####################################################################################################
    #### Generic PID position control (with yaw locked to 0.) ##########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - cur_pos ((3,1) array)            current position ###########################################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - target_pos ((3,1) array)         desired position ###########################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - thrust (float)                   thrust along the drone z-axis ##############################
    #### - target_rpy ((3,1) array)         computed target roll, pitch, and yaw #######################
    #### - yaw_e (float)                    current yaw error ##########################################
    ####################################################################################################
    def _simplePIDPositionControl(self,
                                  control_timestep,
                                  cur_pos,
                                  cur_quat,
                                  target_pos
                                  ):
        pos_e = target_pos - np.array(cur_pos).reshape(3)
        d_pos_e = (pos_e - self.last_pos_e) / control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        #### PID target thrust #####################################
        target_force = np.array([0, 0, self.GRAVITY]) \
                       + np.multiply(self.P_COEFF_FOR, pos_e) \
                       + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                       + np.multiply(self.D_COEFF_FOR, d_pos_e)
        target_rpy = np.zeros(3)
        sign_z =  np.sign(target_force[2])
        if sign_z == 0:
            sign_z = 1
        #### Target rotation #######################################
        target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
        target_rpy[1] = np.arctan2(sign_z*target_force[0], sign_z*target_force[2])
        target_rpy[2] = 0.
        target_rpy[0] = np.clip(target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        target_rpy[1] = np.clip(target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        thrust = np.dot(cur_rotation, target_force)
        return thrust[2], target_rpy, pos_e

    ####################################################################################################
    #### Generic PID attitude control (with yaw locked to 0.) ##########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         time step at which control is computed #####################
    #### - thrust (float)                   desired thrust along the drone z-axis ######################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - target_rpy ((3,1) array)         computed target roll, pitch, and yaw #######################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _simplePIDAttitudeControl(self,
                                  control_timestep,
                                  thrust,
                                  cur_quat,
                                  target_rpy
                                  ):
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        rpy_e = target_rpy - np.array(cur_rpy).reshape(3,)
        if rpy_e[2] > np.pi:
            rpy_e[2] = rpy_e[2] - 2*np.pi
        if rpy_e[2] < -np.pi:
            rpy_e[2] = rpy_e[2] + 2*np.pi
        d_rpy_e = (rpy_e - self.last_rpy_e) / control_timestep
        self.last_rpy_e = rpy_e
        self.integral_rpy_e = self.integral_rpy_e + rpy_e*control_timestep
        #### PID target torques ####################################
        target_torques = np.multiply(self.P_COEFF_TOR, rpy_e) \
                         + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e) \
                         + np.multiply(self.D_COEFF_TOR, d_rpy_e)
        return nnlsRPM(thrust=thrust,
                       x_torque=target_torques[0],
                       y_torque=target_torques[1],
                       z_torque=target_torques[2],
                       counter=self.control_counter,
                       max_thrust=self.MAX_THRUST,
                       max_xy_torque=self.MAX_XY_TORQUE,
                       max_z_torque=self.MAX_Z_TORQUE,
                       a=self.A,
                       inv_a=self.INV_A,
                       b_coeff=self.B_COEFF,
                       gui=True
                       )
 