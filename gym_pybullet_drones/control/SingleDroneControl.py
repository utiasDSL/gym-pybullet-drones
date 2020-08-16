import math
import numpy as np
import pybullet as p
from enum import Enum
from scipy.optimize import nnls
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.SingleDroneEnv import DroneModel, SingleDroneEnv


######################################################################################################################################################
#### Control types enumeration #######################################################################################################################
######################################################################################################################################################
class ControlType(Enum):
    PID = 0                  # PID control


######################################################################################################################################################
#### Control class ###################################################################################################################################
######################################################################################################################################################
class SingleDroneControl(object):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (SingleDroneEnv)             simulation environment #####################################
    #### - control_type (ControlType)       type of controller #########################################
    ####################################################################################################
    def __init__(self, env: SingleDroneEnv, control_type: ControlType=ControlType.PID):
        ####################################################################################################
        #### Set general use constants #####################################################################
        ####################################################################################################
        self.DRONE_MODEL = env.DRONE_MODEL; self.GRAVITY = env.GRAVITY; self.KF = env.KF; self.KT = env.KM
        self.MAX_THRUST = env.MAX_THRUST; self.MAX_XY_TORQUE = env.MAX_XY_TORQUE; self.MAX_Z_TORQUE = env.MAX_Z_TORQUE
        self.CONTROLTYPE = control_type
        self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [1, -1, 1, -1] ]); self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([1/env.KF, 1/(env.KF*env.L), 1/(env.KF*env.L), 1/env.KM]) 
        self.RAD2DEG = 180/np.pi; self.DEG2RAD = np.pi/180
        ####################################################################################################
        #### Set drone model-specific constants  ###########################################################
        ####################################################################################################
        if self.DRONE_MODEL==DroneModel.HB:
            self.P_COEFF_FOR = np.array([.1, .1, .2]); self.I_COEFF_FOR = np.array([.0001, .0001, .0001]); self.D_COEFF_FOR = np.array([.3, .3, .4])
            self.P_COEFF_TOR = np.array([.3, .3, .05]); self.I_COEFF_TOR = np.array([.0001, .0001, .0001]); self.D_COEFF_TOR = np.array([.3, .3, .5])
            self.MAX_ROLL_PITCH = np.pi/6
        elif self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P:
            self.P_COEFF_FOR = np.array([.4, .4, 1.25]); self.I_COEFF_FOR = np.array([.05, .05, .05]); self.D_COEFF_FOR = np.array([.2, .2, .5]) 
            self.P_COEFF_TOR = np.array([70000., 70000., 60000.]); self.I_COEFF_TOR = np.array([.0, .0, 500.]); self.D_COEFF_TOR = np.array([20000., 20000., 12000.])
            self.PWM2RPM_SCALE = 0.2685; self.PWM2RPM_CONST = 4070.3; self.MIN_PWM = 20000; self.MAX_PWM = 65535
            if self.DRONE_MODEL==DroneModel.CF2X: self.MIXER_MATRIX = np.array([ [.5, -.5,  1], [.5, .5, -1], [-.5,  .5,  1], [-.5, -.5, -1] ]) 
            elif self.DRONE_MODEL==DroneModel.CF2P: self.MIXER_MATRIX = np.array([ [0, -1,  1], [+1, 0, -1], [0,  1,  1], [-1, 0, -1] ])
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        self.last_pos_e = np.zeros(3); self.integral_pos_e = np.zeros(3); self.last_rpy_e = np.zeros(3); self.integral_rpy_e = np.zeros(3)
        self.control_counter = 0

    ####################################################################################################
    #### Compute the control action ####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         timestep at which control is computed ######################
    #### - cur_pos (3-by-1 array)           current position ###########################################
    #### - cur_quat_rpy (4-by-1 array)      current orientation as a quaternion ########################
    #### - cur_vel (3-by-1 array)           current velocity ###########################################
    #### - cur_ang_vel (3-by-1 array)       current angular velocity ###################################
    #### - target_pos (3-by-1 array)        desired position ###########################################
    #### - target_rpy (3-by-1 array)        desired orientation as roll, pitch, yaw ####################
    #### - target_vel (3-by-1 array)        desired velocity ###########################################
    #### - target_ang_vel (3-by-1 array)    desired angular velocity ###################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4-by-1 array)               RPM values to apply to the 4 motors ########################
    #### - pos_err (3-by-1 array)           current XYZ position error #################################
    #### - yaw_err (Float)                  current yaw error ##########################################
    ####################################################################################################
    def computeControl(self, control_timestep, cur_pos, cur_quat_rpy, cur_vel, cur_ang_vel, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        self.control_counter += 1
        if self.CONTROLTYPE==ControlType.PID:
            ####################################################################################################
            #### XYZ PID control tuned for DroneModel.HB #######################################################
            #### based on https://github.com/prfraanje/quadcopter_sim ##########################################
            ####################################################################################################
            if self.DRONE_MODEL==DroneModel.HB:
                if target_rpy[2] != 0: print("\n[WARNING] ctrl it:", self.control_counter, "in computeControl(), desired yaw={:.0f}deg but yaw control is NOT implemented for DroneModel.HB".format(target_rpy[2]*self.RAD2DEG))
                thrust, computed_target_rpy, pos_err = self._simplePIDPositionControl(control_timestep, cur_pos, cur_quat_rpy, target_pos)
                rpm = self._simplePIDAttitudeControl(control_timestep, cur_quat_rpy, thrust, computed_target_rpy)
            ####################################################################################################
            #### PID control tuned for Bitcraze's Crazyflie 2.x ################################################
            ####################################################################################################
            elif self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P:  
                base_thrust, computed_target_rpy, pos_err = self._dslPIDPositionControl(control_timestep, cur_pos, cur_quat_rpy, cur_vel, target_pos, target_rpy, target_vel)
                rpm = self._dslPIDAttitudeControl(control_timestep, cur_quat_rpy, base_thrust, computed_target_rpy, cur_ang_vel, target_ang_vel)
            cur_rpy = p.getEulerFromQuaternion(cur_quat_rpy)
            return rpm, pos_err, computed_target_rpy[2]-cur_rpy[2]
        else: print("[ERROR] ctrl it:", self.control_counter, "ControlleType not yet implemented")


######################################################################################################################################################
#### Internals #######################################################################################################################################
######################################################################################################################################################

    ####################################################################################################
    #### Generic PID position control (without yaw) ####################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         timestep at which control is computed ######################
    #### - cur_pos (3-by-1 array)           current position ###########################################
    #### - cur_quat_rpy (4-by-1 array)      current orientation as a quaternion ########################
    #### - target_pos (3-by-1 array)        desired position ###########################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - thrust (Float)                   thrust along the drone z-axis ##############################
    #### - target_rpy (3-by-1 array)        computed target roll, pitch, and yaw #######################
    #### - yaw_err (Float)                  current yaw error ##########################################
    ####################################################################################################
    def _simplePIDPositionControl(self, control_timestep, cur_pos, cur_quat_rpy, target_pos):
        pos_e = target_pos - np.array(cur_pos).reshape(3)
        d_pos_e = (pos_e - self.last_pos_e) / control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        target_force = np.array([0,0,self.GRAVITY]) + np.multiply(self.P_COEFF_FOR,pos_e) + np.multiply(self.I_COEFF_FOR,self.integral_pos_e) + np.multiply(self.D_COEFF_FOR,d_pos_e)
        computed_target_rpy = np.zeros(3)
        sign_z =  np.sign(target_force[2])
        if sign_z == 0: sign_z = 1 
        computed_target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
        computed_target_rpy[1] = np.arctan2(sign_z*target_force[0],sign_z*target_force[2])
        computed_target_rpy[2] = 0.
        computed_target_rpy[0] = np.clip(computed_target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        computed_target_rpy[1] = np.clip(computed_target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat_rpy)).reshape(3,3)
        thrust = np.dot(cur_rotation, target_force)
        return thrust[2], computed_target_rpy, pos_e

    ####################################################################################################
    #### Generic PID attiutude control (without yaw) ###################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         timestep at which control is computed ######################
    #### - cur_quat_rpy (4-by-1 array)      current orientation as a quaternion ########################
    #### - thrust (Float)                   desired thrust along the drone z-axis ######################
    #### - comp_tar_rpy (3-by-1 array)      computed target roll, pitch, and yaw #######################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4-by-1 array)               RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _simplePIDAttitudeControl(self, control_timestep, cur_quat_rpy, thrust, computed_target_rpy):
        cur_rpy = p.getEulerFromQuaternion(cur_quat_rpy)
        rpy_e = computed_target_rpy - np.array(cur_rpy).reshape(3,) 
        if rpy_e[2] > np.pi: rpy_e[2] = rpy_e[2] - 2*np.pi
        if rpy_e[2] < -np.pi: rpy_e[2] = rpy_e[2] + 2*np.pi
        d_rpy_e = (rpy_e - self.last_rpy_e) / control_timestep
        self.last_rpy_e = rpy_e
        self.integral_rpy_e = self.integral_rpy_e + rpy_e*control_timestep
        target_torques = np.multiply(self.P_COEFF_TOR,rpy_e) + np.multiply(self.I_COEFF_TOR,self.integral_rpy_e) + np.multiply(self.D_COEFF_TOR,d_rpy_e)
        # cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat_rpy)).reshape(3,3)
        # target_torques = np.dot(cur_rotation, target_torques)     # Unnecessary for fixed yaw
        return self._physicsToRPM(thrust, target_torques[0], target_torques[1], target_torques[2])

    ####################################################################################################
    #### DSL's CF2.x PID position control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         timestep at which control is computed ######################
    #### - cur_pos (3-by-1 array)           current position ###########################################
    #### - cur_quat_rpy (4-by-1 array)      current orientation as a quaternion ########################
    #### - cur_vel (3-by-1 array)           current velocity ###########################################
    #### - target_pos (3-by-1 array)        desired position ###########################################
    #### - target_rpy (3-by-1 array)        desired orientation as roll, pitch, yaw ####################
    #### - target_vel (3-by-1 array)        desired velocity ###########################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - thrust (Float)                   thrust along the drone z-axis ##############################
    #### - target_rpy (3-by-1 array)        target roll, pitch, and yaw ################################
    #### - yaw_err (Float)                  current yaw error ##########################################
    ####################################################################################################
    def _dslPIDPositionControl(self, control_timestep, cur_pos, cur_quat_rpy, cur_vel, target_pos, target_rpy, target_vel):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat_rpy)).reshape(3,3)
        pos_err = target_pos - cur_pos
        vel_err = target_vel - cur_vel 
        self.integral_pos_e = self.integral_pos_e + pos_err*control_timestep
        self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.); self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)
        target_thrust = np.multiply(self.P_COEFF_FOR, pos_err) + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) + np.multiply(self.D_COEFF_FOR, vel_err) + np.array([0,0,self.GRAVITY])
        scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
        base_thrust = (math.sqrt(scalar_thrust / (4*self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        target_z_ax = target_thrust / np.linalg.norm(target_thrust)
        target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
        target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
        target_euler = (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)
        if np.any(np.abs(target_euler) > math.pi): print("\n[ERROR] ctrl it:", self.control_counter, "in _dslPositionControl(), values outside range [-pi,pi]")
        return base_thrust, target_euler, pos_err

    ####################################################################################################
    #### DSL's CF2.x PID attitude control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         timestep at which control is computed ######################
    #### - cur_quat_rpy (4-by-1 array)      current orientation as a quaternion ########################
    #### - base_thrust (Float)              desired thrust along the drone z-axis ######################
    #### - target_euler (3-by-1 array)      computed target euler angles ###############################
    #### - cur_ang_vel (3-by-1 array)       current angular velocity ###################################
    #### - target_ang_vel (3-by-1 array)    desired angular velocity ###################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4-by-1 array)               RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _dslPIDAttitudeControl(self, control_timestep, cur_quat_rpy, base_thrust, target_euler, cur_ang_vel, target_ang_vel):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat_rpy)).reshape(3,3)
        target_quat = (Rotation.from_euler('XYZ', target_euler, degrees=False)).as_quat()
        w,x,y,z = target_quat
        target_rotation = (Rotation.from_quat([w,x,y,z])).as_matrix()
        rot_matrix_err = np.dot((target_rotation.transpose()),cur_rotation) - np.dot(cur_rotation.transpose(),target_rotation)        
        rot_err = np.array([rot_matrix_err[2,1], rot_matrix_err[0,2], rot_matrix_err[1,0]])
        ang_vel_err = target_ang_vel - cur_ang_vel
        self.integral_rpy_e = self.integral_rpy_e - rot_err*control_timestep
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.); self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
        target_torques = - np.multiply(self.P_COEFF_TOR, rot_err) + np.multiply(self.D_COEFF_TOR, ang_vel_err) + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
        target_torques = np.clip(target_torques, -3200, 3200)
        motor_pwm = base_thrust + np.dot(self.MIXER_MATRIX, target_torques)
        motor_pwm = np.clip(motor_pwm, self.MIN_PWM, self.MAX_PWM)
        return self.PWM2RPM_SCALE*motor_pwm + self.PWM2RPM_CONST

    ####################################################################################################
    #### Non-negative Least Squares (NNLS) derivation of RPM from thrust and torques  ##################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - thrust (Float)                   desired thrust along the local z-axis ######################
    #### - x_torque (Float)                 desired x-axis torque ######################################
    #### - y_torque (Float)                 desired y-axis torque ######################################
    #### - z_torque (Float)                 desired z-axis torque ######################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4-by-1 array)               RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _physicsToRPM(self, thrust, x_torque, y_torque, z_torque):
        new_line = True
        if thrust < 0 or thrust > self.MAX_THRUST:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it:", self.control_counter, "in _physicsToRPM(), unfeasible THRUST {:.2f} outside range [0, {:.2f}]".format(thrust, self.MAX_THRUST))
        if np.abs(x_torque) > self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it:", self.control_counter, "in _physicsToRPM(), unfeasible ROLL torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(y_torque) > self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it:", self.control_counter, "in _physicsToRPM(), unfeasible PITCH torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(z_torque) > self.MAX_Z_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it:", self.control_counter, "in _physicsToRPM(), unfeasible YAW torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -self.MAX_Z_TORQUE, self.MAX_Z_TORQUE))
        B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), self.B_COEFF)
        sq_rpm = np.dot(self.INV_A, B)
        ####################################################################################################
        #### Use NNLS if any of the desired angular velocities is negative #################################
        ####################################################################################################
        if np.min(sq_rpm) < 0:
            sol, res = nnls(self.A, B, maxiter=3*self.A.shape[1])
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it:", self.control_counter, "in _physicsToRPM(), unfeasible SQ. ROTOR SPEEDS: using NNLS")
            print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                    "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
            print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                    "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                    "\t\tResidual: {:.2f}".format(res) )
            sq_rpm = sol
        return np.sqrt(sq_rpm)

