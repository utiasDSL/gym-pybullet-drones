import math
import numpy as np
import pybullet as p
from enum import Enum
from scipy.optimize import nnls
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.Aviary import DroneModel, Aviary


######################################################################################################################################################
#### Control types enumeration #######################################################################################################################
######################################################################################################################################################
class ControlType(Enum):
    PID = 0                  # PID control
    CUSTOM = 1               # Placeholder for a custom control implementation


######################################################################################################################################################
#### Control class ###################################################################################################################################
######################################################################################################################################################
class Control(object):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (Aviary)                     simulation environment #####################################
    #### - control_type (ControlType)       type of controller #########################################
    ####################################################################################################
    def __init__(self, env: Aviary, control_type: ControlType=ControlType.PID):
        #### Set general use constants #####################################################################
        self.DRONE_MODEL = env.DRONE_MODEL; self.GRAVITY = env.GRAVITY; self.KF = env.KF; self.KT = env.KM
        self.MAX_THRUST = env.MAX_THRUST; self.MAX_XY_TORQUE = env.MAX_XY_TORQUE; self.MAX_Z_TORQUE = env.MAX_Z_TORQUE
        self.CONTROLTYPE = control_type
        #### Set PID, model-specific constants #############################################################
        if self.CONTROLTYPE==ControlType.PID:
            if self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P:
                self.P_COEFF_FOR = np.array([.4, .4, 1.25]); self.I_COEFF_FOR = np.array([.05, .05, .05]); self.D_COEFF_FOR = np.array([.2, .2, .5]) 
                self.P_COEFF_TOR = np.array([70000., 70000., 60000.]); self.I_COEFF_TOR = np.array([.0, .0, 500.]); self.D_COEFF_TOR = np.array([20000., 20000., 12000.])
                self.PWM2RPM_SCALE = 0.2685; self.PWM2RPM_CONST = 4070.3; self.MIN_PWM = 20000; self.MAX_PWM = 65535
                if self.DRONE_MODEL==DroneModel.CF2X: self.MIXER_MATRIX = np.array([ [.5, -.5,  -1], [.5, .5, 1], [-.5,  .5,  -1], [-.5, -.5, 1] ]) 
                elif self.DRONE_MODEL==DroneModel.CF2P: self.MIXER_MATRIX = np.array([ [0, -1,  -1], [+1, 0, 1], [0,  1,  -1], [-1, 0, 1] ])
            elif self.DRONE_MODEL==DroneModel.HB:
                self.P_COEFF_FOR = np.array([.1, .1, .2]); self.I_COEFF_FOR = np.array([.0001, .0001, .0001]); self.D_COEFF_FOR = np.array([.3, .3, .4])
                self.P_COEFF_TOR = np.array([.3, .3, .05]); self.I_COEFF_TOR = np.array([.0001, .0001, .0001]); self.D_COEFF_TOR = np.array([.3, .3, .5])
                self.MAX_ROLL_PITCH = np.pi/6
                self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ]); self.INV_A = np.linalg.inv(self.A)
                self.B_COEFF = np.array([1/env.KF, 1/(env.KF*env.L), 1/(env.KF*env.L), 1/env.KM]) 
        #### Placeholder for alternative control implementations ###########################################
        elif self.CONTROLTYPE==ControlType.CUSTOM: print("[WARNING] in Control.__init__(), not yet implemented ControllerType")
        #### Invalid control choice ########################################################################
        else: print("[ERROR] in Control.__init__(), unknown ControllerType")
        self.reset()

    ####################################################################################################
    #### Reset the controller ##########################################################################
    ####################################################################################################
    def reset(self):
        self.control_counter = 0
        #### Initialized PID control variables #############################################################
        if self.CONTROLTYPE==ControlType.PID: self.last_pos_e = np.zeros(3); self.integral_pos_e = np.zeros(3); self.last_rpy_e = np.zeros(3); self.integral_rpy_e = np.zeros(3)

    ####################################################################################################
    #### Compute the control action for a single drone #################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         timestep at which control is computed ######################
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
    def computeControl(self, control_timestep, cur_pos, cur_quat, cur_vel, cur_ang_vel, \
                        target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        self.control_counter += 1
        #### PID controllers ###############################################################################
        if self.CONTROLTYPE==ControlType.PID:
            #### PID control for Bitcraze's Crazyflie 2.x ######################################################
            if self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P:  
                thrust, computed_target_rpy, pos_e = self._dslPIDPositionControl(control_timestep, cur_pos, cur_quat, cur_vel, target_pos, target_rpy, target_vel)
                rpm = self._dslPIDAttitudeControl(control_timestep, thrust, cur_quat, cur_ang_vel, computed_target_rpy, target_ang_vel)
            #### XYZ PID control for DroneModel.HB, based on https://github.com/prfraanje/quadcopter_sim #######
            elif self.DRONE_MODEL==DroneModel.HB:
                if target_rpy[2]!=0: print("\n[WARNING] ctrl it", self.control_counter, "in Control.computeControl(), desired yaw={:.0f}deg but locked to 0. for DroneModel.HB".format(target_rpy[2]*(180/np.pi)))
                thrust, computed_target_rpy, pos_e = self._simplePIDPositionControl(control_timestep, cur_pos, cur_quat, target_pos)
                rpm = self._simplePIDAttitudeControl(control_timestep, thrust, cur_quat, computed_target_rpy)
            #### Return RPM and position, yaw errors ###########################################################
            cur_rpy = p.getEulerFromQuaternion(cur_quat)
            return rpm, pos_e, computed_target_rpy[2]-cur_rpy[2]
        #### Placeholder for alternative control implementations ###########################################
        elif self.CONTROLTYPE==ControlType.CUSTOM:
            print("[WARNING] ctrl it", self.control_counter, "in Control.computeControl(), not yet implemented ControllerType")
            return [0,0,0,0], 0., 0.
        #### Invalid control choice ########################################################################
        else: print("[ERROR] ctrl it", self.control_counter, "in Control.computeControl(), unknown ControllerType")

    ####################################################################################################
    #### Wrapper function to compute the control action from obs as returned by Aviary.step() ##########
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         timestep at which control is computed ######################
    #### - state ((20,1) array)             current state of the drone #################################
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
    def computeControlFromState(self, control_timestep, state, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        return self.computeControl(control_timestep=control_timestep, \
                                            cur_pos=state[0:3], cur_quat=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16], \
                                            target_pos=target_pos, target_vel=target_vel, target_ang_vel=target_ang_vel)

    ####################################################################################################
    #### DSL's CF2.x PID position control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         timestep (inverse of the freq.) at which control is computed
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
    #### - control_timestep (float)         timestep at which control is computed ######################
    #### - thrust (float)                   desired thrust along the drone z-axis ######################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - cur_ang_vel ((3,1) array)        current angular velocity ###################################
    #### - target_euler ((3,1) array)       computed target euler angles ###############################
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
    #### Generic PID position control (with yaw locked to 0.) ##########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         timestep at which control is computed ######################
    #### - cur_pos ((3,1) array)            current position ###########################################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - target_pos ((3,1) array)         desired position ###########################################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - thrust (float)                   thrust along the drone z-axis ##############################
    #### - target_rpy ((3,1) array)         computed target roll, pitch, and yaw #######################
    #### - yaw_e (float)                    current yaw error ##########################################
    ####################################################################################################
    def _simplePIDPositionControl(self, control_timestep, cur_pos, cur_quat, target_pos):
        pos_e = target_pos - np.array(cur_pos).reshape(3)
        d_pos_e = (pos_e - self.last_pos_e) / control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        #### PID target thrust #############################################################################
        target_force = np.array([0,0,self.GRAVITY]) + np.multiply(self.P_COEFF_FOR,pos_e) + np.multiply(self.I_COEFF_FOR,self.integral_pos_e) + np.multiply(self.D_COEFF_FOR,d_pos_e)
        target_rpy = np.zeros(3)
        sign_z =  np.sign(target_force[2])
        if sign_z==0: sign_z = 1 
        #### Target rotation ###############################################################################
        target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
        target_rpy[1] = np.arctan2(sign_z*target_force[0], sign_z*target_force[2])
        target_rpy[2] = 0.
        target_rpy[0] = np.clip(target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        target_rpy[1] = np.clip(target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3,3)
        thrust = np.dot(cur_rotation, target_force)
        return thrust[2], target_rpy, pos_e

    ####################################################################################################
    #### Generic PID attitude control (with yaw locked to 0.) ##########################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (float)         timestep at which control is computed ######################
    #### - thrust (float)                   desired thrust along the drone z-axis ######################
    #### - cur_quat ((4,1) array)           current orientation as a quaternion ########################
    #### - target_rpy ((3,1) array)         computed target roll, pitch, and yaw #######################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
    ####################################################################################################
    def _simplePIDAttitudeControl(self, control_timestep, thrust, cur_quat, target_rpy):
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        rpy_e = target_rpy - np.array(cur_rpy).reshape(3,) 
        if rpy_e[2]>np.pi: rpy_e[2] = rpy_e[2] - 2*np.pi
        if rpy_e[2]<-np.pi: rpy_e[2] = rpy_e[2] + 2*np.pi
        d_rpy_e = (rpy_e - self.last_rpy_e) / control_timestep
        self.last_rpy_e = rpy_e
        self.integral_rpy_e = self.integral_rpy_e + rpy_e*control_timestep
        #### PID target torques ############################################################################
        target_torques = np.multiply(self.P_COEFF_TOR,rpy_e) + np.multiply(self.I_COEFF_TOR,self.integral_rpy_e) + np.multiply(self.D_COEFF_TOR,d_rpy_e)
        return self._nnlsRPM(thrust, target_torques[0], target_torques[1], target_torques[2])

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
        new_line = True
        #### Check the feasibility of thrust and torques ###################################################
        if thrust<0 or thrust>self.MAX_THRUST:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it", self.control_counter, "in Control._nnlsRPM(), unfeasible thrust {:.2f} outside range [0, {:.2f}]".format(thrust, self.MAX_THRUST))
        if np.abs(x_torque)>self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it", self.control_counter, "in Control._nnlsRPM(), unfeasible roll torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(y_torque)>self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it", self.control_counter, "in Control._nnlsRPM(), unfeasible pitch torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(z_torque)>self.MAX_Z_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it", self.control_counter, "in Control._nnlsRPM(), unfeasible yaw torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -self.MAX_Z_TORQUE, self.MAX_Z_TORQUE))
        B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), self.B_COEFF)
        sq_rpm = np.dot(self.INV_A, B)
        #### Use NNLS if any of the desired angular velocities is negative #################################
        if np.min(sq_rpm)<0:
            sol, res = nnls(self.A, B, maxiter=3*self.A.shape[1])
            if new_line: print(); new_line = False
            print("[WARNING] ctrl it", self.control_counter, "in Control._nnlsRPM(), unfeasible squared rotor speeds, using NNLS")
            print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                    "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
            print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                    "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                    "\t\tResidual: {:.2f}".format(res) )
            sq_rpm = sol
        return np.sqrt(sq_rpm)

    ####################################################################################################
    #### Work in progress ##############################################################################
    ####################################################################################################
    def one23DInterface(thrust, SCALE: float=0.2685, CONST: float=4070.3, CT: float=3.1582e-10, MI: float=20000.0, MI: float=65535.0):
        DIM = len(np.array(thrust)); pwm = np.clip((np.sqrt(np.array(thrust)/(CT*(4/DIM)))-CONST)/SCALE, MI, MA)
        if DIM in [1, 4]: return np.repeat(pwm, 4/DIM)
        elif DIM==2: return np.hstack([pwm, np.flip(pwm)])
        else: print("[ERROR] in one23DInterface()"); exit()

