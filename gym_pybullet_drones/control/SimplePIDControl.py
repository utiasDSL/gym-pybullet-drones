import math
import numpy as np
import pybullet as p
from scipy.optimize import nnls

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary


class SimplePIDControl(BaseControl):

    ####################################################################################################
    #### Initialize the controller #####################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - env (BaseAviary)                 simulation environment #####################################
    ####################################################################################################
    def __init__(self, env: BaseAviary):
        super().__init__(env=env)
        if self.DRONE_MODEL!=DroneModel.HB: print("[ERROR] in SimplePIDControl.__init__(), DSLPIDControl requires DroneModel.HB"); exit()
        self.P_COEFF_FOR = np.array([.1, .1, .2]); self.I_COEFF_FOR = np.array([.0001, .0001, .0001]); self.D_COEFF_FOR = np.array([.3, .3, .4])
        self.P_COEFF_TOR = np.array([.3, .3, .05]); self.I_COEFF_TOR = np.array([.0001, .0001, .0001]); self.D_COEFF_TOR = np.array([.3, .3, .5])
        self.MAX_ROLL_PITCH = np.pi/6
        self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ]); self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([1/env.KF, 1/(env.KF*env.L), 1/(env.KF*env.L), 1/env.KM])
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
        if target_rpy[2]!=0: print("\n[WARNING] ctrl it", self.control_counter, "in SimplePIDControl.computeControl(), desired yaw={:.0f}deg but locked to 0. for DroneModel.HB".format(target_rpy[2]*(180/np.pi)))
        thrust, computed_target_rpy, pos_e = self._simplePIDPositionControl(control_timestep, cur_pos, cur_quat, target_pos)
        rpm = self._simplePIDAttitudeControl(control_timestep, thrust, cur_quat, computed_target_rpy)
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2]-cur_rpy[2]

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
    #### - control_timestep (float)         time step at which control is computed #####################
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

 