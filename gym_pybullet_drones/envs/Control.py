from gym_pybullet_drones.envs.SingleDroneEnv import SingleDroneEnv

class Control():

    def __init__(self, environment: SingleDroneEnv):
        print("init")

    def computeControl(self):
        pass










######################################################################################################################################################
#### Control #########################################################################################################################################
######################################################################################################################################################


    ####################################################################################################
    #### 2 variants of custom PID control ##############################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - control_timestep (Float)         the timestep at which control is computed ##################
    #### - cur_pos (3by1 list/array)        the current position #######################################
    #### - cur_quat_rpy (4by1 list/array)   the current orientation as a quaternion ####################
    #### - cur_vel (3by1 list/array)        the current velocity #######################################
    #### - cur_ang_vel (3by1 list/array)    the current angular velocity ###############################
    #### - target_pos (3by1 list/array)     the desired position #######################################
    #### - target_rpy (3by1 list/array)     the desired orientation as roll, pitch, yaw ################
    #### - target_vel (3by1 list/array)     the desired velocity #######################################
    #### - target_ang_vel (3by1 list/array) the desired angular velocity ###############################
    ####################################################################################################
    #### Returns #######################################################################################
    #### - rpm (4by1 list/array)            the RPM values to apply to the 4 motors ####################
    #### - pos err (3by1 list/array)        the current XYZ position error #############################
    #### - yaw err (Float)                  the current yaw error ######################################
    ####################################################################################################
    def control(self, control_timestep, cur_pos, cur_quat_rpy, cur_vel, cur_ang_vel, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_ang_vel=np.zeros(3)):
        cur_rpy = p.getEulerFromQuaternion(cur_quat_rpy)
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat_rpy)).reshape(3,3)
        ####################################################################################################
        #### XYZ PID control tuned for DroneModel.HB #######################################################
        #### based on https://github.com/prfraanje/quadcopter_sim ##########################################
        ####################################################################################################
        if self.DRONE_MODEL==DroneModel.HB:
            MAX_ROLL_PITCH = np.pi/6
            pos_e = target_pos - np.array(cur_pos).reshape(3)
            d_pos_e = (pos_e - self.last_pos_e) / control_timestep
            self.last_pos_e = pos_e
            self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
            P_COEFF_FOR = np.array([.1, .1, .2]); I_COEFF_FOR = np.array([.0001, .0001, .0001]); D_COEFF_FOR = np.array([.3, .3, .4])
            target_force = np.array([0,0,self.GRAVITY]) + np.multiply(P_COEFF_FOR,pos_e) + np.multiply(I_COEFF_FOR,self.integral_pos_e) + np.multiply(D_COEFF_FOR,d_pos_e)
            computed_target_rpy = np.zeros(3)
            sign_z =  np.sign(target_force[2])
            if sign_z == 0: sign_z = 1 
            computed_target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
            computed_target_rpy[1] = np.arctan2(sign_z*target_force[0],sign_z*target_force[2])
            computed_target_rpy[2] = 0.
            if target_rpy[2] != 0. and self.step_counter%(5*self.SIM_FREQ)==1: print("\n[WARNING] it:", self.step_counter, "in control(), desired yaw={:.0f}deg but yaw control is NOT implemented for DroneModel.HB".format(target_rpy[2]*self.RAD2DEG))
            computed_target_rpy[0] = np.clip(computed_target_rpy[0], -MAX_ROLL_PITCH, MAX_ROLL_PITCH)
            computed_target_rpy[1] = np.clip(computed_target_rpy[1], -MAX_ROLL_PITCH, MAX_ROLL_PITCH)
            rpy_e = computed_target_rpy - np.array(cur_rpy).reshape(3,) 
            if rpy_e[2] > np.pi: rpy_e[2] = rpy_e[2] - 2*np.pi
            if rpy_e[2] < -np.pi: rpy_e[2] = rpy_e[2] + 2*np.pi
            d_rpy_e = (rpy_e - self.last_rpy_e) / control_timestep
            self.last_rpy_e = rpy_e
            self.integral_rpy_e = self.integral_rpy_e + rpy_e*control_timestep
            P_COEFF_TOR = np.array([.3, .3, .05]); I_COEFF_TOR = np.array([.0001, .0001, .0001]); D_COEFF_TOR = np.array([.3, .3, .5])
            target_torques = np.multiply(P_COEFF_TOR,rpy_e) + np.multiply(I_COEFF_TOR,self.integral_rpy_e) + np.multiply(D_COEFF_TOR,d_rpy_e)
            target_torques = np.dot(cur_rotation, target_torques)
            target_force = np.dot(cur_rotation, target_force)
            rpm = self._physicsToRPM(target_force[2], target_torques[0], target_torques[1], target_torques[2])
            return rpm, pos_e, 0.-cur_rpy[2]
        ####################################################################################################
        #### PID control tuned for Bitcraze's Crazyflie 2.0 ################################################
        ####################################################################################################
        elif self.DRONE_MODEL==DroneModel.CF2X or self.DRONE_MODEL==DroneModel.CF2P:
            if self.DRONE_MODEL==DroneModel.CF2X: MIXER_MATRIX = np.array([ [.5, -.5,  1], [.5, .5, -1], [-.5,  .5,  1], [-.5, -.5, -1] ]) 
            if self.DRONE_MODEL==DroneModel.CF2P: MIXER_MATRIX = np.array([ [0, -1,  1], [+1, 0, -1], [0,  1,  1], [-1, 0, -1] ])
            PWM2RPM_SCALE = 0.2685; PWM2RPM_CONST = 4070.3
            pos_err = target_pos - cur_pos
            vel_err = target_vel - cur_vel 
            self.integral_pos_e = self.integral_pos_e + pos_err*control_timestep
            self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.); self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)
            P_COEFF_FOR = np.array([.4, .4, 1.25]); I_COEFF_FOR = np.array([.05, .05, .05]); D_COEFF_FOR = np.array([.2, .2, .5]) 
            target_thrust = np.multiply(P_COEFF_FOR, pos_err) + np.multiply(I_COEFF_FOR, self.integral_pos_e) + np.multiply(D_COEFF_FOR, vel_err) + np.array([0,0,self.GRAVITY])
            scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
            base_thrust = (math.sqrt(scalar_thrust / (4*self.KF)) - PWM2RPM_CONST) / PWM2RPM_SCALE
            target_z_ax = target_thrust / np.linalg.norm(target_thrust)
            target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
            target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
            target_x_ax = np.cross(target_y_ax, target_z_ax)
            target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
            target_euler = (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)
            if np.any(np.abs(target_euler) > math.pi): print("\n[ERROR] it:", self.step_counter, "in target_euler, values outside range [-pi,pi]")
            target_quat = (Rotation.from_euler('XYZ', target_euler, degrees=False)).as_quat()
            w,x,y,z = target_quat
            target_rotation = (Rotation.from_quat([w,x,y,z])).as_matrix()
            rot_matrix_err = np.dot((target_rotation.transpose()),cur_rotation) - np.dot(cur_rotation.transpose(),target_rotation)        
            rot_err = np.array([rot_matrix_err[2,1], rot_matrix_err[0,2], rot_matrix_err[1,0]])
            ang_vel_err = target_ang_vel - cur_ang_vel
            self.integral_rpy_e = self.integral_rpy_e - rot_err*control_timestep
            self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.); self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
            P_COEFF_TOR = np.array([70000., 70000., 60000.]); D_COEFF_TOR = np.array([20000., 20000., 12000.]); I_COEFF_TOR = np.array([.0, .0, 500.]) # ; D_COEFF_TOR = np.array([100., 100., 0.])
            target_torques = - np.multiply(P_COEFF_TOR, rot_err) + np.multiply(D_COEFF_TOR, ang_vel_err) + np.multiply(I_COEFF_TOR, self.integral_rpy_e)
            target_torques = np.clip(target_torques, -3200, 3200)
            motor_pwm = base_thrust + np.dot(MIXER_MATRIX, target_torques)
            motor_pwm = np.clip(motor_pwm, 20000, 65535)
            rpm = PWM2RPM_SCALE*motor_pwm + PWM2RPM_CONST
            return rpm, pos_err, target_euler[2]-cur_rpy[2]

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
    #### - rpm (4by1 list/array)            the RPM values to apply to the 4 motors ####################
    ####################################################################################################
    def _physicsToRPM(self, thrust, x_torque, y_torque, z_torque):
        new_line = True
        if thrust < 0 or thrust > self.MAX_THRUST:
            if new_line: print(); new_line = False
            print("[WARNING] it:", self.step_counter, "in _physicsToRPM(), unfeasible THRUST {:.2f} outside range [0, {:.2f}]".format(thrust, self.MAX_THRUST))
        if np.abs(x_torque) > self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] it:", self.step_counter, "in _physicsToRPM(), unfeasible ROLL torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(y_torque) > self.MAX_XY_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] it:", self.step_counter, "in _physicsToRPM(), unfeasible PITCH torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -self.MAX_XY_TORQUE, self.MAX_XY_TORQUE))
        if np.abs(z_torque) > self.MAX_Z_TORQUE:
            if new_line: print(); new_line = False
            print("[WARNING] it:", self.step_counter, "in _physicsToRPM(), unfeasible YAW torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -self.MAX_Z_TORQUE, self.MAX_Z_TORQUE))
        B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), self.B_COEFF)
        sq_rpm = np.dot(self.INV_A, B)
        if np.min(sq_rpm) < 0:
            sol, res = nnls(self.A, B, maxiter=3*self.A.shape[1])
            if new_line: print(); new_line = False
            print("[WARNING] it:", self.step_counter, "in _physicsToRPM(), unfeasible SQ. ROTOR SPEEDS: using NNLS")
            print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                    "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
            print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                    "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                    "\t\tResidual: {:.2f}".format(res) )
            sq_rpm = sol
        return np.sqrt(sq_rpm)


        

