import cvxpy as cp
import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel

class MPC_control(BaseControl):

    def __init__(self, drone_model: DroneModel, g: float=9.8):
        super().__init__(drone_model=drone_model, g=g)

        self.T = 10  # Horizon length (steps)
        self.dt = 1/48 # Control timestep (match your simulation)
        
        #Get Physics Constants from URDF (via BaseControl)
        self.L = self._getURDFParameter('arm') #arm o lenght?
        self.m = self._getURDFParameter('m')
        self.Ixx = self._getURDFParameter('ixx')
        self.Iyy = self._getURDFParameter('iyy')
        self.Izz = self._getURDFParameter('izz')
        self.MAX_speed = self._getURDFParameter('max_speed_kmh')
        self.THRUST2WEIGHT_RATIO = self._getURDFParameter('thrust2weight')
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
       # self.PROP_RADIUS = self._getURDFParameter('prop_radius') # Returns None often
        self.PROP_RADIUS = 0.0231  # 2.31 cm radius
        
        # self.GND_EFF_COEFF = self._getURDFParameter('gnd_eff_coeff') # Returns None often
        self.GND_EFF_COEFF = 11.36859 

        self.GRAVITY = g * self.m
        # --- FIX 1: Define Gravity Vector for Dynamics ---
        # State structure: [x, y, z, vx, vy, vz, ...]
        # vz is at index 5.
        self.g_vector = np.zeros(12)
        self.g_vector[5] = -g * self.dt # Applies -9.8 m/s^2 * dt to vertical velocity

        self.HOVER_RPM = np.sqrt(self.GRAVITY / (4*self.KF))
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY) / (4*self.KF))
        self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MAX_XY_TORQUE = (2*self.L*self.KF*self.MAX_RPM**2)/np.sqrt(2)
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2)
        elif self.DRONE_MODEL == DroneModel.RACE:
            self.MAX_XY_TORQUE = (2*self.L*self.KF*self.MAX_RPM**2)/np.sqrt(2)
        self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)
        self.GND_EFF_H_CLIP = 0.25 * self.PROP_RADIUS * np.sqrt((15 * self.MAX_RPM**2 * self.KF * self.GND_EFF_COEFF) / self.MAX_THRUST)
        
        #state-space Matrices
        self.A_c = np.zeros((12, 12))
        self.B_c = np.zeros((12, 4))

        # --- Construct A Matrix (System Dynamics) ---
        
        # kinematics: Position derivative = Velocity
        # d(x)/dt = vx, d(y)/dt = vy, d(z)/dt = vz
        self.A_c[0, 3] = 1.0
        self.A_c[1, 4] = 1.0
        self.A_c[2, 5] = 1.0

        # Dynamics: Linear Acceleration (Linearized Gravity Tilt)
        # At hover, Thrust T ~= mg. Small angle approx: sin(theta)~=theta, sin(phi)~=phi
        # d(vx)/dt = g * theta
        self.A_c[3, 7] = g 
        # d(vy)/dt = -g * phi
        self.A_c[4, 6] = -g
        
        # Kinematics: Orientation derivative = Angular Velocity
        # For small angles, d(phi)/dt ~= p, d(theta)/dt ~= q, d(psi)/dt ~= r
        self.A_c[6, 9] = 1.0
        self.A_c[7, 10] = 1.0
        self.A_c[8, 11] = 1.0

        # --- Construct B Matrix (Control Inputs) ---
        
        # Thrust Input (u[0]) affects vertical acceleration
        # d(vz)/dt = (1/m) * Thrust
        self.B_c[5, 0] = 1.0 / self.m

        # Torque Inputs (u[1], u[2], u[3]) affect angular acceleration
        # d(p)/dt = (1/Ixx) * Tx
        self.B_c[9, 1] = 1.0 / self.Ixx
        
        # d(q)/dt = (1/Iyy) * Ty
        self.B_c[10, 2] = 1.0 / self.Iyy
        
        # d(r)/dt = (1/Izz) * Tz
        self.B_c[11, 3] = 1.0 / self.Izz
            
        # Euler discretization
        self.A = np.eye(12) + self.A_c * self.dt
        self.B = self.B_c * self.dt   

        max_angle = np.deg2rad(25)

        # Limits on state and input
        self.x_min = np.array([-10, -10,  0, 
                                -8,  -8, -3, 
                                -max_angle, -max_angle, -np.inf,
                                -7, -7, -self.MAX_RPM])
        self.x_max = np.array([10, 10, 10,
                               8, 8, 3,
                               max_angle, max_angle, np.inf,
                                7, 7, self.MAX_RPM])
        
        self.u_min = np.array([0, -self.MAX_XY_TORQUE, -self.MAX_XY_TORQUE, -self.MAX_Z_TORQUE])   #forse 1*??
        self.u_max = np.array([self.MAX_THRUST, self.MAX_XY_TORQUE, self.MAX_XY_TORQUE, self.MAX_Z_TORQUE])
        
        if self.DRONE_MODEL == DroneModel.CF2X or self.DRONE_MODEL == DroneModel.RACE:
            self.MIXER_MATRIX = np.array([ 
                                    [-.5, -.5, -1],
                                    [-.5,  .5,  1],
                                    [.5, .5, -1],
                                    [.5, -.5,  1]
                                    ])
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MIXER_MATRIX = np.array([
                                    [0, -1,  -1],
                                    [+1, 0, 1],
                                    [0,  1,  -1],
                                    [-1, 0, 1]
                                    ])
            
        self.x_init_param = cp.Parameter(12)
        self.x_target_param = cp.Parameter(12)
        self._setup_mpc_problem()
            
    def next_x(self):
        return self.A.dot(self.x) + self.B.transpose().dot(self.u)
        
    def _setup_mpc_problem(self):
        # Define CVXPY variables, cost functions, and constraints here
        # so you don't rebuild the problem every single step (slow!)

        # DA CAMBIAREEE
        weight_input = 10*np.eye(4)    # Weight on the input
        weight_tracking = 600*np.eye(12) # Weight on the tracking state

        cost = 0.
        constraints = []
        
        # Create the optimization variables
        self.x = cp.Variable((12, self.T + 1)) # cp.Variable((dim_1, dim_2))
        self.u = cp.Variable((4, self.T))
    
        # For each stage in k = 0, ..., N-1
        for k in range(self.T):
            
            # EXERCISE: Implement the cost components and/or constraints that need to be satisfied for each step, here    
            constraints += [self.x[0, k] >= self.x_min[0], self.x[0, k] <= self.x_max[0]]  
            constraints += [self.x[1, k] >= self.x_min[1], self.x[1, k] <= self.x_max[1]]  
            constraints += [self.x[2, k] >= self.x_min[2], self.x[2, k] <= self.x_max[2]]  
            constraints += [self.x[3, k] >= self.x_min[3], self.x[3, k] <= self.x_max[3]]  
            constraints += [self.x[4, k] >= self.x_min[4], self.x[4, k] <= self.x_max[4]]  
            constraints += [self.x[5, k] >= self.x_min[5], self.x[5, k] <= self.x_max[5]]  
            constraints += [self.x[6, k] >= self.x_min[6], self.x[6, k] <= self.x_max[6]]  
            constraints += [self.x[7, k] >= self.x_min[7], self.x[7, k] <= self.x_max[7]]  
            constraints += [self.x[8, k] >= self.x_min[8], self.x[8, k] <= self.x_max[8]]  
            constraints += [self.x[9, k] >= self.x_min[9], self.x[9, k] <= self.x_max[9]]  
            constraints += [self.x[10, k] >= self.x_min[10], self.x[10, k] <= self.x_max[10]]  
            constraints += [self.x[11, k] >= self.x_min[11], self.x[11, k] <= self.x_max[11]]    

            if k < self.T-1:
                constraints += [self.x[:, k+1] == self.A @ self.x[:, k] + self.B @ self.u[:, k] + self.g_vector]
                
                constraints += [self.u[0, k] >= self.u_min[0], self.u[0, k] <= self.u_max[0]]
                constraints += [self.u[1, k] >= self.u_min[1], self.u[1, k] <= self.u_max[1]]
                constraints += [self.u[2, k] >= self.u_min[2], self.u[2, k] <= self.u_max[2]]
                constraints += [self.u[3, k] >= self.u_min[3], self.u[3, k] <= self.u_max[3]]
                # Cost function components
                x_err = self.x[:, k] - self.x_target_param
                cost += cp.quad_form(x_err, weight_tracking) + cp.quad_form(self.u[:, k], weight_input)

                #cost += weight_tracking*(self.x_target_param[0] - self.x[0, k])**2 + weight_tracking*(self.x_target_param[1] - self.x[1, k])**2 + \
                #        weight_tracking*(self.x_target_param[2] - self.x[2, k])**2 + cp.quad_form(self.u[:, k], weight_input)
        
        
        constraints += [self.x[0, 0] == self.x_init_param[0]]
        constraints += [self.x[1, 0] == self.x_init_param[1]]
        constraints += [self.x[2, 0] == self.x_init_param[2]]
        constraints += [self.x[3, 0] == self.x_init_param[3]]
        constraints += [self.x[4, 0] == self.x_init_param[4]]
        constraints += [self.x[5, 0] == self.x_init_param[5]]
        constraints += [self.x[6, 0] == self.x_init_param[6]]
        constraints += [self.x[7, 0] == self.x_init_param[7]]
        constraints += [self.x[8, 0] == self.x_init_param[8]]
        constraints += [self.x[9, 0] == self.x_init_param[9]]
        constraints += [self.x[10, 0] == self.x_init_param[10]]
        constraints += [self.x[11, 0] == self.x_init_param[11]]
        #constraints += [x[0, N] == x_target]
        
        # Solves the problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

        # We return the MPC input and the next state (and also the plan for visualization)

    def computeControl(self, control_timestep,cur_pos, cur_quat, cur_vel, cur_ang_vel, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_rpy_rates=np.zeros(3)):
        
        '''
        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.
        ndarray
            (3,1)-shaped array of floats containing the current XYZ position error.
        float
            The current yaw error.
        '''
        
        # 1. Construct State Vectors
        # Current State [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        #roll  = cur_rpy[0]  # Rotation around X-axis (Bank)
        #pitch = cur_rpy[1]  # Rotation around Y-axis (Elevation)
        yaw   = cur_rpy[2]
        error_yaw = np.arccos(np.cos(yaw)* np.cos(target_rpy[2]) + np.sin(yaw)*np.sin(target_rpy[2]))
        x_current = np.hstack([cur_pos, cur_vel, cur_rpy, cur_ang_vel])
        
        # Target State (Construct the 12-vector from arguments)
        # Note: If target_rpy/vel are not provided, they default to zeros (Hover)
        x_target = np.hstack([target_pos, target_vel, target_rpy, target_rpy_rates])
        
        # 2. Update Parameters
        self.x_init_param.value = x_current
        self.x_target_param.value = x_target
        
        # 3. Solve
        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True)
        except:
            print("Solver Failed")
        
        # 4. Extract Optimal Input
        if self.u.value is None:
            print("Infeasible!")
            self.optimal_u = np.zeros(4) # Fail-safe
        else:
            self.optimal_u = self.u[:, 0].value
            print("Optimal u:", self.optimal_u)

        # 5. Convert Forces/Torques to RPM
        # Your u is [Thrust, Tx, Ty, Tz]. Convert to RPMs using your model's mixer.
        # This part depends on if you use the inverse mixer logic or a simple sqrt map.
        # Below is a simplified example assuming generic force->rpm logic:
        
        self.thrust = self.optimal_u[0]
        self.torques = self.optimal_u[1:]
        
        # You need to implement the specific mixer (CF2X/CF2P) here to get 4 RPMs
        # (Copying the logic from DSLPIDControl is a good idea)
        rpms = self._mix_forces_to_rpm(self.thrust, self.torques)
        
        return rpms, np.linalg.norm(target_pos - cur_pos), error_yaw

    # def _mix_forces_to_rpm(self):
    #     # INVERT YOUR MIXING MATRIX HERE
    #     # Placeholder for CF2X logic:
    #     # F_motors = inv(Mixer) * [thrust, tx, ty, tz]
    #     # rpm = sqrt(F_motors / KF)
    #     self.thrust = np.maximum(0, self.thrust)
    #     self.target_torques = np.hstack((self.torques[0], self.torques[1], self.torques[2]))
    #     self.target_torques = np.clip(self.target_torques, -3200, 3200)

    #     self.thrust = (math.sqrt(self.thrust / (4*self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
    #     pwm = self.thrust + np.dot(self.MIXER_MATRIX, self.target_torques)
    #     pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
    #     rpm = self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST
    #     return rpm

    def _mix_forces_to_rpm(self, thrust, torques):
        """
        Converts MPC Force (N) and Torque (Nm) directly into Motor RPMs 
        by inverting the quadratic physical relationship: Force = KF * RPM^2
        """
        t_x, t_y, t_z = torques
        
        # 1. Determine Effective Arm Length (Geometry)
        # For CF2X, the motors are at 45 degrees, so the moment arm is L / sqrt(2)
        if self.DRONE_MODEL == DroneModel.CF2X:
            L_eff = self.L / np.sqrt(2)
        else: # CF2P
            L_eff = self.L
            
        # 2. Invert the Allocation Matrix
        # We want to find the squared RPM (w^2) for each motor that satisfies the required Force/Torques.
        #
        # F_total = KF * (w0^2 + w1^2 + w2^2 + w3^2)
        # T_roll  = KF * L_eff * (w0^2 + w1^2 - w2^2 - w3^2)  <-- Check signs based on motor pos
        # T_pitch = KF * L_eff * (-w0^2 + w1^2 + w2^2 - w3^2) <-- Check signs based on motor pos
        # T_yaw   = KM * (-w0^2 + w1^2 - w2^2 + w3^2)
        
        # Pre-calculate coefficients
        k_thrust = 1.0 / (4.0 * self.KF)
        k_roll   = 1.0 / (4.0 * self.KF * L_eff)
        k_pitch  = 1.0 / (4.0 * self.KF * L_eff)
        k_yaw    = 1.0 / (4.0 * self.KM)

        # 3. Calculate Squared RPMs for each motor
        # These signs correspond to the standard PyBullet/Crazyflie motor layout:
        # Motor 0: Front Right (CCW)
        # Motor 1: Rear Right (CW)
        # Motor 2: Rear Left (CCW)
        # Motor 3: Front Left (CW)
        
        w2_0 = k_thrust*thrust - k_roll*t_x - k_pitch*t_y - k_yaw*t_z
        w2_1 = k_thrust*thrust - k_roll*t_x + k_pitch*t_y + k_yaw*t_z
        w2_2 = k_thrust*thrust + k_roll*t_x + k_pitch*t_y - k_yaw*t_z
        w2_3 = k_thrust*thrust + k_roll*t_x - k_pitch*t_y + k_yaw*t_z

        # 4. Convert to RPM
        # Clip negative values (impossible physics) and take square root
        w2_all = np.array([w2_0, w2_1, w2_2, w2_3])
        w2_all = np.maximum(w2_all, 0)
        rpms = np.sqrt(w2_all)
        
        return rpms