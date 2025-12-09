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
        self.dt = 1/240 # Control timestep (match your simulation)
        
        #Get Physics Constants from URDF (via BaseControl)
        self.m = self._getURDFParameter('m')
        self.Ixx = self._getURDFParameter('ixx')
        self.Iyy = self._getURDFParameter('iyy')
        self.Izz = self._getURDFParameter('izz')
        self.MAX_speed = self._getURDFParameter('max_speed_kmh')
        self.THRUST2WEIGHT_RATIO = self._getURDFParameter('thrust2weight')

        self.GRAVITY = g*self.m
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
        
        #setup the Optimization Problem (Symbolic)
        self._setup_mpc_problem()
            
    def next_x(self, x, u):
        return self.A.dot(x) + self.B.transpose().dot(u)
        
    def _setup_mpc_problem(self, N, x_init, x_target):
        # Define CVXPY variables, cost functions, and constraints here
        # so you don't rebuild the problem every single step (slow!)

        # DA CAMBIAREEE
        weight_input = 0.2*np.eye(4)    # Weight on the input
        weight_tracking = 1.1*np.eye(12) # Weight on the tracking state

        cost = 0.
        constraints = []
        
        # Create the optimization variables
        x = cp.Variable((12, N + 1)) # cp.Variable((dim_1, dim_2))
        u = cp.Variable((4, N))
    
        # For each stage in k = 0, ..., N-1
        for k in range(N):
            
            # EXERCISE: Implement the cost components and/or constraints that need to be satisfied for each step, here    
            constraints += [x[0, k] >= self.x_min[0], x[0, k] <= self.x_max[0]]  
            constraints += [x[1, k] >= self.x_min[1], x[1, k] <= self.x_max[1]]  
            constraints += [x[2, k] >= self.x_min[2], x[2, k] <= self.x_max[2]]  
            constraints += [x[3, k] >= self.x_min[3], x[3, k] <= self.x_max[3]]  
            constraints += [x[4, k] >= self.x_min[4], x[4, k] <= self.x_max[4]]  
            constraints += [x[5, k] >= self.x_min[5], x[5, k] <= self.x_max[5]]  
            constraints += [x[6, k] >= self.x_min[6], x[6, k] <= self.x_max[6]]  
            constraints += [x[7, k] >= self.x_min[7], x[7, k] <= self.x_max[7]]  
            constraints += [x[8, k] >= self.x_min[8], x[8, k] <= self.x_max[8]]  
            constraints += [x[9, k] >= self.x_min[9], x[9, k] <= self.x_max[9]]  
            constraints += [x[10, k] >= self.x_min[10], x[10, k] <= self.x_max[10]]  
            constraints += [x[11, k] >= self.x_min[11], x[11, k] <= self.x_max[11]]    

            if k < N-1:
                constraints += [x[:, k+1] == self.A @ x[:, k] + self.B @ u[:, k]]

                constraints += [u[0, k] >= self.u_min[0], u[0, k] <= self.u_max[0]]
                constraints += [u[1, k] >= self.u_min[1], u[1, k] <= self.u_max[1]]
                constraints += [u[2, k] >= self.u_min[2], u[2, k] <= self.u_max[2]]
                constraints += [u[3, k] >= self.u_min[3], u[3, k] <= self.u_max[3]]

                cost += weight_tracking*(x_target[0] - x[0, k])**2 + weight_tracking*(x_target[1] - x[1, k])**2 + weight_tracking*(x_target[2] - x[2, k])**2 + cp.quad_form(u[:, k], weight_input)
        
        # EXERCISE: Implement the cost components and/or constraints that need to be added once, here
        constraints += [x[0, 0] == x_init[0]]
        constraints += [x[1, 0] == x_init[1]]
        constraints += [x[2, 0] == x_init[2]]
        constraints += [x[3, 0] == x_init[3]]
        constraints += [x[4, 0] == x_init[4]]
        constraints += [x[5, 0] == x_init[5]]
        constraints += [x[6, 0] == x_init[6]]
        constraints += [x[7, 0] == x_init[7]]
        constraints += [x[8, 0] == x_init[8]]
        constraints += [x[9, 0] == x_init[9]]
        constraints += [x[10, 0] == x_init[10]]
        constraints += [x[11, 0] == x_init[11]]
        #constraints += [x[0, N] == x_target]
        
        # Solves the problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

        # We return the MPC input and the next state (and also the plan for visualization)

    def computeControl(self, control_timestep, cur_pos, cur_quat, cur_vel, cur_ang_vel, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_rpy_rates=np.zeros(3)):
        
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
        x_current = np.hstack([cur_pos, cur_vel, cur_rpy, cur_ang_vel])
        
        # Target State (Construct the 12-vector from arguments)
        # Note: If target_rpy/vel are not provided, they default to zeros (Hover)
        x_target = np.hstack([target_pos, target_vel, target_rpy, target_rpy_rates])
        
        # 2. Update Parameters
        self.x_init_param.value = x_current
        self.x_target_param.value = x_target
        
        # 3. Solve
        try:
            self.prob.solve(solver=cp.OSQP, warm_start=True)
        except:
            print("Solver Failed")
        
        # 4. Extract Optimal Input
        if self.u.value is None:
            print("Infeasible!")
            optimal_u = np.zeros(4) # Fail-safe
        else:
            optimal_u = self.u[:, 0].value

        # 5. Convert Forces/Torques to RPM
        # Your u is [Thrust, Tx, Ty, Tz]. Convert to RPMs using your model's mixer.
        # This part depends on if you use the inverse mixer logic or a simple sqrt map.
        # Below is a simplified example assuming generic force->rpm logic:
        
        thrust = optimal_u[0]
        torques = optimal_u[1:]
        
        # You need to implement the specific mixer (CF2X/CF2P) here to get 4 RPMs
        # (Copying the logic from DSLPIDControl is a good idea)
        rpms = self._mix_forces_to_rpm(thrust, torques)
        
        return rpms, np.linalg.norm(target_pos - cur_pos), 0.0

    def _mix_forces_to_rpm(self, thrust, torques):
        # INVERT YOUR MIXING MATRIX HERE
        # Placeholder for CF2X logic:
        # F_motors = inv(Mixer) * [thrust, tx, ty, tz]
        # rpm = sqrt(F_motors / KF)
        return np.array([10000, 10000, 10000, 10000]) # Dummy return