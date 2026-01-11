import time
import control
import cvxpy as cp
import numpy as np
import pybullet as p

# PROJECT IMPORTS
from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel


# MPC CONTROLLER CLASS DEFINITION

class MPC_control(BaseControl):
    """
    Model Predictive Controller (MPC) for trajectory tracking and obstacle avoidance.
    Uses convex optimization (CVXPY) with linear halfspace constraints for obstacles.
    """

    def __init__(self, drone_model: DroneModel, num_obstacles=0, static_walls=None, g: float=9.8):
        super().__init__(drone_model=drone_model, g=g)
        
        # MPC PARAMETERS
        self.T = 30         # BEST FOUND HORIZON
        self.dt = 1/48      # Time step (seconds)
        
        # OBSTACLE CONFIGURATION 
        # Keep dynamic (the word dynamic here is used for the RANDOM obstacles that change at every run) obstacles
        # separate from static walls
        self.num_dyn_obstacles = int(num_obstacles)
        self.static_walls = [] if static_walls is None else list(static_walls)

        # Total number of halfspace constraints used in A_param x <= b_param
        self.num_halfspaces = self.num_dyn_obstacles + len(self.static_walls)
        
        print(f"MPC initialized with:")
        print(f"  Dynamic Obstacles: {self.num_dyn_obstacles}")
        print(f"  Static Walls:      {len(self.static_walls)}")
        print(f"  Total Constraints: {self.num_halfspaces}")

        # SOLVER SETTINGS 
        # self.solver = cp.OSQP
        self.solver = cp.CLARABEL   # BEST FOUND OPTIMIZER
        
        self.solver_settings = dict(
            verbose=False,
            max_iter=200,       
            tol_feas=1e-7,
            tol_gap_abs=1e-7,
            tol_gap_rel=1e-7
        )
        
        # Legacy settings (kept for reference)
        # self.solver_settings = dict(
        #   warm_start=True, 
        #   eps_abs=1e-2,   # Relaxed absolute tolerance (was 1e-3)
        #   eps_rel=1e-2,   # Relaxed relative tolerance (was 1e-3)
        #   max_iter=500,   # Allow more tries if really needed
        #   verbose=False
        # )
        
        # PHYSICAL CONSTANTS
        self.L = self._getURDFParameter('arm')
        self.m = self._getURDFParameter('m')
        self.Ixx, self.Iyy, self.Izz = self._getURDFParameter('ixx'), self._getURDFParameter('iyy'), self._getURDFParameter('izz')
        self.GRAVITY = g * self.m
        
        self.g_vector = np.zeros(12)
        self.g_vector[5] = -g * self.dt

        # LIMITS 
        self.MAX_RPM = np.sqrt((self._getURDFParameter('thrust2weight') * self.GRAVITY) / (4*self.KF))
        self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MAX_XY_TORQUE = (2*self.L*self.KF*self.MAX_RPM**2)/np.sqrt(2)
        else:
            self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2)
        self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)
        


        # DYNAMICS MATRICES (Linearized Quadrotor Model) 
        self.A_c = np.zeros((12, 12))
        self.B_c = np.zeros((12, 4))

        # Fill A matrix (State transition)
        self.A_c[0, 3] = 1.0; self.A_c[1, 4] = 1.0; self.A_c[2, 5] = 1.0 
        self.A_c[3, 7] = g;   self.A_c[4, 6] = -g 
        self.A_c[6, 9] = 1.0; self.A_c[7, 10] = 1.0; self.A_c[8, 11] = 1.0 

        # Fill B matrix (Control input)
        self.B_c[5, 0] = 1.0 / self.m
        self.B_c[9, 1] = 1.0 / self.Ixx
        self.B_c[10, 2] = 1.0 / self.Iyy
        self.B_c[11, 3] = 1.0 / self.Izz

        # Discretize
        self.A = np.eye(12) + self.A_c * self.dt
        self.B = self.B_c * self.dt   

        # OPTIMIZATION VARIABLES 
        self.x_init_param = cp.Parameter(12)
        self.x_target_param = cp.Parameter(12)
        self.ref_param = cp.Parameter((3, self.T+1))
        
        # Dynamic initialization of obstacle parameters
        if self.num_halfspaces > 0:
            self.A_param = cp.Parameter((self.num_halfspaces, 3))
            self.b_param = cp.Parameter(self.num_halfspaces)
        else:
            self.A_param = None
            self.b_param = None
        
        # Debugging / Logging
        self.last_A = None
        self.last_b = None
        self.solve_times = []
        self.solve_status = []
        self.solve_iters = []
        
        # Fallback control if MPC fails/infeasible: start from hover-like thrust
        self.u_prev = np.array([self.GRAVITY, 0.0, 0.0, 0.0], dtype=float)

        # Build the CVXPY problem structure
        self._setup_mpc_problem()


    # 1. MPC PROBLEM SETUP

    def _setup_mpc_problem(self):
        """
        Constructs the MPC optimization problem (Cost function & Constraints).
        """
        
        # Weights
        weight_input = np.diag([4.0, 1.0, 1.0, 0.3])
        
        # We found these weights to perform better in practice for obstacle avoidance as the map is very tiny.
        # weight_tracking = np.diag([15, 15, 6, 3, 3, 4, 2, 2, 0.5, 0.5, 0.5, 0.5])

        weight_tracking = np.diag([40,  40,  40, 
                                     3,   3,   4, 
                                     2,   2, 0.5, 
                                   0.5, 0.5, 0.5])

        # Variables
        self.x = cp.Variable((12, self.T + 1))
        self.u = cp.Variable((4, self.T))
        
        # TERMINAL COST Calculation (LQR approximation)
        P, L, K = control.dare(self.A, self.B, weight_tracking, weight_input)
        eig_val, eig_vec = np.linalg.eig(P)
        c = 0.01

        cost = 0.
        constraints = []
        
        # Slack variables to prevent infeasibility in tight maze spots
        # One slack variable per hyperplane constraint, per horizon step
        self.slack = cp.Variable((self.num_halfspaces, self.T), nonneg=True)

        # Build Horizon
        for k in range(self.T):
            
            # Dynamics Constraint
            if k < self.T-1:
                constraints += [self.x[:, k+1] == self.A @ self.x[:, k] + self.B @ self.u[:, k] + self.g_vector]
                
            # Input Constraints (Thrust & Torque Limits)
            constraints += [self.u[:, k] <= [self.MAX_THRUST, self.MAX_XY_TORQUE, self.MAX_XY_TORQUE, self.MAX_Z_TORQUE]]
            constraints += [self.u[:, k] >= [0, -self.MAX_XY_TORQUE, -self.MAX_XY_TORQUE, -self.MAX_Z_TORQUE]]
                
            # Obstacle Avoidance (Linear Halfspaces)
            if self.A_param is not None:
                #constraints += [self.A_param @ self.x[:3, k] <= self.b_param]
                constraints += [self.A_param @ self.x[:3, k] <= self.b_param + self.slack[:, k]]
                
            # Ground Constraint
            constraints += [self.x[2, :] >= 0.05]  # z >= 0.05 for all horizon states
            
            # Massive penalty for using slack (soft constraints)
            # This ensures the drone only violates the margin if absolutely necessary.
            cost += cp.sum(self.slack[:, k]) * 1e5

            # Stage Cost
            x_err = self.x[:, k] - self.x_target_param
            cost += cp.quad_form(x_err, weight_tracking) + cp.quad_form(self.u[:, k], weight_input)

        # Terminal Cost & Constraints
        xT_err = self.x[:, self.T] - self.x_target_param
        cost += cp.quad_form(xT_err, P)
        
        # TERMINAL SET Constraints (Stability)
        for j in range(len(eig_val)):
            constraints += [(self.x[:, self.T] - self.x_target_param) @ eig_vec[j] / np.linalg.norm(eig_vec[j]) <= np.sqrt(c/eig_val[j])]
            constraints += [(self.x[:, self.T] - self.x_target_param) @ eig_vec[j] / np.linalg.norm(eig_vec[j]) >= -np.sqrt(c/eig_val[j])]

        # Initial State Constraint
        constraints += [self.x[:, 0] == self.x_init_param]
        
        # Define Problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
    

    # 2. CONSTRAINT UPDATES (OBSTACLES)
    '''
    def update_param(self, cur_pos, r_drone, obstacles_center, r_obs):
        """
        Updates the linear constraints (A x <= b) based on the current drone position
        relative to obstacles.
        """
        # Guard clause for 0 obstacles
        if self.num_halfspaces > 0:
            A, b = self.convex_region(
                cur_pos=cur_pos,
                r_drone=r_drone,
                obstacle_list=obstacles_center,
                r_obstacle_list=r_obs,
                wall_list=self.static_walls
            )
        
            # Enforce fixed size for CVXPY Parameters (Padding/Truncating)
            K = self.num_halfspaces

            if A.shape[0] < K:
                # Pad with non-binding constraints
                missing = K - A.shape[0]
                A_pad = np.zeros((missing, 3))
                b_pad = 1e6 * np.ones(missing)  # always satisfied: 0*x <= 1e6
                A = np.vstack([A, A_pad])
                b = np.hstack([b, b_pad])

            elif A.shape[0] > K:
                # Truncate if we have too many (shouldn't happen with current logic)
                A = A[:K, :]
                b = b[:K]

            # Store for plotting/debug
            self.last_A = A.copy()
            self.last_b = b.copy()

            # Update CVXPY parameters
            self.A_param.value = A
            self.b_param.value = b

    '''
    def update_param(self, cur_pos, r_drone, obstacles_center, r_obs):
        """
        Updates constraints by filtering for only the nearest obstacles 
        and padding the rest to maintain fixed CVXPY parameter shapes.
        """
        if self.num_halfspaces > 0:
            # 1. Calculate distances to all dynamic obstacles
            dyn_obs_dists = []
            for i, p_obs in enumerate(obstacles_center):
                d = np.linalg.norm(cur_pos - np.array(p_obs))
                dyn_obs_dists.append((d, p_obs, r_obs[i]))
            
            # Sort by distance (closest first)
            dyn_obs_dists.sort(key=lambda x: x[0])
            
            # 2. Calculate distances to all walls (using center of the box)
            wall_dists = []
            for wall in self.static_walls:
                wall_center = np.array([wall[0], wall[1], wall[2]])
                d = np.linalg.norm(cur_pos - wall_center)
                wall_dists.append((d, wall))
                
            wall_dists.sort(key=lambda x: x[0])

            # 3. Pick a subset of the closest objects
            SENSING_RADIUS = 2.5 
            
            near_obs_centers = [item[1] for item in dyn_obs_dists if item[0] < SENSING_RADIUS]
            near_obs_radii = [item[2] for item in dyn_obs_dists if item[0] < SENSING_RADIUS]
            near_walls = [item[1] for item in wall_dists if item[0] < SENSING_RADIUS + 1.0]

            # 4. Generate hyperplanes for ONLY these nearby objects
            A_real, b_real = self.convex_region(
                cur_pos=cur_pos,
                r_drone=r_drone,
                obstacle_list=near_obs_centers,
                r_obstacle_list=near_obs_radii,
                wall_list=near_walls
            )

            # Store the UNPADDED versions for the plotting functions
            self.last_A = A_real.copy()
            self.last_b = b_real.copy()

            # 5. Padding Logic to maintain fixed size for CVXPY Parameters
            K = self.num_halfspaces 
            if A_real.shape[0] < K:
            
                missing = K - A_real.shape[0]
                A_pad = np.zeros((missing, 3))
                b_pad = 100.0 * np.ones(missing)
                
                A_final = np.vstack([A_real, A_pad])
                b_final = np.hstack([b_real, b_pad])
            else:
                # Truncate to the K most critical if necessary
                A_final = A_real[:K, :]
                b_final = b_real[:K]

            # Update CVXPY parameters for the solver
            self.A_param.value = A_final
            self.b_param.value = b_final

    def convex_region(self, cur_pos, r_drone, obstacle_list, r_obstacle_list, wall_list):
        """
        Calculates the separating hyperplanes for convex collision avoidance.
        Returns matrices A, b such that A * pos <= b describes the safe region.
        """
        A_rows, b_rows = [], []
        cur_pos = np.asarray(cur_pos).reshape(3)

        # Dynamic Spherical Obstacles 
        for i, p_obs in enumerate(obstacle_list):
            p_obs = np.asarray(p_obs).reshape(3)
            r_obs = float(r_obstacle_list[i])

            # Vector from obstacle center to drone
            v = cur_pos - p_obs
            dist = np.linalg.norm(v)
            n = v / (dist + 1e-6) # Normal vector

            # Effective radius (Drone + Obstacle + Margin)
            R = r_drone + r_obs + 0.05
            q = p_obs + R * n

            # Plane equation: n^T * x <= n^T * q
            b_plane = float(n @ q)
            A_obs, b_obs = -n, -b_plane

            # Ensure current position satisfies the inequality (Flip if needed)
            if A_obs @ cur_pos > b_obs:
                A_obs, b_obs = -A_obs, -b_obs

            A_rows.append(A_obs)
            b_rows.append(b_obs)

        # Static Box Walls 
        BOX_MARGIN = 0.1
        for wall in wall_list:
            ox, oy, oz, hx, hy, hz = map(float, wall)

            # Inflate box by drone radius + margin
            hxI = hx + r_drone + BOX_MARGIN
            hyI = hy + r_drone + BOX_MARGIN
            hzI = hz + r_drone + BOX_MARGIN

            lo = np.array([ox - hxI, oy - hyI, oz - hzI])
            hi = np.array([ox + hxI, oy + hyI, oz + hzI])

            # Find closest point 'q' on inflated box to cur_pos
            q = np.minimum(np.maximum(cur_pos, lo), hi)

            v = cur_pos - q
            dist = np.linalg.norm(v)

            if dist < 1e-6:
                # Inside/very close: push toward nearest face
                d_to_faces = np.array([
                    abs(cur_pos[0] - lo[0]), abs(hi[0] - cur_pos[0]),
                    abs(cur_pos[1] - lo[1]), abs(hi[1] - cur_pos[1]),
                    abs(cur_pos[2] - lo[2]), abs(hi[2] - cur_pos[2]),
                ])
                j = int(np.argmin(d_to_faces))
                
                # Determine normal based on closest face
                n = np.zeros(3)
                if j == 0: n[0] = -1.0
                elif j == 1: n[0] =  1.0
                elif j == 2: n[1] = -1.0
                elif j == 3: n[1] =  1.0
                elif j == 4: n[2] = -1.0
                else:        n[2] =  1.0

                # Snap q to the face
                q = cur_pos.copy()
                if n[0] < 0: q[0] = lo[0]
                if n[0] > 0: q[0] = hi[0]
                if n[1] < 0: q[1] = lo[1]
                if n[1] > 0: q[1] = hi[1]
                if n[2] < 0: q[2] = lo[2]
                if n[2] > 0: q[2] = hi[2]
            else:
                n = v / dist

            # Plane: n^T x >= n^T q  ->  -n^T x <= -n^T q
            b_plane = float(n @ q)
            A_wall = -n
            b_wall = -b_plane

            #if A_wall @ cur_pos > b_wall:
             #   A_wall, b_wall = -A_wall, -b_wall

            A_rows.append(A_wall)
            b_rows.append(b_wall)

        if len(A_rows) == 0:
            return np.zeros((0, 3)), np.zeros((0,))

        return np.vstack(A_rows), np.array(b_rows).reshape(-1)


    # 3. CONTROL LOOP

    def computeControlFromState(self, control_timestep, state, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_rpy_rates=np.zeros(3)):
        """
        Main control function called at every time step.
        """
        
        # Extract 12-state vector
        pos = state[0:3]
        rpy = state[7:10]
        vel = state[10:13]
        ang_vel = state[13:16]
        
        x_current = np.hstack([pos, vel, rpy, ang_vel])
        x_target = np.hstack([target_pos, target_vel, target_rpy, target_rpy_rates])
        
        # Update optimization parameters
        self.x_init_param.value = x_current
        self.x_target_param.value = x_target
        
        # Solve with timing
        t0 = time.perf_counter()
        try:
            self.problem.solve(
                solver=self.solver,
                warm_start=True,
                # **self.solver_settings  # Use specific settings if needed
            )
        except Exception:
            # Log failure and handle gracefully
            self.solve_times.append(time.perf_counter() - t0)
            self.solve_status.append("exception")
            self.solve_iters.append(np.nan)
            return np.zeros(4), 0, 0

        # Log solver stats
        dt_solve = time.perf_counter() - t0
        self.solve_times.append(dt_solve)
        self.solve_status.append(str(self.problem.status))

        iters = np.nan
        try:
            iters = self.problem.solver_stats.num_iters
        except Exception:
            pass
        self.solve_iters.append(iters)

        # Retrieve Control Action
        if self.problem.status in ["infeasible", "unbounded"] or self.x.value is None:
                print(f"MPC Warning: {self.problem.status}. Using fallback.")
                optimal_u = self.u_prev # Use last successful command
        else:
                optimal_u = self.u[:, 0].value
            
        self.u_prev = optimal_u.copy()

        # Convert optimal forces/torques to motor RPMs
        rpms = self._mix_forces_to_rpm(optimal_u[0], optimal_u[1:])
        return rpms, np.linalg.norm(target_pos - state[0:3]), 0


    # 4. FINAL FUNCTION TO OBTAIN MOTOR RPMS

    def _mix_forces_to_rpm(self, thrust, torques):
        """
        Mixes the generic force/torque outputs from MPC into specific motor RPMs.
        """
        t_x, t_y, t_z = torques
        L_eff = self.L / np.sqrt(2) if self.DRONE_MODEL == DroneModel.CF2X else self.L
        
        # Coefficients
        k_thrust = 1.0 / (4.0 * self.KF)
        k_roll = 1.0 / (4.0 * self.KF * L_eff)
        k_pitch = 1.0 / (4.0 * self.KF * L_eff)
        k_yaw = 1.0 / (4.0 * self.KM)

        # Motor mixing algorithm
        w2 = np.array([
            k_thrust*thrust - k_roll*t_x - k_pitch*t_y - k_yaw*t_z,
            k_thrust*thrust - k_roll*t_x + k_pitch*t_y + k_yaw*t_z,
            k_thrust*thrust + k_roll*t_x + k_pitch*t_y - k_yaw*t_z,
            k_thrust*thrust + k_roll*t_x - k_pitch*t_y + k_yaw*t_z
        ])

        return np.sqrt(np.maximum(w2, 0))
    
    def computeControl(self, *args, **kwargs):
        pass