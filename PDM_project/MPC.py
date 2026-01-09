import control
import cvxpy as cp
import numpy as np
import pybullet as p
from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel
import time


# --- MPC CONTROLLER CLASS DEFINITION ---
class MPC_control(BaseControl):

    def __init__(self, drone_model: DroneModel, num_obstacles=0, static_walls=None, g: float=9.8):
        super().__init__(drone_model=drone_model, g=g)
        self.T = 20
        self.dt = 1/48
        # >>> CHANGED: keep dynamic obstacles separate from static walls
        self.num_dyn_obstacles = int(num_obstacles)
        self.static_walls = [] if static_walls is None else list(static_walls)

        # >>> ADDED: total number of halfspace constraints used in A_param x <= b_param
        self.num_halfspaces = self.num_dyn_obstacles + len(self.static_walls)
        #self.num_obstacles = num_obstacles # Store the count
        print("len(obstacles_center) =", (self.num_dyn_obstacles))
        print("len(static_walls)     =", len((self.static_walls)))
        print("num_halfspaces (param)=", self.num_halfspaces)

        #self.solver = cp.OSQP
        self.solver = cp.CLARABEL
        
        self.solver_settings = dict(
                                        verbose=False,
                                        max_iter=200,       
                                        tol_feas=1e-7,
                                        tol_gap_abs=1e-7,
                                        tol_gap_rel=1e-7
                                    )
        #self.solver_settings = dict(
         #   warm_start=True, 
         #   eps_abs=1e-2,   # Relaxed absolute tolerance (was 1e-3)
         #   eps_rel=1e-2,   # Relaxed relative tolerance (was 1e-3)
         #   max_iter=500,   # Allow more tries if really needed
         #   verbose=False
        #)
        
        self.L = self._getURDFParameter('arm')
        self.m = self._getURDFParameter('m')
        self.Ixx, self.Iyy, self.Izz = self._getURDFParameter('ixx'), self._getURDFParameter('iyy'), self._getURDFParameter('izz')
        self.GRAVITY = g * self.m
        
        self.g_vector = np.zeros(12)
        self.g_vector[5] = -g * self.dt

        self.MAX_RPM = np.sqrt((self._getURDFParameter('thrust2weight') * self.GRAVITY) / (4*self.KF))
        self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MAX_XY_TORQUE = (2*self.L*self.KF*self.MAX_RPM**2)/np.sqrt(2)
        else:
            self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2)
        self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)

        self.A_c = np.zeros((12, 12))
        self.B_c = np.zeros((12, 4))

        self.A_c[0, 3] = 1.0; self.A_c[1, 4] = 1.0; self.A_c[2, 5] = 1.0 
        self.A_c[3, 7] = g; self.A_c[4, 6] = -g 
        self.A_c[6, 9] = 1.0; self.A_c[7, 10] = 1.0; self.A_c[8, 11] = 1.0 

        self.B_c[5, 0] = 1.0 / self.m
        self.B_c[9, 1] = 1.0 / self.Ixx
        self.B_c[10, 2] = 1.0 / self.Iyy
        self.B_c[11, 3] = 1.0 / self.Izz

        self.A = np.eye(12) + self.A_c * self.dt
        self.B = self.B_c * self.dt   

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
                # >>> ADDED: store latest convex region (for plotting/debug)
        self.last_A = None
        self.last_b = None
        self.solve_times = []
        self.solve_status = []
        self.solve_iters = []
        
        self.VXY_MAX = 5.0   # m/s (start with 0.6–1.2 depending on your map)
        #self.VZ_MAX  = 2.0   # m/s


        self._setup_mpc_problem()

    # Setup of the MPC problem
    def _setup_mpc_problem(self):
        weight_input = np.diag([4.0, 
                                1.0, 
                                1.0, 
                                0.3])
        
        #weight_tracking = np.diag([8, 8, 12, 3, 3, 4, 2, 2, 0.5, 0.5, 0.5, 0.5])

        #We found these weights to perform better in practice for obstacle avoidance as the map is very tiny.
        #weight_tracking = np.diag([15,  15,  6, 
        #                             3,   3,   4, 
        #                             2,   2, 0.5, 
        #                           0.5, 0.5, 0.5])

        weight_tracking = np.diag([15,  15,  6, 
                                     3,   3,   4, 
                                     2,   2, 0.5, 
                                   0.5, 0.5, 0.5])

        cost = 0.
        constraints = []
        self.x = cp.Variable((12, self.T + 1))
        self.u = cp.Variable((4, self.T))
        P, L, K = control.dare(self.A, self.B, weight_tracking, weight_input)
        eig_val,eig_vec=np.linalg.eig(P)
        c=0.01


        for k in range(self.T):
            if k < self.T-1:
                constraints += [self.x[:, k+1] == self.A @ self.x[:, k] + self.B @ self.u[:, k] + self.g_vector]
                
            # Input constraints
                constraints += [self.u[:, k] <= [self.MAX_THRUST, self.MAX_XY_TORQUE, self.MAX_XY_TORQUE, self.MAX_Z_TORQUE]]
                constraints += [self.u[:, k] >= [0, -self.MAX_XY_TORQUE, -self.MAX_XY_TORQUE, -self.MAX_Z_TORQUE]]
                
            # Only add constraints if obstacles exist
            if self.A_param is not None:
                constraints += [self.A_param @ self.x[:3, k] <= self.b_param]
                
            constraints += [self.x[2, :] >= 0.05]  # z >= 0.05 for all horizon states


            x_err = self.x[:, k] - self.x_target_param
            cost += cp.quad_form(x_err, weight_tracking) + cp.quad_form(self.u[:, k], weight_input)

         # ----- Terminal cost -----
        xT_err = self.x[:, self.T] - self.x_target_param
        cost += cp.quad_form(xT_err, P)
        
        for j in range(len(eig_val)):
            constraints+= [(self.x[:,self.T]-self.x_target_param)  @ eig_vec[j]/np.linalg.norm(eig_vec[j])<=np.sqrt(c/eig_val[j])]
            constraints+= [(self.x[:,self.T]-self.x_target_param)  @ eig_vec[j]/np.linalg.norm(eig_vec[j])>=-np.sqrt(c/eig_val[j])]

        constraints += [self.x[:, 0] == self.x_init_param]
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
    
    # Update parameters 
    def update_param(self, cur_pos, r_drone, obstacles_center, r_obs):
        # Guard clause for 0 obstacles
            if self.num_halfspaces > 0:
                A, b = self.convex_region(
                    cur_pos=cur_pos,
                    r_drone=r_drone,
                    obstacle_list=obstacles_center,
                    r_obstacle_list=r_obs,
                    wall_list=self.static_walls
                )
            
           # >>> ADDED: enforce fixed size for CVXPY Parameters
            K = self.num_halfspaces

            if A.shape[0] < K:
                missing = K - A.shape[0]
                A_pad = np.zeros((missing, 3))
                b_pad = 1e6 * np.ones(missing)  # always satisfied: 0*x <= 1e6
                A = np.vstack([A, A_pad])
                b = np.hstack([b, b_pad])

            elif A.shape[0] > K:
                A = A[:K, :]
                b = b[:K]

            # store for plotting/debug
            self.last_A = A.copy()
            self.last_b = b.copy()

            # set cvxpy params (NOW shapes always match)
            self.A_param.value = A
            self.b_param.value = b



    def convex_region(self, cur_pos, r_drone, obstacle_list, r_obstacle_list, wall_list):
        A_rows, b_rows = [], []
        cur_pos = np.asarray(cur_pos).reshape(3)

        # ---------- 1) Dynamic spherical obstacles ----------
        for i, p_obs in enumerate(obstacle_list):
            p_obs = np.asarray(p_obs).reshape(3)
            r_obs = float(r_obstacle_list[i])

            v = cur_pos - p_obs
            dist = np.linalg.norm(v)
            n = v / (dist + 1e-6)

            R = r_drone + r_obs + 0.05
            q = p_obs + R * n

            b_plane = float(n @ q)
            A_obs, b_obs = -n, -b_plane

            # ensure current position satisfies the inequality
            if A_obs @ cur_pos > b_obs:
                A_obs, b_obs = -A_obs, -b_obs

            A_rows.append(A_obs)
            b_rows.append(b_obs)

        # ---------- 2) Static box walls ----------
        BOX_MARGIN = 0.05
        for wall in wall_list:
            ox, oy, oz, hx, hy, hz = map(float, wall)

            # inflate box by drone radius + margin
            hxI = hx + r_drone + BOX_MARGIN
            hyI = hy + r_drone + BOX_MARGIN
            hzI = hz + r_drone + BOX_MARGIN

            lo = np.array([ox - hxI, oy - hyI, oz - hzI])
            hi = np.array([ox + hxI, oy + hyI, oz + hzI])

            # closest point on inflated box to cur_pos
            q = np.minimum(np.maximum(cur_pos, lo), hi)

            v = cur_pos - q
            dist = np.linalg.norm(v)

            if dist < 1e-6:
                # inside/very close: push toward nearest face
                d_to_faces = np.array([
                    abs(cur_pos[0] - lo[0]), abs(hi[0] - cur_pos[0]),
                    abs(cur_pos[1] - lo[1]), abs(hi[1] - cur_pos[1]),
                    abs(cur_pos[2] - lo[2]), abs(hi[2] - cur_pos[2]),
                ])
                j = int(np.argmin(d_to_faces))
                n = np.zeros(3)
                if j == 0: n[0] = -1.0
                elif j == 1: n[0] =  1.0
                elif j == 2: n[1] = -1.0
                elif j == 3: n[1] =  1.0
                elif j == 4: n[2] = -1.0
                else:        n[2] =  1.0

                q = cur_pos.copy()
                if n[0] < 0: q[0] = lo[0]
                if n[0] > 0: q[0] = hi[0]
                if n[1] < 0: q[1] = lo[1]
                if n[1] > 0: q[1] = hi[1]
                if n[2] < 0: q[2] = lo[2]
                if n[2] > 0: q[2] = hi[2]
            else:
                n = v / dist

            # plane: n^T x >= n^T q  ->  -n^T x <= -n^T q
            b_plane = float(n @ q)
            A_wall = -n
            b_wall = -b_plane

            if A_wall @ cur_pos > b_wall:
                A_wall, b_wall = -A_wall, -b_wall

            A_rows.append(A_wall)
            b_rows.append(b_wall)

        if len(A_rows) == 0:
            return np.zeros((0, 3)), np.zeros((0,))

        return np.vstack(A_rows), np.array(b_rows).reshape(-1)


        #return np.vstack(A_rows), np.array(b_rows).reshape(-1)

    # Compute the control action (RPMs) from the state (control_timestep not necessary)
    def computeControlFromState(self, control_timestep, state, target_pos, target_rpy=np.zeros(3), target_vel=np.zeros(3), target_rpy_rates=np.zeros(3)):
        
        # Extract 12-state vector
        pos = state[0:3]
        rpy = state[7:10]
        vel = state[10:13]
        ang_vel = state[13:16]
        x_current = np.hstack([pos, vel, rpy, ang_vel])
        
        x_target = np.hstack([target_pos, target_vel, target_rpy, target_rpy_rates])
        
        self.x_init_param.value = x_current
        self.x_target_param.value = x_target
        
        # >>> ADDED: time the solver + log status/iterations
        t0 = time.perf_counter()
        try:
            self.problem.solve(
                solver=self.solver,
                warm_start=True,
                #**self.solver_settings  # <<< IMPORTANT: actually use your OSQP settings
            )
        except Exception:
            # log failure
            self.solve_times.append(time.perf_counter() - t0)
            self.solve_status.append("exception")
            self.solve_iters.append(np.nan)
            return np.zeros(4), 0, 0

        dt_solve = time.perf_counter() - t0
        self.solve_times.append(dt_solve)

        # status string (cvxpy)
        self.solve_status.append(str(self.problem.status))

        # iterations (OSQP-specific, may not always exist)
        iters = np.nan
        try:
            # In many CVXPY versions, OSQP exposes iteration count here
            iters = self.problem.solver_stats.num_iters
        except Exception:
            pass
        self.solve_iters.append(iters)

        # continue as before
        if self.u.value is None:
            optimal_u = np.zeros(4)
        else:
            optimal_u = self.u[:, 0].value


        #try:
         #   self.problem.solve(solver=self.solver, warm_start=True)
        #except:
         #   return np.zeros(4), 0, 0

        #if self.u.value is None:
         #   optimal_u = np.zeros(4)
        #else:
        #    optimal_u = self.u[:, 0].value

        rpms = self._mix_forces_to_rpm(optimal_u[0], optimal_u[1:])
        return rpms, np.linalg.norm(target_pos - state[0:3]), 0

    # Forces and RPMs
    def _mix_forces_to_rpm(self, thrust, torques):
        t_x, t_y, t_z = torques
        L_eff = self.L / np.sqrt(2) if self.DRONE_MODEL == DroneModel.CF2X else self.L
        k_thrust = 1.0 / (4.0 * self.KF)
        k_roll = 1.0 / (4.0 * self.KF * L_eff)
        k_pitch = 1.0 / (4.0 * self.KF * L_eff)
        k_yaw = 1.0 / (4.0 * self.KM)

        w2 = np.array([
            k_thrust*thrust - k_roll*t_x - k_pitch*t_y - k_yaw*t_z,
            k_thrust*thrust - k_roll*t_x + k_pitch*t_y + k_yaw*t_z,
            k_thrust*thrust + k_roll*t_x + k_pitch*t_y - k_yaw*t_z,
            k_thrust*thrust + k_roll*t_x - k_pitch*t_y + k_yaw*t_z
        ])

        return np.sqrt(np.maximum(w2, 0))
    
    # Compute control function (have a look)
    def computeControl(self, *args, **kwargs):
        pass