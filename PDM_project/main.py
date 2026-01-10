import numpy as np
import time
import pybullet as p
from scipy.interpolate import CubicSpline

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger
from env import CustomCtrlAviary
from RRT import RRTStar
from MPC import MPC_control

import helpers
from helpers import plot_mpc_solver_stats
from helpers import plot_rrt_mpc_2d
from helpers import plot_convex_snapshot_xy
from helpers import animate_convex_tunnel_xy
from helpers import plot_convex_frames_grid





# Safety check: ensures the spline does not cut into obstacles
def check_spline_collision(spline, total_time, obstacle_list, dt=0.1):
    times = np.arange(0, total_time, dt)
    for t in times:
        pos = spline(t)
        x, y, z = pos

        # 1. Ground collision check
        if z < 0.05:
            return True

        # 2. Obstacles Collision check
        for (ox, oy, oz, hx, hy, hz) in obstacle_list:
            if (ox - hx < x < ox + hx) and \
               (oy - hy < y < oy + hy) and \
               (oz - hz < z < oz + hz):
                return True
    return False

def main():

    # --- CONFIGURATION ---
    drone_model = DroneModel.CF2X
    physics = Physics.PYB
    control_freq_hz = 48
    sim_freq_hz = 240

    start_pos = [0.0, 0.0, 0.1]
    goal_pos = [9.0, 12.0, 0.2]

    # RRT* takes into account the maze walls - Format: [x, y, z, half_x, half_y, half_z]
    maze_walls = [
        [4.5, -3.0, 3.0,  9.0, 1.5, 3.0], # 1. Bottom Boundary
        [4.5, 15.0, 3.0,  9.0, 1.5, 3.0], # 2. Top Boundary
        [-3.0, 6.0, 3.0,  1.5, 7.5, 3.0], # 3. Left Boundary
        [12.0, 6.0, 3.0,  1.5, 7.5, 3.0], # 4. Right Boundary
        [3.0,  3.0, 3.0,  4.5, 1.5, 3.0], # 5. Inner Wall 1
        [6.0,  9.0, 3.0,  4.5, 1.5, 3.0], # 6. Inner Wall 2'

        [7.0, -1.0, 0.5,  0.5, 0.5, 0.5], # 1. First obstacle
        [7.0,  0.0, 0.5,  0.5, 0.5, 0.5], # 1. First obstacle
        [7.0,  0.0, 1.5,  0.5, 0.5, 0.5], # 1. First obstacle
        [7.0,  1.0, 0.5,  0.5, 0.5, 0.5], # 1. First obstacle
        [7.0,  1.0, 1.5,  0.5, 0.5, 0.5], # 1. First obstacle
        [7.0,  1.0, 2.5,  0.5, 0.5, 0.5], # 1. First obstacle

        [2.0,  5.0, 2.5,  0.5, 0.5, 0.5], # 2. Second obstacle
        [2.0,  6.0, 0.5,  0.5, 0.5, 0.5], # 2. Second obstacle
        [2.0,  6.0, 1.5,  0.5, 0.5, 0.5], # 2. Second obstacle
        [2.0,  6.0, 2.5,  0.5, 0.5, 0.5], # 2. Second obstacle
        [2.0,  7.0, 2.5,  0.5, 0.5, 0.5], # 2. Second obstacle

        [2.0,  11.0, 0.5,  0.5, 0.5, 0.5], # 3. Third obstacle
        [2.0,  11.0, 1.5,  0.5, 0.5, 0.5], # 3. Third obstacle
        [2.0,  12.0, 0.5,  0.5, 0.5, 0.5], # 3. Third obstacle
        [2.0,  12.0, 1.5,  0.5, 0.5, 0.5], # 3. Third obstacle
        [2.0,  13.0, 0.5,  0.5, 0.5, 0.5], # 3. Third obstacle
        [2.0,  13.0, 1.5,  0.5, 0.5, 0.5] # 3. Third obstacle
    ]
    cube_walls = [
    [7.0, -1.0, 0.5,  0.5, 0.5, 0.5],
    [7.0,  0.0, 0.5,  0.5, 0.5, 0.5],
    [7.0,  0.0, 1.5,  0.5, 0.5, 0.5],
    [7.0,  1.0, 0.5,  0.5, 0.5, 0.5],
    [7.0,  1.0, 1.5,  0.5, 0.5, 0.5],
    [7.0,  1.0, 2.5,  0.5, 0.5, 0.5],

    [2.0,  5.0, 2.5,  0.5, 0.5, 0.5],
    [2.0,  6.0, 0.5,  0.5, 0.5, 0.5],
    [2.0,  6.0, 1.5,  0.5, 0.5, 0.5],
    [2.0,  6.0, 2.5,  0.5, 0.5, 0.5],
    [2.0,  7.0, 2.5,  0.5, 0.5, 0.5],

    [2.0,  11.0, 0.5,  0.5, 0.5, 0.5],
    [2.0,  11.0, 1.5,  0.5, 0.5, 0.5],
    [2.0,  12.0, 0.5,  0.5, 0.5, 0.5],
    [2.0,  12.0, 1.5,  0.5, 0.5, 0.5],
    [2.0,  13.0, 0.5,  0.5, 0.5, 0.5],
    [2.0,  13.0, 1.5,  0.5, 0.5, 0.5],
]
    # We add a safety margin (sort of boundig box for walls)
    SAFETY_MARGIN = 0.4
    
    rrt_obstacles = []

    # Create array of safety bounded walls
    for wall in maze_walls:
        inflated_wall = list(wall)

        # Add margin to x-width and y-width
        inflated_wall[3] += SAFETY_MARGIN  # half_x
        inflated_wall[4] += SAFETY_MARGIN  # half_y

        rrt_obstacles.append(inflated_wall)

    # --- PHASE 1: PLANNING (RRT* + SPLINE) ---
    print("Starting Planning...")
    traj_spline = None
    total_time = 0

    # >>> ADDED: keep these so we can plot them later
    path = None
    new_obstacles = None
    convex_log = []  # to log convex regions along the path for plotting later

    for attempt in range(100):  # Tries many times to find a suitable SPLine
        print(f"Planning Attempt {attempt + 1}...")

        rrt = RRTStar(
            start=start_pos,
            goal=goal_pos,
            rand_area=[-5, 20],
            obstacle_list=rrt_obstacles,
            expand_dis=1.5,
            goal_sample_rate=30,
            max_iter=20000
        )

        path_candidate = rrt.plan()

        if path_candidate is None:
            print("  -> No path found.")
            continue

        print(f"  -> Path found with {len(path_candidate)} nodes. Trying to find a suitable SPLine.")

        path_candidate = np.array(path_candidate)

        # 1. Filter path (remove duplicates)
        unique_path = [path_candidate[0]]
        for p_idx in range(1, len(path_candidate)):
            if np.linalg.norm(path_candidate[p_idx] - unique_path[-1]) > 0.1:
                unique_path.append(path_candidate[p_idx])
        path_candidate = np.array(unique_path)

        # 2. Select random waypoints for obstacle creation
        # (NOTE: your code uses 8 obstacles; keep as-is)
        new_obstacles_idx = np.random.choice(range(1, path_candidate.shape[0] - 3), size=3, replace=False)
        new_obstacles_candidate = path_candidate[new_obstacles_idx]

        # 3. Calculate Segment Distances
        segment_vectors = np.diff(path_candidate, axis=0)
        segment_dists = np.linalg.norm(segment_vectors, axis=1)
        total_path_len = np.sum(segment_dists)

        # 4. Dynamic velocity profiling
        current_dist = 0
        times = [0.0]

        CRUISE_SPEED = 1.0  # Cruise speed
        APPROACH_DIST = 2.0 # Start braking 2 meters away
        MIN_SPEED = 0.2     # Arrival speed

        for dist in segment_dists:
            current_dist += dist
            dist_to_go = total_path_len - current_dist

            if dist_to_go < APPROACH_DIST:
                ratio = dist_to_go / APPROACH_DIST
                speed = MIN_SPEED + (CRUISE_SPEED - MIN_SPEED) * ratio
            else:
                speed = CRUISE_SPEED

            dt = dist / max(speed, 0.1)
            times.append(times[-1] + dt)

        t_waypoints = np.array(times)
        total_time_candidate = t_waypoints[-1]

        # Generate clamped spline
        temp_spline = CubicSpline(t_waypoints, path_candidate, axis=0, bc_type='clamped')

        if not check_spline_collision(temp_spline, total_time_candidate, rrt_obstacles):
            traj_spline = temp_spline
            total_time = total_time_candidate

            # >>> ADDED: keep the winning path/obstacles for plotting later
            path = path_candidate
            new_obstacles = new_obstacles_candidate

            print(f"Safe Trajectory Found. Duration: {total_time:.2f}s")
            break

    if traj_spline is None:
        print("Failed to find path.")
        return

    # >>> ADDED: sample the spline densely for the final 2D plot
    plot_N = 600
    t_plot = np.linspace(0.0, total_time, plot_N)
    spline_samples = np.array([traj_spline(t) for t in t_plot])  # shape (N,3)

    # --- PHASE 2: SIMULATION SETUP ---
    env = CustomCtrlAviary(
        drone_model=drone_model,
        num_drones=1,
        initial_xyzs=np.array([start_pos]),
        physics=physics,
        pyb_freq=sim_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=True,
        obstacles=True,
        user_debug_gui=False,
        obstacles_pos=new_obstacles
    )

    # Disable rendering to avoid simulation glitches at the start
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.CLIENT)

    obs, _ = env.reset()

    # Get obstacles from the environment (maze.urdf does not count here)
    all_obstacles = env.get_obstacles_info()
    num_obstacles = len(all_obstacles)
    print(f"MPC detected {num_obstacles} dynamic obstacles.")
    print("RRT OBSTACLES (for MPC walls):", len(rrt_obstacles))

    # Initialize MPC
    ctrl = MPC_control(drone_model=drone_model, num_obstacles=num_obstacles, static_walls=cube_walls)

    logger = Logger(logging_freq_hz=control_freq_hz, num_drones=1, output_folder='results')

    # Draw RRT* trajectory (RED)
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i + 1], [1, 0, 0], 3, 0, env.CLIENT)

    # Draw SPLine trajectory (GREEN)
    draw_dt = 0.2
    times = np.arange(0, total_time, draw_dt)
    for t in times[:-1]:
        p1 = traj_spline(t)
        p2 = traj_spline(t + draw_dt)
        p.addUserDebugLine(p1, p2, [0, 1, 0], 3, 0, env.CLIENT)

    # Re-enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.CLIENT)

    # Prepare MPC obstacle data
    obs_centers = []
    obs_radii = []
    if num_obstacles > 0:
        obs_centers = [o['pos'] for o in all_obstacles]
        obs_radii = [o['r'] for o in all_obstacles]

    r_drone = env.drone_radius()

    # --- PHASE 3: EXECUTION ---
    print(f"Flying. Duration: {total_time:.2f}s")

    # >>> ADDED: store real drone trajectory for final 2D plotting
    drone_traj = []

    # Run loop
    for i in range(int(total_time * control_freq_hz) + 200):
        now = i / control_freq_hz

        # 1. Trajectory reference
        if now > total_time:
            target_pos = goal_pos
            target_vel = np.zeros(3)
        else:
            target_pos = traj_spline(now)
            target_vel = traj_spline(now, 1)

        # 2. Update MPC constraints (if obstacles exist)
        if num_obstacles > 0:
            ctrl.update_param(obs[0][0:3], r_drone, obs_centers, obs_radii)
        
            if num_obstacles > 0 and ctrl.last_A is not None:
                convex_log.append({
                    "t": now,
                    "A": ctrl.last_A.copy(),
                    "b": ctrl.last_b.copy(),
                    "z": float(obs[0][2]),
                    "pos": obs[0][0:3].copy()
                })
        # >>> ADDED: build reference horizon for corridor constraint (k=0..T)  (ALWAYS)
        ref_h = np.zeros((3, ctrl.T + 1))
        for k in range(ctrl.T + 1):
            tk = now + k * ctrl.dt           # use MPC dt
            tk = min(tk, total_time)         # clamp so we don't go past spline end
            ref_h[:, k] = traj_spline(tk)
        ctrl.ref_param.value = ref_h


        # 3. Compute control
        action, _, _ = ctrl.computeControlFromState(
            control_timestep=1 / control_freq_hz,
            state=obs[0],
            target_pos=target_pos,
            target_vel=target_vel,
            target_rpy=np.zeros(3)
        )

        # 4. Step environment
        obs, _, _, _, _ = env.step(np.array([action]))

        # Chase camera logic
        drone_pos = obs[0][0:3]
        drone_rpy = obs[0][7:10]  # Roll, Pitch, Yaw

        # >>> ADDED: log position for 2D plot
        drone_traj.append(drone_pos.copy())

        # Convert Yaw from Radians to Degrees for PyBullet
        yaw_degrees = np.degrees(drone_rpy[2])

        # Update Camera
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=yaw_degrees - 90,
            cameraPitch=-30,
            cameraTargetPosition=drone_pos,
            physicsClientId=env.CLIENT
        )

        # 5. Log
        control_ref = np.hstack([target_pos, np.zeros(3), target_vel, np.zeros(3)])
        logger.log(drone=0, timestamp=now, state=obs[0], control=control_ref)

        # Sync realtime
        time.sleep(1 / control_freq_hz)

    # Close the environment and plot
    env.close()
    logger.save()
    logger.plot()
    plot_mpc_solver_stats(ctrl, save_path="results/solver_stats.png")


    # >>> ADDED: produce a dedicated 2D plot (x-y) with altitude shown by color + over/under markers
    drone_traj = np.array(drone_traj)  # (T,3)
    plot_rrt_mpc_2d(
        path_xyz=path,                      # discrete RRT path nodes
        spline_xyz=spline_samples,          # spline samples
        maze_walls=maze_walls,              # rectangles
        mpc_obstacles_info=all_obstacles,   # circles (pos + r)
        drone_traj_xyz=drone_traj,          # real trajectory
        start_pos=start_pos,
        goal_pos=goal_pos,
        save_path="results/rrt_mpc_2d.png"  # comment out if you don't want saving
    )

    if num_obstacles > 0 and len(convex_log) > 0:
        # 1) One clear snapshot (middle)
        plot_convex_snapshot_xy(
            drone_traj=drone_traj,
            convex_log=convex_log,
            obs_centers=obs_centers,
            obs_radii=obs_radii,
            r_drone=r_drone,
            bbox=(-2, 12, -2, 16),
            snap_index=len(convex_log)//2
        )

        # 2) Evolution as animation
        animate_convex_tunnel_xy(
            drone_traj=drone_traj,
            convex_log=convex_log,
            obs_centers=obs_centers,
            obs_radii=obs_radii,
            r_drone=r_drone,
            bbox=(-2, 12, -2, 16),
            stride=8,
            interval_ms=80,
            save_path="convex_tunnel_evolution.gif",  # or .mp4
            dpi=120
        )



        # (Optional) Evolution as a grid of frames (no animation backend issues)
        plot_convex_frames_grid(
            drone_traj=drone_traj,
            convex_log=convex_log,
            obs_centers=obs_centers,
            obs_radii=obs_radii,
            r_drone=r_drone,
            bbox=(-2, 12, -2, 16),
            rows=2, cols=4
        )
        
        



if __name__ == "__main__":
    main()
