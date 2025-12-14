import numpy as np
import time
import pybullet as p
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger
from env import CustomCtrlAviary
from RRT import RRTStar
from MPC import MPC_control


def check_spline_collision(spline, total_time, obstacle_list, dt=0.1):
    """Safety check: ensures the spline doesn't cut corners into obstacles."""
    times = np.arange(0, total_time, dt)
    for t in times:
        pos = spline(t)
        x, y, z = pos

        # 1. Check Ground Collision
        if z < 0.05: 
            return True

        # 2. Check Obstacles Collision
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
    plot = True  # Enable plotting
    
    start_pos = [0.0, 0.0, 0.1] 
    goal_pos  = [9.0, 12.0, 0.2]

    # RRT* takes into account the maze walls
    # Format: [x, y, z, half_x, half_y, half_z]
    maze_walls = [
        # 1. Bottom Boundary
        [4.5, -3.0, 3.0,  9.0, 1.5, 3.0],
        
        # 2. Top Boundary
        [4.5, 15.0, 3.0,  9.0, 1.5, 3.0],
        
        # 3. Left Boundary
        [-3.0, 6.0, 3.0,  1.5, 7.5, 3.0],
        
        # 4. Right Boundary
        [12.0, 6.0, 3.0,  1.5, 7.5, 3.0],
        
        # 5. Inner Wall 1
        [3.0,  3.0, 3.0,  4.5, 1.5, 3.0],
        
        # 6. Inner Wall 2
        [6.0,  9.0, 3.0,  4.5, 1.5, 3.0]
    ]

    # We add a safety margin to the Half-Extents (indices 3, 4, 5)
    SAFETY_MARGIN = 0.4  # meters (Drone radius is ~0.1m, so 0.4m gives 0.3m clearance)
    
    rrt_obstacles = []
    for wall in maze_walls:
        # Copy the wall to avoid modifying the original list
        inflated_wall = list(wall)
        
        # Add margin to x-width and y-width
        inflated_wall[3] += SAFETY_MARGIN # half_x
        inflated_wall[4] += SAFETY_MARGIN # half_y
        # inflated_wall[5] += SAFETY_MARGIN # half_z (Optional, usually not needed for vertical walls)
        
        rrt_obstacles.append(inflated_wall)

    # --- PHASE 1: PLANNING (RRT* + SPLINE) ---
    print("Starting Planning...")
    traj_spline = None
    total_time = 0
    
    for attempt in range(100): # Reduce attempts, we want one GOOD attempt
        print(f"Planning Attempt {attempt+1}...")
        
        rrt = RRTStar(
            start=start_pos, 
            goal=goal_pos, 
            rand_area=[-5, 20],  # Give it plenty of margin outside the walls
            obstacle_list=rrt_obstacles, 
            expand_dis=1.5,      # Aggressive step size (was 1.5)
            goal_sample_rate=30, # Focus heavily on the goal (was default)
            max_iter=10000       # Plenty of iterations
        )
        
        path = rrt.plan()
        
        if path is None: 
            print("  -> No path found.")
            continue

        print(f"  -> Path found with {len(path)} nodes! Smoothing...")

        path = np.array(path)
        
        # 1. Filter path (Remove duplicates)
        unique_path = [path[0]]
        for p_idx in range(1, len(path)):
            if np.linalg.norm(path[p_idx] - unique_path[-1]) > 0.1:
                unique_path.append(path[p_idx])
        path = np.array(unique_path)

        # 2. Calculate Segment Distances
        segment_vectors = np.diff(path, axis=0)
        segment_dists = np.linalg.norm(segment_vectors, axis=1)
        total_path_len = np.sum(segment_dists)
        
        # 3. Dynamic Velocity Profiling
        # We build the "Time" array segment by segment
        current_dist = 0
        times = [0.0]
        
        CRUISE_SPEED = 1.2  # Fast!
        APPROACH_DIST = 5.0 # Start braking 3 meters away
        MIN_SPEED = 0.2     # Arrival speed

        for i, dist in enumerate(segment_dists):
            # How far are we along the path?
            current_dist += dist
            dist_to_go = total_path_len - current_dist
            
            # Logic: If we are close to the end, fly slower
            if dist_to_go < APPROACH_DIST:
                # Linear interpolation: Brake from 1.2 -> 0.2
                # ratio 1.0 = far, ratio 0.0 = at goal
                ratio = dist_to_go / APPROACH_DIST 
                speed = MIN_SPEED + (CRUISE_SPEED - MIN_SPEED) * ratio
            else:
                speed = CRUISE_SPEED
            
            # Time = Distance / Speed
            dt = dist / max(speed, 0.1) # Safety clamp
            times.append(times[-1] + dt)
            
        t_waypoints = np.array(times)
        total_time = t_waypoints[-1]
        
        # 4. Generate Clamped Spline
        # The spline will now naturally slow down at the end because 
        # the timestamps t_waypoints are spaced further apart.
        temp_spline = CubicSpline(t_waypoints, path, axis=0, bc_type='clamped')

        if not check_spline_collision(temp_spline, total_time, rrt_obstacles):
            traj_spline = temp_spline
            print(f"Safe Trajectory Found! Duration: {total_time:.2f}s")
            break
            
    if traj_spline is None:
        print("Failed to find path.")
        return

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
        user_debug_gui=False
    )
    
    # Hide setup glitches
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.CLIENT)
    
    obs, _ = env.reset()

    # 1. Get Obstacles (Empty if only maze.urdf is loaded)
    all_obstacles = env.get_obstacles_info()
    num_obstacles = len(all_obstacles)
    print(f"MPC detected {num_obstacles} dynamic obstacles.")

    # 2. Initialize MPC
    ctrl = MPC_control(drone_model=drone_model, num_obstacles=num_obstacles)
    
    logger = Logger(logging_freq_hz=control_freq_hz, num_drones=1, output_folder='results')

    # Draw RRT* trajectory (RED)
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i+1], [1, 0, 0], 3, 0, env.CLIENT)
    
    # Draw SPLine trajectory (BLUE)
    draw_dt = 0.2
    times = np.arange(0, total_time, draw_dt)
    for t in times[:-1]:
        p1 = traj_spline(t)
        p2 = traj_spline(t + draw_dt)
        p.addUserDebugLine(p1, p2, [0, 0, 1], 3, 0, env.CLIENT)

    # Re-enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.CLIENT)

    # Prepare MPC Obstacle Data (Pre-calc optimization)
    obs_centers = []
    obs_radii = []
    if num_obstacles > 0:
        obs_centers = [o['pos'] for o in all_obstacles]
        obs_radii = [o['r'] for o in all_obstacles]
    
    r_drone = env.drone_radius()

    # --- PHASE 3: EXECUTION ---
    print(f"Flying... Duration: {total_time:.2f}s")
    
    # Run loop
    for i in range(int(total_time * control_freq_hz) + 200): 
        now = i / control_freq_hz
        
        # 1. Trajectory Reference
        if now > total_time: 
            target_pos = goal_pos
            target_vel = np.zeros(3)
        else:
            target_pos = traj_spline(now)
            target_vel = traj_spline(now, 1)

        # 2. Update MPC Constraints (Only if obstacles exist)
        if num_obstacles > 0:
            ctrl.update_param(obs[0][0:3], r_drone, obs_centers, obs_radii)

        # 3. Compute Control
        action, _, _ = ctrl.computeControlFromState(
            control_timestep=1/control_freq_hz,
            state=obs[0],
            target_pos=target_pos,
            target_vel=target_vel,
            target_rpy=np.zeros(3)
        )

        # 4. Step Environment
        obs, _, _, _, _ = env.step(np.array([action]))
        
        # 5. Log
        control_ref = np.hstack([target_pos, np.zeros(3), target_vel, np.zeros(3)])
        logger.log(drone=0, timestamp=now, state=obs[0], control=control_ref)
        
        # Sync Realtime
        time.sleep(1/control_freq_hz)

    env.close()
    logger.save()
    logger.plot()

if __name__ == "__main__":
    main()