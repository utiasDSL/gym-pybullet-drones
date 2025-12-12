import numpy as np
import time
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from env_RRT import CustomCtrlAviary
from RRT import RRT
from RRTStar import RRTStar
from RRTStar_new import RRTStarNew
import pybullet as p
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def main():

    # Select global planning algorithm
    alg = 3    # 1 for RRT, 2 for RRT*, 3 for RRT*_new

    # Define useful variables
    drone_model = DroneModel.CF2X
    num_drones = 1
    physics = Physics.DYN
    gui = True
    record = False
    plot = True
    obstacles = True
    user_debug_gui = False
    output_folder = 'results'
    colab = False
    control_freq_hz = 240 
    
    # Define start and end positions
    start_pos = [0.0, 0.0, 0.2] 
    goal_pos  = [4.0, 4.0, 0.2]

    # Define obstacles
    obstacle_list = [
        [1.0, 3.5, 0.5, 0.5, 0.5, 0.5], 
        [1.0, 3.5, 1.5, 0.5, 0.5, 0.5],
        [3.5, 1.0, 0.5, 0.5, 0.5, 0.5],
        [3.5, 1.0, 1.5, 0.5, 0.5, 0.5]
    ]

    # Initialize selected global planning algorithm
    if alg == 1:
        rrt = RRT(start=start_pos, goal=goal_pos, rand_area=[-1, 5], obstacle_list=obstacle_list, expand_dis=0.2)
    elif alg == 2:
        rrt = RRTStar(start=start_pos, goal=goal_pos, rand_area=[-1, 5], obstacle_list=obstacle_list, expand_dis=0.2)
    elif alg == 3:
        rrt = RRTStarNew(start=start_pos, goal=goal_pos, rand_area=[-1, 5], obstacle_list=obstacle_list, expand_dis=0.2)
    else:
        print("Select a valid global planning algorithm")
        return

    # Search for a path
    path = rrt.plan()

    if path is None:
        print("RRT could not find a path")
        return

    # --- NEW: SPLINE TRAJECTORY GENERATION ---
    # Instead of just adding dots, we create a function x(t)
    
    AVG_SPEED = 0.4  # m/s (Adjust this to make drone faster/slower)
    
    path = np.array(path)
    # Calculate distance between every RRT waypoint
    distances = np.linalg.norm(np.diff(path, axis=0), axis=1)
    # Calculate cumulative distance (0, d1, d1+d2, ...)
    cum_dist = np.insert(np.cumsum(distances), 0, 0)
    # Calculate total time required based on speed
    total_time = cum_dist[-1] / AVG_SPEED
    # Assign a timestamp to every RRT waypoint
    t_waypoints = cum_dist / AVG_SPEED
    
    # Create the Spline Function
    # This allows us to query position AND velocity at any time t
    traj_spline = CubicSpline(t_waypoints, path, axis=0)
    
    # -----------------------------------------

    # Create the environment
    env = CustomCtrlAviary(
        drone_model=drone_model,
        num_drones=num_drones,
        neighbourhood_radius=np.inf,
        physics=physics,
        gui=gui,
        record=record,
        obstacles=obstacles,
        user_debug_gui=user_debug_gui
    )

    # Disable rendering until setup is done
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.CLIENT)

    logger = Logger(logging_freq_hz=control_freq_hz, num_drones=num_drones, output_folder=output_folder, colab=colab)
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    obs, info = env.reset() 

    # Draw the RRT "Skeleton" path (Red Lines)
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i+1], [1, 0, 0], 3, 0, env.CLIENT)

    # Enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.CLIENT)
    
    step_counter = 0
    start_time = time.time()

    print(f"Flying smooth trajectory... Duration: {total_time:.2f}s")

    try:
        while True:
            # 1. Calculate "Flight Time" (how long we have been flying)
            # We scale it slightly to match simulation time vs real time
            # For robustness, we use step_counter / freq instead of time.time()
            now = step_counter / control_freq_hz
            
            if now > total_time:
                print("Goal Reached (Time limit exprired)")
                break

            # 2. Extract Flat Outputs from Spline
            target_pos = traj_spline(now)      # Position (x, y, z)
            target_vel = traj_spline(now, 1)   # Velocity (vx, vy, vz)

            # 3. Compute Control (WITH FEEDFORWARD VELOCITY)
            action, _, _ = ctrl.computeControlFromState(
                control_timestep=1/control_freq_hz,
                state=obs[0],
                target_pos=target_pos,
                target_vel=target_vel,   # <--- The magic fix for offsets
                target_rpy=np.array([0,0,0])
            )

            # 4. Step Simulation
            obs, reward, terminated, truncated, info = env.step(np.array([action]))
            step_counter += 1

            # 5. Logging
            control_ref = np.hstack([target_pos, np.zeros(3), np.zeros(6)])
            logger.log(drone=0,
                    timestamp=now,
                    state=obs[0],
                    control=control_ref
            )

            # Sync with real time (optional, makes GUI look realistic)
            time.sleep(1 / control_freq_hz) 

    except KeyboardInterrupt:
        print("Exiting")
    
    env.close()
    logger.save()
    if plot:
        logger.plot()
        plt.show()

if __name__ == "__main__":
    main()