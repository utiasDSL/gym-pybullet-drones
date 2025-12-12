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

def main():

    # Select global planning algorithm
    alg = 1  # 1 for RRT, 2 for RRT*, 3 for RRT*_new

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

    # Define obstacles position and bounding sphere
    """
    obstacle_list = [
        [1.0, 3.0, 0.5, 0.6, 0.6, 0.6], 
        [1.0, 3.0, 1.5, 0.6, 0.6, 0.6],
        [3.0, 1.0, 0.5, 0.6, 0.6, 0.6],
        [3.0, 1.0, 1.5, 0.6, 0.6, 0.6],
        [2.0, 2.0, 0.5, 0.6, 0.6, 0.6],
        [2.0, 2.0, 1.5, 0.6, 0.6, 0.6],
    ]
    """

    obstacle_list = [
        [1.3, 2.5, 0.5, 0.6, 0.6, 0.6], 
        [1.3, 2.5, 1.5, 0.6, 0.6, 0.6],
        [2.5, 1.3, 0.5, 0.6, 0.6, 0.6],
        [2.5, 1.3, 1.5, 0.6, 0.6, 0.6]
    ]

    
    # Initialize selected global planning algorithm
    if alg == 1:
        # Initialize the RRT
        rrt = RRT(
            start=start_pos,
            goal=goal_pos,
            rand_area=[-1, 5],                 # will be -10, 10
            obstacle_list=obstacle_list,
            expand_dis=0.2
        )
    elif alg == 2:
        # Initialize the RRT*
        rrt = RRTStar(
            start=start_pos,
            goal=goal_pos,
            rand_area=[-1, 5],                 # will be -10, 10
            obstacle_list=obstacle_list,
            expand_dis=0.2
        )
    elif alg == 3:
        rrt = RRTStarNew(
            start=start_pos,
            goal=goal_pos,
            rand_area=[-1, 5],                 # will be -10, 10
            obstacle_list=obstacle_list,
            expand_dis=0.2
        )
    else:
        print("Select a valid global planning algorithm")
        return

    # Search for a path by using RRT
    path = rrt.plan()

    # Return if path not found
    if path is None:
        print("RRT could not find a path")
        return

    # --- ADD THIS: PATH SMOOTHING ---
    # Inject extra points so the drone has a "smooth curve" to follow
    # instead of sharp zig-zags.
    new_path = []
    path = np.array(path)
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i+1]
        dist = np.linalg.norm(p2 - p1)
        # Add a point every 10cm
        num_steps = int(dist / 0.10) 
        for s in range(num_steps):
            interp_pt = p1 + (p2 - p1) * (s / num_steps)
            new_path.append(interp_pt)
    new_path.append(path[-1]) # Add goal
    path = np.array(new_path)

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

    # Disable rendering until the environment reset
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.CLIENT)

    # Initialize the logger
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
    )

    # Initialize the controller (it will be PDM, now is PID)
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    # Reset environment
    obs, info = env.reset() 

    # Draw found path 
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i+1], [1, 0, 0], 3, 0, env.CLIENT)

    # Enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.CLIENT)
    
    # Define simulation variables
    current_waypoint_index = 0
    path = np.array(path)
    step_counter = 0

    # Run simulation
    try:

        # Simulation loop
        while True:

            # Get current target waypoint
            target = path[current_waypoint_index]
            
            # Use controller to get action
            action, _, _ = ctrl.computeControlFromState(
                control_timestep=1/control_freq_hz,
                state=obs[0],
                target_pos=target,
                target_rpy=np.array([0,0,0])
            )

            # Step simulation
            obs, reward, terminated, truncated, info = env.step(np.array([action]))
            step_counter += 1

            # Logging
            target_rpy = np.zeros(3)
            control_ref = np.hstack([target, target_rpy, np.zeros(6)])
            logger.log(drone=0,
                    timestamp=step_counter/env.CTRL_FREQ,
                    state=obs[0],
                    control=control_ref
            )

            # Check if reached waypoint (acceptability range is 15 cm)
            drone_pos = obs[0][0:3] 
            dist_to_target = np.linalg.norm(drone_pos - target)

            if dist_to_target < 0.30:
                print(f"Reached waypoint {current_waypoint_index}")
                if current_waypoint_index < len(path) - 1:
                    current_waypoint_index += 1
                else:
                    print("Goal Reached")
                    break 

            time.sleep(1 / 120) 

    except KeyboardInterrupt:
        print("Exiting")
    
    # Close environment
    env.close()
    
    # Save results and plot them
    logger.save()
    if plot:
        print("Plotting results")
        logger.plot()
        plt.show()




if __name__ == "__main__":
    main()