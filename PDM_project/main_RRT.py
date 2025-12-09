import numpy as np
import time
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from env_RRT import CustomCtrlAviary
from RRT import RRT
import pybullet as p
import matplotlib.pyplot as plt # Import matplotlib just in case

def main():

    # 1. INITIALIZE THE ENVIRONMENT
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
    
    # 240Hz is the default physics frequency in BaseAviary
    # We will log at this frequency for smooth plots
    control_freq_hz = 240 

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

    # Initialize the logger
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    # 2. RUN RRT PLANNING
    print("Planning path with RRT...")
    
    start_pos = [0.0, 0.0, 0.5] 
    goal_pos  = [4.0, 4.0, 0.5]

    obstacle_list = [
        [1.0, 1.0, 0.5, 0.7],
        [1.0, 1.0, 1.5, 0.7],
        [3.0, 3.0, 0.5, 0.7],
        [3.0, 3.0, 1.5, 0.7]
    ]

    rrt = RRT(
        start=start_pos,
        goal=goal_pos,
        rand_area=[-2, 5],
        obstacle_list=obstacle_list,
        expand_dis=0.2
    )
    
    path = rrt.plan()

    if path is None:
        print("RRT could not find a path!")
        env.close()
        return

    print("Resetting environment...")
    obs, info = env.reset() 

    print("Path found! Drawing it...")
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i+1], [1, 0, 0], 3, 0, env.CLIENT)
    
    current_waypoint_index = 0
    path = np.array(path)
    
    # --- CHANGE 1: Create a dedicated step counter ---
    step_counter = 0
    
    print("Environment running. Following RRT path...")

    try:
        while True:
            # Get current target waypoint
            target = path[current_waypoint_index]
            
            # Use PID to get action
            action, _, _ = ctrl.computeControlFromState(
                control_timestep=1/control_freq_hz,
                state=obs[0],
                target_pos=target,
                target_rpy=np.array([0,0,0])
            )

            # Step simulation
            obs, reward, terminated, truncated, info = env.step(np.array([action]))
            
            # Increment step counter
            step_counter += 1

            # Logging
            target_rpy = np.zeros(3)
            control_ref = np.hstack([target, target_rpy, np.zeros(6)])
            
            logger.log(drone=0,
                    timestamp=step_counter/env.CTRL_FREQ,
                    state=obs[0],
                    control=control_ref
                    )

            # Check if reached waypoint
            drone_pos = obs[0][0:3] 
            dist_to_target = np.linalg.norm(drone_pos - target)

            if dist_to_target < 0.15:
                print(f"Reached waypoint {current_waypoint_index}")
                if current_waypoint_index < len(path) - 1:
                    current_waypoint_index += 1
                else:
                    print("Goal Reached!")
                    break 

            time.sleep(1 / 120) 

    except KeyboardInterrupt:
        print("Exiting...")
    
    # --- CHANGE 4: Plotting happens AFTER the loop ---
    env.close()
    
    # Save results
    logger.save()
    
    if plot:
        print("Plotting results...")
        logger.plot()
        # Keep plots open (sometimes needed in scripts)
        plt.show()

if __name__ == "__main__":
    main()