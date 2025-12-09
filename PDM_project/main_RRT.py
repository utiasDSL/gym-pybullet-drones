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
    start_pos = [0.0, 0.0, 0.5] 
    goal_pos  = [4.0, 4.0, 0.5]

    # Define obstacles position and bounding sphere
    obstacle_list = [
        [1.0, 1.0, 0.5, 0.8],
        [1.0, 1.0, 1.5, 0.8],
        [3.0, 3.0, 0.5, 0.8],
        [3.0, 3.0, 1.5, 0.8]
    ]

    # Initialize the RRT
    rrt = RRT(
        start=start_pos,
        goal=goal_pos,
        rand_area=[-2, 5],
        obstacle_list=obstacle_list,
        expand_dis=0.2
    )
    
    # Search for a path by using RRT
    path = rrt.plan()

    # Return if path not found
    if path is None:
        print("RRT could not find a path")
        return

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

            if dist_to_target < 0.15:
                print(f"Reached waypoint {current_waypoint_index}")
                if current_waypoint_index < len(path) - 1:
                    current_waypoint_index += 1
                else:
                    print("Goal Reached")
                    break 

            time.sleep(1 / 240) 

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