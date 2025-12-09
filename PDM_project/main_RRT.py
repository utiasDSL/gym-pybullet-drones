import numpy as np
import time
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from env_RRT import CustomCtrlAviary
from RRT import RRT  # Import your new RRT file

def main():
    # 1. SETUP ENV
    env = CustomCtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        neighbourhood_radius=np.inf,
        physics=Physics.DYN,
        gui=True,
        record=False,
        obstacles=True,
        user_debug_gui=False
    )
    
    # Initialize the built-in PID controller
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    # 2. RUN RRT PLANNING (Before simulation starts!)
    print("Planning path with RRT...")
    
    # --- STRATEGY: CROSS THE OBSTACLE ---
    # Start: Before the cube (x=0)
    # Obstacle: In the middle (x=1)
    # Goal: Behind the cube (x=2)
    
    start_pos = [0.0, 0.0, 0.5] 
    goal_pos  = [0.0, 8.0, 0.5]


    # --- DEFINE OBSTACLES FOR RRT ---
    # Format: [x, y, z, radius]
    # We approximate the cubes as spheres for the math.
    # If the cube is 1x1x1m, a radius of 0.6m covers it safely with a small margin.
    
    obstacle_list = [
        [1.0, 0.0, 0.5, 0.6],  # The cube in front of origin
    ]

    rrt = RRT(
        start=start_pos,
        goal=goal_pos,
        rand_area=[-1, 3],     # Limit search area so it doesn't fly too far away
        obstacle_list=obstacle_list,
        expand_dis=0.2         # Step size: smaller = more precise around corners
    )
    
    path = rrt.plan()

    if path is None:
        print("RRT could not find a path!")
        env.close()
        return
    else:
        print("Path found!", path)

    # 3. SIMULATION LOOP
    obs, info = env.reset()
    
    current_waypoint_index = 0
    # Convert path list to numpy array for easier handling
    path = np.array(path)
    
    print("Environment running. Following RRT path...")

    try:
        while True:
            # Get current target waypoint
            target = path[current_waypoint_index]
            
            # Use PID to get action
            # obs[0] gives the state of the first drone
            action, _, _ = ctrl.computeControlFromState(
                control_timestep=1/240.0,
                state=obs[0],
                target_pos=target,
                target_rpy=np.array([0,0,0])
            )

            # Step simulation
            obs, reward, terminated, truncated, info = env.step(np.array([action]))
            
            # --- ADD THIS LINE HERE ---
            time.sleep(1 / 120)  # Runs at 30 FPS (Slow and smooth)
            
            # CHECK IF WE REACHED WAYPOINT
            # Extract drone position (first 3 elements of state)
            drone_pos = obs[0][0:3] 
            dist_to_target = np.linalg.norm(drone_pos - target)

            if dist_to_target < 0.15: # If within 15cm of target
                print(f"Reached waypoint {current_waypoint_index}")
                if current_waypoint_index < len(path) - 1:
                    current_waypoint_index += 1
                else:
                    print("Goal Reached!")
                    # Just hover at the last point
                    
            # time.sleep(1/240) # Uncomment if GUI is too fast

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        env.close()

if __name__ == "__main__":
    main()