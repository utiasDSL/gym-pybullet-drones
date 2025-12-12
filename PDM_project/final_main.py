import numpy as np
import time
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from env_RRT import CustomCtrlAviary
# from RRT import RRT
# from RRTStar import RRTStar
# from RRTStar_new import RRTStarNew
from final_RRT import RRTStar
import pybullet as p
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


# --- COLLISION CHECK FOR SPLINE TRAJECTORY ---
def check_spline_collision(spline, total_time, obstacle_list, dt=0.1):
    times = np.arange(0, total_time, dt)

    for t in times:
        pos = spline(t)
        x, y, z = pos[0], pos[1], pos[2]
        
        for (ox, oy, oz, hx, hy, hz) in obstacle_list:
            if (ox - hx < x < ox + hx) and \
               (ox - hy < y < oy + hy) and \
               (oz - hz < z < oz + hz) and \
               (z <= 0.1):
                return True 
    return False 


# --- DEFINITION OF MAIN FUNCTION ---
def main():

    # --- STANDARD VARIABLE SETUP ---
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
    
    start_pos = [0.0, 0.0, 0.1] 
    goal_pos  = [4.0, 4.0, 0.2]

    # --- DEFINE SET OF OBSTACLES ---
    # we introduce a 0.1 m margin to make sure the drone does not get too close to the obstacles
    obstacles = [
        [1.0, 2.5, 0.5, 0.6, 0.6, 0.6], 
        [1.0, 2.5, 1.5, 0.6, 0.6, 0.6],
        [2.5, 1.0, 0.5, 0.6, 0.6, 0.6],
        [2.5, 1.0, 1.5, 0.6, 0.6, 0.6]
    ]

    # --- PLANNING LOOP ---
    max_retries = 200
    safe_path_found = False
    
    print("Starting Planning.")

    for attempt in range(max_retries):
        
        # 1. Initialize RRT* and path plan
        rrt = RRTStar(
            start=start_pos, 
            goal=goal_pos, 
            rand_area=[0, 5], 
            obstacle_list=obstacles, 
            expand_dis=0.5
        )

        path = rrt.plan()

        if path is None:
            continue

        # 2. Generate Spline and check if it avoids obstacles
        AVG_SPEED = 0.4
        path = np.array(path)
        distances = np.linalg.norm(np.diff(path, axis=0), axis=1)
        cum_dist = np.insert(np.cumsum(distances), 0, 0)
        total_time = cum_dist[-1] / AVG_SPEED
        t_waypoints = cum_dist / AVG_SPEED
        
        traj_spline = CubicSpline(t_waypoints, path, axis=0)

        if check_spline_collision(traj_spline, total_time, obstacles):
            print(f"Attempt {attempt}: Spline hit the wall. Trying again.")
        else:
            print("Safe Spline trajectory found.")
            safe_path_found = True
            break
    
    if not safe_path_found:
        print("Could not find a safe trajectory.")
        return

    # Initialize environment
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

    # Disable rendering to avoid starting glithces
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.CLIENT)

    # Initialize Logger and Controller
    logger = Logger(logging_freq_hz=control_freq_hz, num_drones=num_drones, output_folder=output_folder, colab=colab)
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)       # it will be MPC, now is PID

    # Reset the environment (required)
    obs, _ = env.reset() 

    # Draw RRT* trajectory (RED)
    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i], path[i+1], [1, 0, 0], 3, 0, env.CLIENT)

    # Draw Spline Trajectory (BLUE)
    draw_dt = 0.2
    draw_times = np.arange(0, total_time, draw_dt)
    for t in draw_times:
        if t + draw_dt > total_time: break
        p1 = traj_spline(t)
        p2 = traj_spline(t + draw_dt)
        p.addUserDebugLine(p1, p2, [0, 0, 1], 3, 0, env.CLIENT)

    # Enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.CLIENT)
    
    step_counter = 0

    print(f"Flying duration: {total_time:.2f}s")

    # --- SIMULATION LOOP ---
    try:

        # Simulate drone behaviour
        while True:
            now = step_counter / control_freq_hz
            if now > total_time: 
                break

            target_pos = traj_spline(now)
            target_vel = traj_spline(now, 1)

            # computes the action based on target position and velocity
            action, _, _ = ctrl.computeControlFromState(
                control_timestep=1/control_freq_hz,
                state=obs[0],
                target_pos=target_pos,
                target_vel=target_vel,
                target_rpy=np.array([0,0,0])
            )

            obs, _, _, _, _ = env.step(np.array([action]))
            step_counter += 1

            control_ref = np.hstack([target_pos, np.zeros(3), np.zeros(6)])
            logger.log(drone=0, timestamp=now, state=obs[0], control=control_ref)
            time.sleep(1 / control_freq_hz) 

    except KeyboardInterrupt:
        print("Exiting")
    
    # End of simulation and plotting
    env.close()
    logger.save()
    if plot:
        logger.plot()
        plt.show()


# --- RUN MAIN FUNCTION ---
if __name__ == "__main__":
    main()