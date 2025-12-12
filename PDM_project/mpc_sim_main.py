import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import pybullet_data

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from MPC import MPC_control
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from new_env import CustomCtrlAviary

# ... (Keep your default constants here) ...
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48 # Lower control freq is usually better for MPC speed
DEFAULT_DURATION_SEC = 60
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):

    # --- 1. DEFINE TRAJECTORY (WAYPOINTS) ---
    # List of points [x, y, z] the drone should visit in order
    waypoints = [
        np.array([0, 0, 1,  0, 0, 0]),
        np.array([1, 0, 1,  0, 0, 0]),
        np.array([1, 1, 1,  0, 0, 0]),
        np.array([0, 1, 1,  0, 0, 0]),
        np.array([0, 0, 1,  0, 0, 0]),
    ]

    
    # Indices to track which waypoint each drone is currently targeting
    waypoint_indices = [0] * num_drones
    
    # Initialize Target State variables
    TARGET_POS = np.zeros((num_drones, 3))
    TARGET_RPY = np.zeros((num_drones, 3))
    TARGET_VEL = np.zeros((num_drones, 3))

    # Set initial target for all drones to the first waypoint
    for j in range(num_drones):
        TARGET_POS[j, :] = waypoints[0][:3]
        TARGET_VEL[j, :] = waypoints[0][3:]

    # Initialize Sim
    INIT_XYZS = np.array([[0, 0, 0.1] for _ in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, 0] for _ in range(num_drones)])

    env = CustomCtrlAviary(drone_model=drone,
                        num_drones=num_drones,
                        initial_xyzs=INIT_XYZS,
                        initial_rpys=INIT_RPYS,
                        physics=physics,
                        neighbourhood_radius=10,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        record=record_video,
                        obstacles=True,                             #obstacles=obstacles
                        user_debug_gui=user_debug_gui
                        )
    
    for k in range(len(waypoints) - 1):
        p.addUserDebugLine(
            waypoints[k][:3].tolist(),
            waypoints[k+1][:3].tolist(),
            [1, 0, 0],          # red
            lineWidth=3,
            lifeTime=0,         # 0 = permanent
            physicsClientId=env.CLIENT
        )
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    if drone in [DroneModel.CF2X, DroneModel.CF2P, DroneModel.RACE]:
        ctrl = [MPC_control(drone_model=drone) for i in range(num_drones)]

    action = np.zeros((num_drones,4))
    START = time.time()

    # get the obstacles information and pass them to the MPC controller
    all_obstacles = env.get_obstacles_info()
    obstacles_center, r_obs =  ctrl[0].obstacles(all_obstacles)
    r_drone = env.drone_radius() 
    

    # --- SIMULATION LOOP ---
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):
        
        # Step the simulation
        # print("Action at step", i, ":", action)
        
        obs, reward, terminated, truncated, info = env.step(action)
        # print("Obs at step", i, ":", obs)

        for j in range(num_drones):
            
            # --- 2. WAYPOINT SWITCHING LOGIC ---
            # Get current position of drone j
            cur_pos = obs[j][0:3]

            ctrl[j].update_param(cur_pos, r_drone, obstacles_center, r_obs)
            
            # Calculate distance to current target
            dist_to_target = np.linalg.norm(TARGET_POS[j, :] - cur_pos)
            
            # Threshold: How close to get before switching (e.g., 15cm)
            if dist_to_target < 0.02:
                # Move to next waypoint index
                if waypoint_indices[j] < len(waypoints) - 1:
                    waypoint_indices[j] += 1
                    TARGET_POS[j, :] = waypoints[waypoint_indices[j]][:3]
                    TARGET_VEL[j, :] = waypoints[waypoint_indices[j]][3:]
                    print(f"Drone {j} Reached target! Switching to: {TARGET_POS[j, :]}")

            # --- 3. COMPUTE CONTROL ---
            action[j,:], _, _ = ctrl[j].computeControlFromState(
                                            control_timestep=env.CTRL_TIMESTEP,
                                            state=obs[j],
                                            target_pos=TARGET_POS[j, :],
                                            target_rpy=TARGET_RPY[j, :],
                                            target_vel=TARGET_VEL[j, :],
                                            )
            
            
        # Log
        for j in range(num_drones):
            logger.log(drone=j,
                    timestamp=i/env.CTRL_FREQ,
                    state=obs[j],
                    control=np.hstack([TARGET_POS[j, :], TARGET_RPY[j, :], TARGET_VEL[j, :], np.zeros(3)])
                    )
            
        env.render()
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    env.close()
    logger.save()
    logger.save_as_csv("mpc_trajectory")
    
    if plot:
        logger.plot()

if __name__ == "__main__":
    run()
