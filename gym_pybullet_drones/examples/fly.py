"""
Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python fly.py

Notes
-----
The drones move along a defined path of waypoints in the X-Y plane.
"""

import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from mpl_toolkits.mplot3d import Axes3D

from corridor_finder import RRT_start, Node

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_VISION = True
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_AGGREGATE = True
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 18
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
waypoints = []

norm_vs_time = []
timestamps = []
x_error = []
y_error = []
z_error = []
x_pos = []
y_pos = []
z_pos = []

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        vision=DEFAULT_VISION,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        aggregate=DEFAULT_AGGREGATE,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    #### Initialize the simulation #############################
    H = .1
    H_STEP = .05
    INIT_XYZS = np.array([[0, 0, H+i*H_STEP] for i in range(num_drones)])  # Start drones at (0, 0)
    INIT_RPYS = np.array([[0, 0, 0] for _ in range(num_drones)])  # Starting roll, pitch, yaw
    AGGR_PHY_STEPS = int(simulation_freq_hz/control_freq_hz) if aggregate else 1

    #### Define waypoints ######################################
    # waypoints = np.array([
    #     [0, 0, 0.1],   # Waypoint 1
    #     [1, 1, 0.1],   # Waypoint 2
    #     [2, 1, 0.1],   # Waypoint 3
    #     [3, 1, 0.1],  # Waypoint 4
    #     [4, 1, 0.1],   # Return to Waypoint 1
    #     [4, 2, 0.1],
    #     [4, 3, 0.1],
    #     [4, 4, 0.1],
    #     [4, 5, 0.1],
    # ])

    #### Create the environment with or without video capture ##
    if vision: 
        env = VisionAviary(drone_model=drone,
                           num_drones=num_drones,
                           initial_xyzs=INIT_XYZS,
                           initial_rpys=INIT_RPYS,
                           physics=physics,
                           neighbourhood_radius=10,
                           freq=simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=gui,
                           record=record_video,
                           obstacles=obstacles
                           )
    else: 
        env = CtrlAviary(drone_model=drone,
                         num_drones=num_drones,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=physics,
                         neighbourhood_radius=10,
                         freq=simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    global waypoints
    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone, m=env.M) for _ in range(num_drones)]
    elif drone in [DroneModel.HB]:
        ctrl = [SimplePIDControl(drone_model=drone) for _ in range(num_drones)]

    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/control_freq_hz))
    action = {str(i): np.array([0, 0, 0, 0]) for i in range(num_drones)}
    START = time.time()
    
    rrt = RRT_start(safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0)
    rrt.setPt(startPt=np.array([0, 0, 0.1]), endPt = np.array([10, 0, 0.5]), xl=-5, xh=15, yl=-5, yh=15, zl=0.1, zh=1,
                local_range=10, max_iter=1000, sample_portion=0.1, goal_portion=0.05)
    traj_exist = False
    wp_counters = 0

    for i in range(0, int(duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)
        
        depth_image = obs["0"]["dep"]

        pcd = env._pcd_generation(depth_image)
        pcd_np = np.asarray(pcd.points)

        # Set the input point cloud
        rrt.setInput(pcd_np)
        
        # Plan a path within a time limit (seconds)
        if not traj_exist:
            rrt.safeRegionExpansion(time_limit=0.5)
        else:
            rrt.safeRegionRefine(time_limit=0.5)
            rrt.safeRegionEvaluate(time_limit=0.5)
        
        # Get the generated path and check if it exists
        prev_wp = waypoints
        if rrt.getPathExistStatus():
            traj_exist = True
            waypoints, radiuses = rrt.getPath()
            print('length of path: ', len(waypoints))
            if wp_counters == 0:
                wp_counters = 1
        else:
            traj_exist = False
            print('traj not exist loser')
            waypoints = [np.array([0.0, 0.0, 0.1])]

        same_wp = True
        if len(prev_wp) != len(waypoints):
            same_wp = False
        
        for wp1, wp2 in zip(prev_wp, waypoints):
            if not np.array_equal(wp1, wp2):
                same_wp = False
                break
        
        if not same_wp:
            min_dist = 100000000000
            cp = np.array([10, 0, 0.5])
            for point in waypoints:
                cur_state = np.array([obs['0']['state'][0], obs['0']['state'][1], obs['0']['state'][2]])
                dist = np.linalg.norm(cur_state - point)
                if dist < min_dist:
                    min_dist = dist
                    cp = point
            
            target_pos = cp
            print('########## target_pos ##########', target_pos)
            print('########## waypoints ##########', waypoints)

        else:
            print('wp_counters', wp_counters)
            if wp_counters > len(waypoints):
                wp_counters = len(waypoints) - 1
            target_pos = waypoints[wp_counters]
            
        #### Compute control at the desired frequency ##############
        if i % CTRL_EVERY_N_STEPS == 0:
            #### Compute control for the current waypoint #############
            for j in range(num_drones):
                action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                       state=obs[str(j)]["state"],
                                                                       # target_pos = np.array([1, 0, 4]).flatten(),
                                                                       target_pos=target_pos.flatten(),
                                                                       target_rpy=INIT_RPYS[j, :]
                                                                       )
            #### Go to the next waypoint and loop #####################
            for j in range(num_drones):
                norm_wp = np.linalg.norm(np.array([obs[str(j)]["state"][0], obs[str(j)]["state"][1], obs[str(j)]["state"][2]]) - target_pos)
                print('x_diff: ', obs[str(j)]["state"][0] - target_pos[0] ,'y_diff: ', obs[str(j)]["state"][1] - target_pos[1],'z_diff: ',obs[str(j)]["state"][2] - target_pos[2] )
                if norm_wp < 0.25: 
                    wp_counters = wp_counters + 1
                    if wp_counters >= len(waypoints):
                        print("All waypoints reached.")
                        wp_counters = len(waypoints) - 1
            
            norm_vs_time.append(norm_wp)
            x_error.append(obs[str(j)]["state"][0] - target_pos[0])
            y_error.append(obs[str(j)]["state"][1] - target_pos[1])
            z_error.append(obs[str(j)]["state"][2] - target_pos[2])
            x_pos.append(obs[str(j)]["state"][0])
            y_pos.append(obs[str(j)]["state"][1])
            z_pos.append(obs[str(j)]["state"][2])
            timestamps.append(i / env.SIM_FREQ)



        #### Log the simulation ####################################
        # for j in range(num_drones):
        #     logger.log(drone=j,
        #                timestamp=i/env.SIM_FREQ,
        #                state=obs[str(j)]["state"],
        #                control=np.hstack([waypoints[wp_counters, :], INIT_RPYS[j, :], np.zeros(6)])
        #                )

        #### Printout ##############################################
        if i % env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if vision:
                for j in range(num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
                          )

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()
        plt.figure()
        plt.plot(timestamps, norm_vs_time)
        plt.title("Norm of Waypoint Error vs Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Norm of Waypoint Error (m)")
        plt.grid(True)

        plt.figure()
        plt.plot(timestamps, x_error)
        plt.title("x Error vs Time")
        plt.xlabel("Time (s)")
        plt.ylabel("x Error (m)")
        plt.grid(True)

        plt.figure()
        plt.plot(timestamps, y_error)
        plt.title("y Error vs Time")
        plt.xlabel("Time (s)")
        plt.ylabel("y Error (m)")
        plt.grid(True)

        plt.figure()
        plt.plot(timestamps, z_error)
        plt.title("z Error vs Time")
        plt.xlabel("Time (s)")
        plt.ylabel("z Error (m)")
        plt.grid(True)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the UAV trajectory
        ax.plot(x_pos, y_pos, z_pos, label='UAV Trajectory', color='b')

        # Plot the waypoints
        waypoints_x = waypoints[:, 0]
        waypoints_y = waypoints[:, 1]
        waypoints_z = waypoints[:, 2]
        ax.scatter(waypoints_x, waypoints_y, waypoints_z, color='r', label='Waypoints')

        # Set labels and title
        ax.set_title("3D UAV Trajectory and Waypoints")
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_zlabel("Z Position (m)")

        # Show legend and grid
        ax.legend()
        ax.grid(True)

        plt.show()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Waypoint following flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 1)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=DEFAULT_VISION,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to enable the user debug GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=DEFAULT_AGGREGATE,   type=str2bool,      help='Whether to aggregate the physics steps (default: True)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,   type=str2bool,      help='Whether to include obstacles (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,   type=int, help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,    type=int, help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,       type=int, help='Duration of simulation in seconds (default: 12)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER,    type=str, help='Output folder (default: results)', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB,      type=str2bool,      help='Whether the script is running in Google Colab (default: False)', metavar='')
    args = parser.parse_args()

    run(drone=args.drone,
        num_drones=args.num_drones,
        physics=args.physics,
        vision=args.vision,
        gui=args.gui,
        record_video=args.record_video,
        plot=args.plot,
        user_debug_gui=args.user_debug_gui,
        aggregate=args.aggregate,
        obstacles=args.obstacles,
        simulation_freq_hz=args.simulation_freq_hz,
        control_freq_hz=args.control_freq_hz,
        duration_sec=args.duration_sec,
        output_folder=args.output_folder,
        colab=args.colab
        )
