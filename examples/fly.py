import os
import time
import argparse
from datetime import datetime
import pdb
import math
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from utils import *
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionCtrlAviary import VisionCtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionCtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DroneModel.CF2X,    type=lambda model: DroneModel[model],   help='Drone model (default: CF2X)', choices=list(DroneModel), metavar='')
    parser.add_argument('--num_drones',         default=3,                  type=int,                               help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=Physics.PYB,        type=lambda phy: Physics[phy],          help='Physics updates (default: PYB)', choices=list(Physics), metavar='')
    parser.add_argument('--vision',             default=False,              type=str2bool,                          help='Whether to use VisionCtrlAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=True,               type=str2bool,                          help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=False,              type=str2bool,                          help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=True,               type=str2bool,                          help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=False,              type=str2bool,                          help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=False,              type=str2bool,                          help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=True,               type=str2bool,                          help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240,                type=int,                               help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=48,                 type=int,                               help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=5,                  type=int,                               help='Duration of the simulation in seconds (default: 5)', metavar='')
    ARGS = parser.parse_args()

    DRONE = ARGS.drone; NUM_DRONES = ARGS.num_drones; PHYSICS = ARGS.physics; 
    VISION = ARGS.vision; GUI = ARGS.gui; RECORD_VIDEO = ARGS.record_video; PLOT = ARGS.plot
    USER_DEBUG_GUI = ARGS.user_debug_gui; AGGREGATE = ARGS.aggregate; OBSTACLES = ARGS.obstacles
    SIMULATION_FREQ_HZ = ARGS.simulation_freq_hz; CONTROL_FREQ_HZ = ARGS.control_freq_hz; DURATION_SEC = ARGS.duration_sec

    #### Initialize the simulation #####################################################################
    H = .1; H_STEP = .05; R = .3; INIT_XYZS = np.array([ [R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(NUM_DRONES) ])
    AGGR_PHY_STEPS = int(SIMULATION_FREQ_HZ/CONTROL_FREQ_HZ) if AGGREGATE else 1

    #### Create the environment with or without video capture part of each drone's state ###############
    if VISION: env = VisionCtrlAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, \
                    visibility_radius=10, freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=GUI, record=RECORD_VIDEO, obstacles=OBSTACLES)
    else: env = CtrlAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, \
                    visibility_radius=10, freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=GUI, record=RECORD_VIDEO, obstacles=OBSTACLES, user_debug_gui=USER_DEBUG_GUI)

    #### Initialize a circular trajectory ##############################################################
    PERIOD = 10; NUM_WP = CONTROL_FREQ_HZ*PERIOD; TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP): TARGET_POS[i,:] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0,0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0,1], INIT_XYZS[0,2]  
    wp_counters = np.array([ int((i*NUM_WP/6)%NUM_WP) for i in range(NUM_DRONES) ])
    
    #### Initialize the logger #########################################################################
    logger = Logger(logging_freq_hz=int(SIMULATION_FREQ_HZ/AGGR_PHY_STEPS), num_drones=NUM_DRONES)

    #### Initialize the controllers ####################################################################    
    ctrl = [DSLPIDControl(env) for i in range(NUM_DRONES)]
    # ctrl = [SimplePIDControl(env) for i in range(NUM_DRONES)]

    #### Run the simulation ############################################################################
    CTRL_EVERY_N_STEPS= int(np.floor(env.SIM_FREQ/CONTROL_FREQ_HZ))
    action = { str(i): np.array([0,0,0,0]) for i in range(NUM_DRONES) }
    START = time.time()
    for i in range(0, int(DURATION_SEC*env.SIM_FREQ), AGGR_PHY_STEPS):

        #### Step the simulation ###########################################################################
        obs, reward, done, info = env.step(action)

        #### Compute control at the desired frequency @@@@@#################################################       
        if i%CTRL_EVERY_N_STEPS==0:

            #### Compute control for the current way point #####################################################
            for j in range(NUM_DRONES): 
                action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP, state=obs[str(j)]["state"], \
                                                                                target_pos=np.hstack([TARGET_POS[wp_counters[j],0:2], H+j*H_STEP]))

            #### Go to the next way point and loop #############################################################
            for j in range(NUM_DRONES): wp_counters[j] = wp_counters[j] + 1 if wp_counters[j]<(NUM_WP-1) else 0

        #### Log the simulation ############################################################################
        for j in range(NUM_DRONES): logger.log(drone=j, timestamp=i/env.SIM_FREQ, state= obs[str(j)]["state"], control=np.hstack([ TARGET_POS[wp_counters[j],0:2], H+j*H_STEP, np.zeros(9) ]))   
        
        #### Printout ######################################################################################
        if i%env.SIM_FREQ==0: 
            env.render()
            #### Print the matrices with the images captured by each drone #####################################
            if VISION: 
                for j in range(NUM_DRONES): print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),\
                                                    obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),\
                                                    obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"]))

        #### Sync the simulation ###########################################################################
        if GUI: sync(i, START, env.TIMESTEP)
   
    #### Close the environment #########################################################################
    env.close()

    #### Save the simulation results ###################################################################
    logger.save()

    #### Plot the simulation results ###################################################################
    if PLOT: logger.plot()
