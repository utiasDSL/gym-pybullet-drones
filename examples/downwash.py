import os
import time
from datetime import datetime
import pdb
import math
import numpy as np
import pybullet as p
import pickle
import matplotlib.pyplot as plt

from utils import *
from gym_pybullet_drones.envs.Aviary import DroneModel, Physics, Aviary
from gym_pybullet_drones.envs.Logger import Logger
from gym_pybullet_drones.envs.Control import ControlType, Control

DRONE = DroneModel.CF2X
NUM_DRONES = 2
GUI = True
PHYSICS = Physics.PYB_GND_DRAG_DW
RECORD_VIDEO = False
SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48
DURATION_SEC = 10

if __name__ == "__main__":

    #### Initialize the simulation #####################################################################
    INIT_XYZS = np.array([[.5,0,1],[-.5,0,.5]])
    env = Aviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, visibility_radius=10, \
                    normalized_spaces=False, freq=SIMULATION_FREQ_HZ, gui=GUI, obstacles=True, record=RECORD_VIDEO); env.reset()

    #### Initialize the trajectories ###################################################################
    PERIOD = 10; NUM_WP = CONTROL_FREQ_HZ*PERIOD; TARGET_POS = np.zeros((NUM_WP,2))
    for i in range(NUM_WP): TARGET_POS[i,:] = [0.5*np.cos(2*np.pi*(i/NUM_WP)), 0]
    wp_counters = np.array([ 0, int(NUM_WP/2) ])
    
    #### Initialize the logger #########################################################################
    logger = Logger(duration_sec=DURATION_SEC, simulation_freq_hz=SIMULATION_FREQ_HZ, num_drones=NUM_DRONES)

    #### Initialize the controllers ####################################################################    
    ctrl = [Control(env, control_type=ControlType.PID) for i in range(NUM_DRONES)]

    #### Run the simulation ############################################################################
    CTRL_EVERY_N_STEPS= int(np.floor(env.SIM_FREQ/CONTROL_FREQ_HZ))
    action = { str(i): np.array([0,0,0,0]) for i in range(NUM_DRONES) }
    START = time.time(); temp_action = {}
    for i in range(DURATION_SEC*env.SIM_FREQ):

        #### Step the simulation ###########################################################################
        obs, reward, done, info = env.step(action)

        #### Compute control at the desired frequency @@@@@#################################################       
        if i%CTRL_EVERY_N_STEPS==0:

            #### Compute control for the current waypoint ######################################################
            for j in range(NUM_DRONES): 
                action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP, \
                                                                            state=obs[str(j)]["state"], \
                                                                            target_pos=np.hstack([ TARGET_POS[wp_counters[j],:], INIT_XYZS[j,2] ]))

            #### Go to the next waypoint and loop ##############################################################
            for j in range(NUM_DRONES): wp_counters[j] = wp_counters[j] + 1 if wp_counters[j]<(NUM_WP-1) else 0

        #### Log the simulation ############################################################################
        for j in range(NUM_DRONES): logger.log(drone=j, timestamp=i/env.SIM_FREQ, state= obs[str(j)]["state"], control=np.hstack([ TARGET_POS[wp_counters[j],:], INIT_XYZS[j,2], np.zeros(9) ]))   
        
        #### Printout ######################################################################################
        env.render()

        #### Sync the simulation ###########################################################################
        if GUI: sync(i, START, env.TIMESTEP)
   
    #### Close the environment #########################################################################
    env.close()

    #### Save the simulation results ###################################################################
    logger.save()

    #### Plot the simulation results ###################################################################
    logger.plot()
