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

GUI = False
RECORD_VIDEO = False
TRACE_FILE = "example_trace.pkl"
PHYSICS = Physics.PYB

if __name__ == "__main__":

    #### Load a trace and control reference from a .pkl file ###########################################
    with open(os.path.dirname(os.path.abspath(__file__))+"/../files/"+TRACE_FILE, 'rb') as in_file:
        TRACE_TIMESTAMPS, TRACE_DATA, TRACE_CTRL_REFERENCE, _, _, _ = pickle.load(in_file)
    
    #### Compute trace's parameters ####################################################################
    DURATION_SEC = int(TRACE_TIMESTAMPS[-1]); SIMULATION_FREQ_HZ = int(len(TRACE_TIMESTAMPS)/TRACE_TIMESTAMPS[-1])
    
    #### Initialize the simulation #####################################################################
    env = Aviary(drone_model=DroneModel.CF2X, initial_xyzs=np.array([0,0,.1]).reshape(1,3), \
                    physics=PHYSICS, freq=SIMULATION_FREQ_HZ, gui=GUI, record=RECORD_VIDEO, obstacles=False)
    INITIAL_STATE = env.reset(); action = np.zeros(4); pos_err = 9999.

    #### Assuming TRACE_FILE starts at position [0,0,0] and the sim starts at [0,0,INITIAL_STATE[2]] ###
    TRACE_CTRL_REFERENCE[:,2] = INITIAL_STATE[2]
    
    #### Initialize the logger #########################################################################
    logger = Logger(simulation_freq_hz=SIMULATION_FREQ_HZ, num_drones=2)

    #### Initialize the controller #####################################################################
    ctrl = Control(env, control_type=ControlType.PID)
    
    #### Run the comparison ############################################################################
    START = time.time()
    for i in range(DURATION_SEC*env.SIM_FREQ):

        #### Step the simulation ###########################################################################
        obs, reward, done, info = env.step(action)

        #### Compute the next action using the set points from the trace file ##############################
        action, pos_err, yaw_err = ctrl.computeControlFromState(control_timestep=env.TIMESTEP, state=obs, \
                                                                target_pos=TRACE_CTRL_REFERENCE[i,0:3], target_vel=TRACE_CTRL_REFERENCE[i,3:6])


        #### Re-arrange the trace for consistency with the logger  #########################################
        trace_obs = np.hstack([TRACE_DATA[i,0:3], np.zeros(4), TRACE_DATA[i,6:9], TRACE_DATA[i,3:6], TRACE_DATA[i,9:12], TRACE_DATA[i,12:16]])

        #### Log the trace ################################################################################# 
        logger.log(drone=0, timestamp=TRACE_TIMESTAMPS[i], state=trace_obs, control=np.hstack([TRACE_CTRL_REFERENCE[i,:], np.zeros(6)]))
        
        #### Log the simulation ############################################################################ 
        logger.log(drone=1, timestamp=i/env.SIM_FREQ, state=obs, control=np.hstack([TRACE_CTRL_REFERENCE[i,:], np.zeros(6)]))
        
        #### Printout ######################################################################################
        if i%env.SIM_FREQ==0: env.render()

        #### Sync the simulation ###########################################################################
        if GUI: sync(i, START, env.TIMESTEP)

    #### Close the environment #########################################################################
    env.close()

    #### Save the simulation results ###################################################################
    logger.save()

    #### Plot the simulation results ###################################################################
    logger.plot(pwm=True)

