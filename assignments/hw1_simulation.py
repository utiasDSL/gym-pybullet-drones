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

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

from hw1_control import HW1Control

if __name__ == "__main__":

    #### Create the environment ########################################################################
    GUI = True # set to False to disable rendering
    env = CtrlAviary(gui=GUI)  

    #### Initialize the logger #########################################################################
    logger = Logger(logging_freq_hz=env.SIM_FREQ)

    #### Initialize the controller #####################################################################
    ctrl = HW1Control(env)

    #### Initialize the action #########################################################################
    action = {}
    obs = env.reset()
    state = obs["0"]["state"]
    action["0"] = ctrl.computeControl(current_position=state[0:3], 
                                        current_quaternion=state[3:7], 
                                        current_velocity=state[10:13], 
                                        current_angular_velocity=state[13:16],
                                        target_position=np.zeros(3), 
                                        target_velocity=np.zeros(3)
                                        )

    #### Run the simulation ############################################################################
    DURATION = 10
    START = time.time()
    for i in range(0, DURATION*env.SIM_FREQ):

        #### Step the simulation ###########################################################################
        obs, _, _, _ = env.step(action)

        #### Compute control ###############################################################################
        state = obs["0"]["state"]
        action["0"] = ctrl.computeControl(current_position=state[0:3], 
                                            current_quaternion=state[3:7], 
                                            current_velocity=state[10:13], 
                                            current_angular_velocity=state[13:16],
                                            target_position=np.zeros(3), 
                                            target_velocity=np.zeros(3) 
                                            )

        #### Log the simulation ############################################################################
        logger.log(drone=0, timestamp=i/env.SIM_FREQ, state=obs["0"]["state"])

        #### Printout ######################################################################################
        if i%env.SIM_FREQ==0: env.render()

        #### Sync the simulation ###########################################################################
        if GUI: sync(i, START, env.TIMESTEP) 

    #### Close the environment #########################################################################
    env.close()

    #### Plot the simulation results ###################################################################
    logger.plot()










