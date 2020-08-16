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
from gym_pybullet_drones.envs.SingleDroneEnv import DroneModel, SingleDroneEnv
from gym_pybullet_drones.envs.MultiDroneEnv import MultiDroneEnv
from gym_pybullet_drones.control.Control import ControlType, Control

DRONE = DroneModel.CF2X
GUI = True
RECORD_VIDEO = True
SAVE_TO_FILE = True
SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48
DURATION_SEC = 10
NUM_RESTARTS = 0

if __name__ == "__main__":

    ####################################################################################################
    #### Alternatively, initialize a 1m by 1m, 41-waypoint trajectory ##################################
    ####################################################################################################
    zeros = np.zeros(20); ones = -1*np.ones(20); zero2one = np.array([-x/20-0.05 for x in range(20)]); one2zero = np.flip(zero2one)
    waypoints = (np.vstack([np.hstack([0,zero2one,ones,one2zero,zeros]), np.hstack([0,zeros,zero2one,ones,one2zero]),.3*np.ones(81)])).transpose()
    wp_counter = 0; current_wp = waypoints[wp_counter]
    
    ####################################################################################################
    #### Initialize the simulation #####################################################################
    ####################################################################################################
    start = time.time()
    env = SingleDroneEnv(drone_model=DRONE, pybullet=True, aero_effects=False, normalized_spaces=False, freq=SIMULATION_FREQ_HZ, gui=GUI, obstacles=True, record=RECORD_VIDEO)
    initial_state = env.reset()
    PYB_CLIENT = env.getPyBulletClient(); DRONE_ID = env.getDroneId() # Use PYB_CLIENT, DRONE_ID to apply additional forces, if desired
    action = np.zeros(4); pos_err = 9999.
    control_every_n_steps = int(np.floor(env.SIM_FREQ/CONTROL_FREQ_HZ))
    simulation_data = np.zeros((DURATION_SEC*SIMULATION_FREQ_HZ,16))
    simulation_control_reference = np.zeros((DURATION_SEC*SIMULATION_FREQ_HZ,12))
    simulation_timestamps = np.zeros((DURATION_SEC*SIMULATION_FREQ_HZ,1))
    ####################################################################################################
    #### Setup the controller ##########################################################################
    ####################################################################################################
    ctrl = Control(env, control_type=ControlType.PID)
    for i in range(DURATION_SEC*env.SIM_FREQ):

        ####################################################################################################
        #### Step the simulation  ##########################################################################
        ####################################################################################################
        state, reward, done, info = env.step(action)

        ####################################################################################################
        #### Alternatively, navigate the waypoints in the waypoint list ####################################
        ####################################################################################################
        if i%control_every_n_steps==0:
            target_rpy=np.array([0,0,-45])*env.DEG2RAD
            action, pos_err, yaw_err = ctrl.computeControl(control_timestep=control_every_n_steps*env.TIMESTEP, cur_pos=state[0:3], cur_quat_rpy=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16], \
                                        target_pos=current_wp)                                          # Waypoints
                                        # target_pos=np.array([-.5, -1., .8]))                          # XYZ transfer
                                        # target_pos=np.array([0., 0., 0.5]), target_rpy=target_rpy)    # Hover and yaw         
            ####################################################################################################
            #### Go to the next waypoint #######################################################################
            ####################################################################################################
            if np.linalg.norm(pos_err) < 0.05 and np.linalg.norm(state[10:13])<0.1: 
                wp_counter = 0 if wp_counter==(len(waypoints)-1) else wp_counter+1
                current_wp = waypoints[wp_counter]

        ####################################################################################################
        #### Log the simulation ############################################################################
        #################################################################################################### 
        simulation_timestamps[i] = i/env.SIM_FREQ
        simulation_data[i,:] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        simulation_control_reference[i,:] = np.hstack([current_wp, np.zeros(3), target_rpy, np.zeros(3)])
        
        ####################################################################################################
        #### Printout and sync #############################################################################
        ####################################################################################################
        env.render()
        if GUI: sync(i, start, env.TIMESTEP)

        ####################################################################################################
        #### Reset the simulation ##########################################################################
        ####################################################################################################
        if i>1 and i%((DURATION_SEC/(NUM_RESTARTS+1))*env.SIM_FREQ)==0: initial_state = env.reset(); wp_counter = 0

    ####################################################################################################
    #### Close the environment #########################################################################
    ####################################################################################################
    env.close()

    ####################################################################################################
    #### Save the simulation results ###################################################################
    ####################################################################################################
    if SAVE_TO_FILE:
        with open(os.path.dirname(os.path.abspath(__file__))+"/../files/saves/save-flight-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".npy", 'wb') as out_file:
            np.save(out_file, simulation_timestamps); np.save(out_file, simulation_data); np.save(out_file, simulation_control_reference)

    ####################################################################################################
    #### Plot the simulation results ###################################################################
    ####################################################################################################
    min_last_step = simulation_data.shape[0]
    simulation_plot_data = simulation_data[:min_last_step,:]
    fig, axs = plt.subplots(8,2)
    t = np.arange(0, DURATION_SEC, 1/SIMULATION_FREQ_HZ)
    labels = ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'RPM1', 'RPM2', 'RPM3', 'RPM4']
    for i in range(16):
        axs[i%8,i//8].plot(t, simulation_plot_data[:,i], '--r', label='sim.')
        axs[i%8,i//8].set_xlabel('time')
        axs[i%8,i//8].set_ylabel(labels[i])
        axs[i%8,i//8].grid(True)
        axs[i%8,i//8].legend(loc='upper right', frameon=True)
    fig.subplots_adjust(left=0.06, bottom=0.05, right=0.99, top=0.98, wspace=0.15, hspace=0.0)
    plt.show()

