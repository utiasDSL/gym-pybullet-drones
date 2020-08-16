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
from gym_pybullet_drones.control.Control import ControlType, Control

GUI = False
RECORD_VIDEO = False
SAVE_TO_FILE = True
TRACE_FILE = "trace.pkl"

if __name__ == "__main__":

    ####################################################################################################
    #### Load a trace and control reference from a .pkl file ###########################################
    ####################################################################################################
    with open(os.path.dirname(os.path.abspath(__file__))+"/../assets/traces/"+TRACE_FILE, 'rb') as in_file:
        trace_timestamps, trace_data, trace_control_reference, _, trace_error, trace_rmse = pickle.load(in_file)
    SIMULATION_FREQ_HZ = int(len(trace_timestamps)/trace_timestamps[-1])
    DURATION_SEC = int(trace_timestamps[-1])
    trace_control_reference = np.array(trace_control_reference)
    
    ####################################################################################################
    #### Initialize the simulation #####################################################################
    ####################################################################################################
    start = time.time()
    env = SingleDroneEnv(drone_model=DroneModel.CF2X, pybullet=True, aero_effects=False, normalized_spaces=False, freq=SIMULATION_FREQ_HZ, gui=GUI, obstacles=False, record=RECORD_VIDEO)
    initial_state = env.reset()
    action = np.zeros(4); pos_err = 9999.
    trace_control_reference[:,2] = initial_state[2]         # i.e drone starts at (0, 0, initial_state[2])
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
        #### Compute the next action using the set points from the trace file ##############################
        ####################################################################################################
        action, pos_err, yaw_err = ctrl.computeControl(control_timestep=env.TIMESTEP, cur_pos=state[0:3], cur_quat_rpy=state[3:7], cur_vel=state[10:13], cur_ang_vel=state[13:16], \
                                    target_pos=trace_control_reference[i,0:3], target_rpy=np.array([0,0,0]), target_vel=trace_control_reference[i,3:6])

        ####################################################################################################
        #### Log the simulation ############################################################################
        #################################################################################################### 
        simulation_timestamps[i] = i/env.SIM_FREQ
        simulation_data[i,:] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        simulation_control_reference[i,:] = np.hstack([trace_control_reference[i,:], np.zeros(3), np.zeros(3)])
        
        ####################################################################################################
        #### Printout and sync #############################################################################
        ####################################################################################################
        env.render()
        if GUI: sync(i, start, env.TIMESTEP)

    ####################################################################################################
    #### Close the environment #########################################################################
    ####################################################################################################
    env.close()

    ####################################################################################################
    #### Save the simulation results ###################################################################
    ####################################################################################################
    if SAVE_TO_FILE:
        with open(os.path.dirname(os.path.abspath(__file__))+"/../assets/saves/save-trace-comparison-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".npy", 'wb') as out_file:
            np.save(out_file, simulation_timestamps); np.save(out_file, simulation_data); np.save(out_file, simulation_control_reference)

    ####################################################################################################
    #### Plot the simulation results ###################################################################
    ####################################################################################################
    min_last_step = min(trace_data.shape[0],simulation_data.shape[0]) 
    simulation_plot_data = simulation_data[:min_last_step,:]
    trace_plot_data = trace_data[:min_last_step,:]
    fig, axs = plt.subplots(8,2)
    t = np.arange(0, DURATION_SEC, 1/SIMULATION_FREQ_HZ)
    labels = ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'PWM1', 'PWM2', 'PWM3', 'PWM4']
    for i in range(16):
        axs[i%8,i//8].plot(t, trace_plot_data[:,i], '-b', label='trace')
        if i > 11:        # Comparison to CF2's PWM instead of RPM for file trace_1.pkl
            simulation_plot_data[:,i] = (simulation_plot_data[:,i] - 4070.3) / 0.2685
        axs[i%8,i//8].plot(t, simulation_plot_data[:,i], '--r', label='sim.')
        axs[i%8,i//8].set_xlabel('time')
        axs[i%8,i//8].set_ylabel(labels[i])
        axs[i%8,i//8].grid(True)
        axs[i%8,i//8].legend(loc='upper right', frameon=True)
    fig.subplots_adjust(left=0.06, bottom=0.05, right=0.99, top=0.98, wspace=0.15, hspace=0.0)
    plt.show()

