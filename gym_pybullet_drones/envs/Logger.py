import os
import time
from datetime import datetime
from cycler import cycler
import numpy as np
import matplotlib.pyplot as plt


######################################################################################################################################################
#### Logger class ####################################################################################################################################
######################################################################################################################################################
class Logger(object):

    ####################################################################################################
    #### Initialize the logger #########################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - duration_sec (int)               duration of the simulation in seconds ######################
    #### - simulation_freq_hz (int)         simulation frequency in Hz #################################
    #### - num_drones (int)                 number of drones ###########################################
    ####################################################################################################
    def __init__(self, duration_sec: int, simulation_freq_hz: int, num_drones: int=1):
        self.DURATION_SEC = duration_sec; self.SIMULATION_FREQ_HZ = simulation_freq_hz
        self.NUM_DRONES = num_drones; self.NUM_STEPS = self.DURATION_SEC * self.SIMULATION_FREQ_HZ
        self.counters = np.zeros(num_drones)
        self.timestamps = np.zeros((num_drones, self.NUM_STEPS))
        self.states = np.zeros((num_drones, self.NUM_STEPS, 16)) #### 16 states: pos_x, pos_y, pos_z, 
                                                                                # vel_x, vel_y, vel_z, 
                                                                                # roll, pitch, yaw, 
                                                                                # ang_vel_x, ang_vel_y, ang_vel_z, 
                                                                                # rpm0, rpm1, rpm2, rpm3
        #### Note: this is not the same order nor length returned in obs["i"]["state"] by Aviary.step() ####
        self.controls = np.zeros((num_drones, self.NUM_STEPS, 12)) #### 12 control targets: pos_x, pos_y, pos_z,
                                                                                            # vel_x, vel_y, vel_z, 
                                                                                            # roll, pitch, yaw, 
                                                                                            # ang_vel_x, ang_vel_y, ang_vel_z

    ####################################################################################################
    #### Log entries for a single simulation step, of a single drone ###################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - drone (int)                      drone associated to the log entry ##########################
    #### - timestamp (float)                timestamp of the log in simulation clock ###################
    #### - state ((20,1) array)             drone's state ##############################################
    #### - control ((12,1) array)           drone's control target #####################################
    ####################################################################################################
    def log(self, drone: int, timestamp, state, control):
        current_counter = int(self.counters[drone])
        if drone<0 or drone>=self.NUM_DRONES or timestamp<0 or timestamp>self.DURATION_SEC \
            or len(state)!=20 or len(control)!=12 or current_counter>=self.NUM_STEPS: print("[ERROR] in Logger.log(), invalid data")
        self.timestamps[drone, current_counter] = timestamp
        self.states[drone, current_counter,:] = np.hstack([ state[0:3], state[10:13], state[7:10], state[13:20] ])
        self.controls[drone, current_counter,:] = control
        self.counters[drone] += 1

    ####################################################################################################
    #### Save the logs to file #########################################################################
    ####################################################################################################
    def save(self):
        with open(os.path.dirname(os.path.abspath(__file__))+"/../../files/save-flight-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".npy", 'wb') as out_file:
            np.save(out_file, self.timestamps); np.save(out_file, self.states); np.save(out_file, self.controls)

    ####################################################################################################
    #### Plot the logged (state) data ##################################################################
    ####################################################################################################
    #### Arguments #####################################################################################
    #### - pwm (bool)                       if True, convert logged RPM into PWM values ################
    ####################################################################################################
    def plot(self, pwm=False):
        #### Loop over colors and line styles ##############################################################
        plt.rc('axes', prop_cycle=(cycler('color', ['r', 'g', 'b', 'y']) + cycler('linestyle', ['-', '--', ':', '-.'])))
        fig, axs = plt.subplots(8,2)
        t = np.arange(0, self.DURATION_SEC, 1/self.SIMULATION_FREQ_HZ)
        labels = ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'PWM0', 'PWM1', 'PWM2', 'PWM3'] if pwm else ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'RPM0', 'RPM1', 'RPM2', 'RPM3']
        for i in range(16):
            for j in range(self.NUM_DRONES):
                #### This line converts RPM into PWM for all drones except drone_0 (used in compare.py) ############
                if pwm and i>11 and j>0: self.states[j,:,i] = (self.states[j,:,i] - 4070.3) / 0.2685
                axs[i%8,i//8].plot(t, self.states[j,:,i], label="drone_"+str(j))
            axs[i%8,i//8].set_xlabel('time')
            axs[i%8,i//8].set_ylabel(labels[i])
            axs[i%8,i//8].grid(True)
            axs[i%8,i//8].legend(loc='upper right', frameon=True)
        fig.subplots_adjust(left=0.06, bottom=0.05, right=0.99, top=0.98, wspace=0.15, hspace=0.0)
        plt.show()

