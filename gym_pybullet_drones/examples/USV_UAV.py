"""Script demonstrating the joint use of velocity input.

The simulation is run by a `VelocityAviary` environment.

Example
-------
In a terminal, run as:

    $ python pid_velocity.py

Notes
-----
The drones use interal PID control to track a target velocity.

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
import autograd.numpy as np
from autograd import grad
from gym_pybullet_drones.examples.USV_trajectory import USV_trajectory
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from scipy.optimize import minimize
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary

DEFAULT_DRONE = DroneModel("cf2x")
DEFAULT_GUI = False
DEFAULT_RECORD_VIDEO = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 300
DEFAULT_CONTROL_FREQ_HZ = 300
DEFAULT_DURATION_SEC = 100
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
NUM_DRONE = 2

from dataclasses import dataclass, field
from functools import cache
from typing import Callable
@dataclass(frozen=True)
class TimeData:
  T: float # длительность во времени
  fs: int # частота дискретизации
  n: int = field(init=False)# число отсчетов
  dt: float = field(init=False) # длительность отсчета времени
  t: np.ndarray = field(init=False) # отсчеты времени

  def __post_init__(self):
    object.__setattr__(self, 'n', int(self.T * self.fs))
    object.__setattr__(self, 'dt', 1/self.fs)
    object.__setattr__(self, 't', np.arange(self.n) * self.dt)

  def sample(self, fs):
    return TimeData(T=self.T, fs=fs)

def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB,
        num_drone=NUM_DRONE
        ):
        #### Initialize the simulation #############################
    INIT_XYZS = np.array([
                          [0, 10, 10],
                          [0, 50, 10]
                          ])
    INIT_RPYS = np.array([
                          [0, 0, 0],
                          [0, 0, np.pi/3]
                          ])

    r1 = np.array([[0, 0], [0, 20], [0, 40], [0, 60]])
    xyz1 = np.array([[0, 0, 0], [0, 20, 0], [0, 40, 0], [0, 60, 0]])
    #### Create the environment ################################
    env = VelocityAviary(drone_model=drone,
                         num_drones=num_drone,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=Physics.PYB,
                         neighbourhood_radius=10,
                         pyb_freq=simulation_freq_hz,
                         ctrl_freq=control_freq_hz,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )

    #### Compute number of control steps in the simlation ######
    time_data = TimeData(duration_sec, simulation_freq_hz)
    PERIOD = duration_sec
    NUM_WP = control_freq_hz*PERIOD
    wp_counters = np.array([0 for i in range(4)])
    trajs = USV_trajectory(time_data, m=4, r0=r1, xyz0=xyz1)

    #### Initialize the velocity target ############
    TARGET_VEL = np.zeros((num_drone, NUM_WP, 4))

    def loss_function_n(x, usv_coord):
        norm = np.linalg.norm(x[:, :, None] - x[:, None], axis=-1) ** 2
        uav_sum_dist = np.sum(norm.reshape(norm.shape[0], -1), axis=1) / 2
        uav_usv_sum_dist = np.sum(np.min(np.linalg.norm(x[:, :, None] - usv_coord[:, None], axis=-1), axis=1) ** 2,
                                  axis=1)
        return uav_usv_sum_dist + 0.05 * uav_sum_dist

    #### Initialize the logger #################################0
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drone,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Run the simulation ####################################
    action = np.zeros((num_drone, 4))
    START = time.time()
    usv_coord = trajs.xyz
    opt_x = np.zeros((usv_coord.shape[0], num_drone, 3))
    opt_x[0] = INIT_XYZS
    for i in range(1, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)
        function = lambda x: loss_function_n(x.reshape(1, num_drone, 3), usv_coord[i, :, :].reshape(1, 4, 3))
        optimized = minimize(function, opt_x[i-1].reshape(1, -1))
        opt_x[i] = optimized.x.reshape(num_drone, 3)

        #### Compute control for the current way point #############

        d_err = opt_x[i] - np.transpose(np.array([obs[:, 0], obs[:, 1], obs[:, 2]]), (1, 0))

        for j in range(num_drone):
            TARGET_VEL[j, i, :] = [d_err[j, 0], d_err[j, 1], 0, 3]
            action[j, :] = TARGET_VEL[j, i, :]

        #### Go to the next way point and loop #####################
        for j in range(num_drone):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drone):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_VEL[j, wp_counters[j], 0:3], np.zeros(9)])
                       )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Plot the simulation results ###########################
    #logger.save_as_csv("vel") # Optional CSV save
    if plot:
        logger.plot_trajct(trajs=trajs)

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Velocity control example using VelocityAviary')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,      type=str2bool,      help='Whether to add obstacles to the environment (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))

