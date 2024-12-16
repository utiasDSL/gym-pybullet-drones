"""
Script demonstrating a simulation with one drone and three random boxes.

The simulation is run using the `BaseAviary` environment.

Example
-------
In a terminal, run as:

    $ python create_environment2.py

Notes
-----
The environment includes three randomly sized and placed boxes as obstacles.
"""

import os
import time
import random
import numpy as np
import pybullet as p
import environment_setup as env_set

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync


def run_simulation():
    """Run the simulation with one drone and three random obstacles."""
    # Initialize drone position and orientation
    INIT_XYZS = np.array([[0, 0, 1]])  # Initial position for the single drone
    INIT_RPYS = np.array([[0, 0, 0]])  # Initial orientation for the single drone

    # Create the environment with obstacles enabled
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=Physics.PYB,
        neighbourhood_radius=10,
        pyb_freq=240,
        ctrl_freq=48,
        gui=True,
        record=False,
        obstacles=True,  # Enable obstacles
        user_debug_gui=False,
    )

    # Initialize the logger
    logger = Logger(logging_freq_hz=48, num_drones=1, output_folder="results")

    # Run the simulation loop
    action = np.zeros((1, 4))  # Hover action
    duration_sec = 100  # Run the simulation for 10 seconds
    start_time = time.time()

    env_setup = env_set.Environment(client=env.getPyBulletClient())
    env_setup.add_obstacle("cube")

    for step in range(0, int(duration_sec * env.CTRL_FREQ)):
        # Step the simulation
        obs, reward, terminated, truncated, info = env.step(action)

        # Log data
        control = np.hstack([INIT_XYZS[0], INIT_RPYS[0], np.zeros(6)])  # Shape (12,)
        logger.log(drone=0, timestamp=step / env.CTRL_FREQ, state=obs[0], control=control)

        # Render the simulation
        env.render()

        # Sync simulation to real-time
        sync(step, start_time, env.CTRL_TIMESTEP)


    # Close the environment
    env.close()

    # Save the results
    logger.save()  
    logger.plot()

if __name__ == "__main__":
    run_simulation()
