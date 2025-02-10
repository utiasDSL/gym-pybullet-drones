import os
import time
import argparse
import numpy as np
import pybullet as p
from datetime import datetime
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

class DroneSimulation:
    def __init__(self, **kwargs):
        self.drone_model = kwargs.get("drone", DroneModel("cf2x"))
        self.num_drones = kwargs.get("num_drones", 5)
        self.physics = kwargs.get("physics", Physics("pyb_gnd_drag_dw"))
        self.gui = kwargs.get("gui", True)
        self.record_video = kwargs.get("record_video", False)
        self.plot = kwargs.get("plot", False)
        self.user_debug_gui = kwargs.get("user_debug_gui", False)
        self.obstacles = kwargs.get("obstacles", True)
        self.simulation_freq_hz = kwargs.get("simulation_freq_hz", 240)
        self.control_freq_hz = kwargs.get("control_freq_hz", 48)
        self.duration_sec = kwargs.get("duration_sec", 9999999)
        self.output_folder = kwargs.get("output_folder", "results")
        self.colab = kwargs.get("colab", False)
        self.env = None
        self.logger = None
        self.controllers = []
        self.action = np.zeros((self.num_drones, 4))
        self.init_positions()
        self.init_trajectory2() # or init_trajectory2

    def init_positions(self):
        H = 0.5
        H_STEP = 0.05
        R = 0.3
        self.init_xyzs = np.array([
            [R * np.cos((i / 6) * 2 * np.pi + np.pi / 2), R * np.sin((i / 6) * 2 * np.pi + np.pi / 2) - R, H + i * H_STEP]
            for i in range(self.num_drones)
        ])
        self.init_rpys = np.array([[0, 0, i * (np.pi / 2) / self.num_drones] for i in range(self.num_drones)])

    def init_trajectory2(self):
            PERIOD = 14
            NUM_WP = self.control_freq_hz * PERIOD
            self.target_pos = np.zeros((NUM_WP, 3))
            RADIUS = 1
            
            # Alternative trajectory (figure-eight pattern)
            for i in range(NUM_WP):
                theta = (i / NUM_WP) * (2 * np.pi)
                x = RADIUS * np.sin(2 * theta)
                y = RADIUS * np.sin(theta)
                self.target_pos[i, :] = [x, y, 0]
            
            self.wp_counters = np.array([int((i * NUM_WP / 6) % NUM_WP) for i in range(self.num_drones)])
        
    def init_trajectory(self):
        PERIOD = 10
        NUM_WP = self.control_freq_hz * PERIOD
        self.target_pos = np.zeros((NUM_WP, 3))
        for i in range(NUM_WP):
            self.target_pos[i, :] = (
                self.init_xyzs[0, 0] + 0.3 * np.cos((i / NUM_WP) * (2 * np.pi) + np.pi / 2),
                self.init_xyzs[0, 1] + 0.3 * np.sin((i / NUM_WP) * (2 * np.pi) + np.pi / 2) - 0.3,
                self.init_xyzs[0, 2],  # Use the initial height of the first drone
            )
        self.wp_counters = np.array([int((i * NUM_WP / 6) % NUM_WP) for i in range(self.num_drones)])

    def setup_environment(self):
        self.env = CtrlAviary(
            drone_model=self.drone_model,
            num_drones=self.num_drones,
            initial_xyzs=self.init_xyzs,
            initial_rpys=self.init_rpys,
            physics=self.physics,
            neighbourhood_radius=10,
            pyb_freq=self.simulation_freq_hz,
            ctrl_freq=self.control_freq_hz,
            gui=self.gui,
            record=self.record_video,
            obstacles=self.obstacles,
            user_debug_gui=self.user_debug_gui,
        )
        self.logger = Logger(
            logging_freq_hz=self.control_freq_hz,
            num_drones=self.num_drones,
            output_folder=self.output_folder,
            colab=self.colab,
        )
        self.controllers = [DSLPIDControl(drone_model=self.drone_model) for _ in range(self.num_drones)]

    def run_simulation(self):
        self.setup_environment()
        start_time = time.time()
        for i in range(0, int(self.duration_sec * self.env.CTRL_FREQ)):
            obs, _, _, _, _ = self.env.step(self.action)
            self.update_controls(obs)
            self.log_data(obs, i)
            if self.gui:
                sync(i, start_time, self.env.CTRL_TIMESTEP)
        self.env.close()
        if self.plot:
            self.logger.plot()

    def update_controls(self, obs):
        for j in range(self.num_drones):
            self.action[j, :], _, _ = self.controllers[j].computeControlFromState(
                control_timestep=self.env.CTRL_TIMESTEP,
                state=obs[j],
                target_pos=np.hstack([self.target_pos[self.wp_counters[j], 0:2], self.init_xyzs[j, 2]]),
                target_rpy=self.init_rpys[j, :],
            )
            self.wp_counters[j] = (self.wp_counters[j] + 1) % len(self.target_pos)

    def log_data(self, obs, step):
        for j in range(self.num_drones):
            self.logger.log(
                drone=j,
                timestamp=step / self.env.CTRL_FREQ,
                state=obs[j],
                control=np.hstack([self.target_pos[self.wp_counters[j], 0:2], self.init_xyzs[j, 2], self.init_rpys[j, :], np.zeros(6)]),
            )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Helix flight simulation using CtrlAviary and DSLPIDControl")
    parser.add_argument("--drone", default=DroneModel("cf2x"), type=DroneModel, choices=DroneModel)
    parser.add_argument("--num_drones", default=4, type=int)
    parser.add_argument("--physics", default=Physics("pyb_gnd_drag_dw"), type=Physics, choices=Physics)
    parser.add_argument("--gui", default=True, type=str2bool)
    parser.add_argument("--record_video", default=False, type=str2bool)
    parser.add_argument("--plot", default=False, type=str2bool)
    parser.add_argument("--user_debug_gui", default=False, type=str2bool)
    parser.add_argument("--obstacles", default=True, type=str2bool)
    parser.add_argument("--simulation_freq_hz", default=240, type=int)
    parser.add_argument("--control_freq_hz", default=48, type=int)
    parser.add_argument("--duration_sec", default=9999999, type=int)
    parser.add_argument("--output_folder", default="results", type=str)
    parser.add_argument("--colab", default=False, type=bool)
    args = parser.parse_args()

    sim = DroneSimulation(**vars(args))
    sim.run_simulation()