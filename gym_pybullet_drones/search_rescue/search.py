import time
import argparse
import numpy as np
import pybullet as p
from dataclasses import dataclass

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
import utils
from grid_map import GridMap
TRAJECT_PERIOD = 20

@dataclass
class SimulationConfig:
    drone: DroneModel
    num_drones: int
    physics: Physics
    gui: bool
    record_video: bool
    plot: bool
    user_debug_gui: bool
    obstacles: bool
    simulation_freq_hz: int
    control_freq_hz: int
    duration_sec: int
    output_folder: str
    colab: bool

def initialize_positions(num_drones):
    initial_height = 0
    height_increment = 0
    circle_radius = 1
    angle_offset = np.pi / num_drones

    positions = np.array([
        [
            circle_radius * np.cos((i / num_drones) * 2 * np.pi + angle_offset),
            circle_radius * np.sin((i / num_drones) * 2 * np.pi + angle_offset),
            initial_height + i * height_increment
        ]
        for i in range(num_drones)
    ])

    orientations = np.array([
        [0, 0, i * (np.pi / 2) / num_drones]
        for i in range(num_drones)
    ])

    return positions, orientations

def initialize_trajectory(control_frequency_hz, trajectory_period, initial_positions, num_drones):
    num_waypoints = control_frequency_hz * trajectory_period
    circle_radius = 1
    angle_offset = 2 * np.pi / num_drones

    target_positions = np.array([
        [
            circle_radius * np.cos((i / num_waypoints) * (2 * np.pi) + angle_offset) + initial_positions[0, 0],
            circle_radius * np.sin((i / num_waypoints) * (2 * np.pi) + angle_offset) + initial_positions[0, 1],
            0.3
        ]
        for i in range(num_waypoints)
    ])

    waypoint_counters = np.array([
        int((i * num_waypoints / num_drones) % num_waypoints)
        for i in range(initial_positions.shape[0])
    ])

    return target_positions, waypoint_counters

def run(config: SimulationConfig):
    INIT_XYZS, INIT_RPYS = initialize_positions(config.num_drones)
    TARGET_POS, wp_counters = initialize_trajectory(config.control_freq_hz, TRAJECT_PERIOD, INIT_XYZS, config.num_drones)
    
    env = CtrlAviary(
        drone_model=config.drone, num_drones=config.num_drones, initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS, physics=config.physics, neighbourhood_radius=10,
        pyb_freq=config.simulation_freq_hz, ctrl_freq=config.control_freq_hz, gui=config.gui,
        record=config.record_video, obstacles=config.obstacles, user_debug_gui=config.user_debug_gui
    )
    PYB_CLIENT = env.getPyBulletClient()
    #logger = Logger(config.control_freq_hz, config.num_drones, config.output_folder, config.colab)
    
    controllers = [DSLPIDControl(config.drone) for _ in range(config.num_drones)]
    action = np.zeros((config.num_drones, 4))
    
    # draw aditional objects
    #utils.drawSquare(p)
    #grid_map = GridMap(4, 4, p)
    #grid_map.draw_map()
    
    START = time.time()
    for i in range(int(config.duration_sec * env.CTRL_FREQ)):
        obs, _, _, _, _ = env.step(action)
        
        for j in range(config.num_drones):
            action[j, :], _, _ = controllers[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP, state=obs[j],
                target_pos=np.hstack(TARGET_POS[wp_counters[j], :3]),
                target_rpy=INIT_RPYS[j, :]
            )
        
        wp_counters = (wp_counters + 1) % len(TARGET_POS)
        #for j in range(config.num_drones):
        #    logger.log(j, i / env.CTRL_FREQ, obs[j],
        #               np.hstack([TARGET_POS[wp_counters[j], :2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)]))
        
        # Get mouse click position (returns x, y screen coordinates)
        mouse_events = p.getMouseEvents()

        for event in mouse_events:
            event_type, mouse_x, mouse_y, button_index, button_state = event

            if event_type == 2 and button_index == 0:  # Click detected and Left button
                # Get the mouse position (in screen coordinates)
                print(f"Mouse click position (screen)->: ({mouse_x}, {mouse_y})")
            
        # prints info drones in terminal
        #env.render()

        if config.gui:
            sync(i, START, env.CTRL_TIMESTEP)
    
    env.close()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone', type=str, default="cf2x")
    parser.add_argument('--num_drones', type=int, default=4)
    parser.add_argument('--physics', type=str, default="pyb_gnd_drag_dw")
    parser.add_argument('--gui', type=str2bool, default=True)
    parser.add_argument('--record_video', type=str2bool, default=False)
    parser.add_argument('--plot', type=str2bool, default=False)
    parser.add_argument('--user_debug_gui', type=str2bool, default=False)
    parser.add_argument('--obstacles', type=str2bool, default=False)
    parser.add_argument('--simulation_freq_hz', type=int, default=240)
    parser.add_argument('--control_freq_hz', type=int, default=48)
    parser.add_argument('--duration_sec', type=int, default=9999999)
    parser.add_argument('--output_folder', type=str, default='results')
    parser.add_argument('--colab', type=str2bool, default=False)
    args = parser.parse_args()
    
    config = SimulationConfig(
        drone=DroneModel(args.drone),
        num_drones=args.num_drones,
        physics=Physics(args.physics),
        gui=args.gui,
        record_video=args.record_video,
        plot=args.plot,
        user_debug_gui=args.user_debug_gui,
        obstacles=args.obstacles,
        simulation_freq_hz=args.simulation_freq_hz,
        control_freq_hz=args.control_freq_hz,
        duration_sec=args.duration_sec,
        output_folder=args.output_folder,
        colab=args.colab,
    )
    
    run(config)
