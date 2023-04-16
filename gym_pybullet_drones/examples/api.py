"""Script demonstrating how to use the API/spin an environment.

Example
-------
In a terminal, run as:

    $ python api.py

Notes
-----
TBD.

"""
import time
import argparse
import gymnasium as gym
import numpy as np
# from stable_baselines3 import A2C
# from stable_baselines3.a2c import MlpPolicy
# from stable_baselines3.common.env_checker import check_env
# import ray
# from ray.tune import register_env
# from ray.rllib.agents import ppo

from gym_pybullet_drones.utils.Logger import Logger
# from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.utils.utils import sync, str2bool

# DEFAULT_RLLIB = False
DEFAULT_GUI = True
# DEFAULT_RECORD_VIDEO = False
DEFAULT_OUTPUT_FOLDER = 'results'
# DEFAULT_COLAB = False

def run(
        gui=DEFAULT_GUI,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=False,
        # rllib=DEFAULT_RLLIB,output_folder=DEFAULT_OUTPUT_FOLDER, gui=DEFAULT_GUI, plot=True, colab=DEFAULT_COLAB, record_video=DEFAULT_RECORD_VIDEO
        ):

    #### Check the environment's spaces ########################
    env = gym.make("hover-aviary-v0")
    print("[INFO] Action space:", env.action_space)
    print("[INFO] Observation space:", env.observation_space)
    # check_env(env,
    #           warn=True,
    #           skip_render_check=True
    #           )

    observation, info = env.reset() # (seed=42)
    for _ in range(1000):
        action = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            observation, info = env.reset()
    env.close()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='API example for `gym-pybullet-drones` environments')
    # parser.add_argument('--rllib',      default=DEFAULT_RLLIB,        type=str2bool,       help='Whether to use RLlib PPO in place of stable-baselines A2C (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    # parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    # parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
