import os
import time
import argparse
from datetime import datetime
import pdb
import math
import numpy as np
import pybullet as p
import pickle
import matplotlib.pyplot as plt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import ray
from ray import tune
from ray.tune import register_env
from ray.rllib.agents import ppo
from ray.rllib.agents.ppo import PPOTrainer, PPOTFPolicy
from ray.rllib.examples.policy.random_policy import RandomPolicy
from ray.rllib.utils.test_utils import check_learning_achieved


from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics

from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary


from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--file',      type=str,       help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()

    #### Restore agent #################################################################################
    agent = ppo.PPOTrainer(config=config); agent.restore(ARGS.file)

    #### Extract and print policies ####################################################################
    policy0 = agent.get_policy("pol0")
    policy1 = agent.get_policy("pol1")
    policy2 = agent.get_policy("pol2")
    print(policy0.model.base_model.summary())
    print(policy1.model.base_model.summary())
    print(policy2.model.base_model.summary())

    #### Create test environment ########################################################################
    env = FlockAviary(num_drones=ARGS.num_drones, gui=True, record=False, obstacles=True)
    obs = env.reset()
    action = { str(i): np.array([0,0,0,0]) for i in range(ARGS.num_drones) }
    start = time.time()
    for i in range(10*env.SIM_FREQ):

        #### Deploy the policies ###########################################################################
        print("Debug Obs", obs)
        temp = {}
        temp["0"] = policy0.compute_single_action(np.hstack([ obs["0"]["state"], obs["0"]["neighbors"] ]))
        temp["1"] = policy1.compute_single_action(np.hstack([ obs["1"]["state"], obs["1"]["neighbors"] ]))
        temp["2"] = policy2.compute_single_action(np.hstack([ obs["2"]["state"], obs["2"]["neighbors"] ]))
        print("Debug Act", temp)
        action = {"0": temp["0"][0], "1": temp["1"][0], "2": temp["2"][0]}

        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        if done["__all__"]: obs = env.reset()
    env.close()

    #### Shut down Ray #################################################################################
    ray.shutdown()










