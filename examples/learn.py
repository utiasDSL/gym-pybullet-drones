import os
import time
import argparse
import pdb
import math
import numpy as np
import pybullet as p
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from stable_baselines3 import PPO
from stable_baselines3 import A2C
from stable_baselines3.a2c import MlpPolicy
from stable_baselines3.common.env_checker import check_env
import ray
from ray import tune
from ray.tune import register_env
from ray.rllib.agents import ppo

from utils import *
from gym_pybullet_drones.envs.RLTakeoffAviary import RLTakeoffAviary

if __name__ == "__main__":

    #### Define, parse, and assign (optional) arguments for the script #################################
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using RLTakeoffAviary')
    parser.add_argument('--rllib',      default=False,        type=str2bool,       help='Whether to use RLlib PPO in place of stable-baselines A2C (default: False)', metavar='')
    namespace = parser.parse_args(); RLLIB = namespace.rllib    

    #### Check the environment's spaces ################################################################
    env = gym.make("rl-takeoff-aviary-v0")
    print("[INFO] Action space:", env.action_space)
    print("[INFO] Observation space:", env.observation_space)
    check_env(env, warn=True, skip_render_check=True) 

    #### Train the model ###############################################################################
    if not RLLIB:
        model = A2C(MlpPolicy, env, verbose=1)
        model.learn(total_timesteps=500000) # 500000
    else:
        ray.shutdown(); ray.init(ignore_reinit_error=True)
        register_env("rl-takeoff-aviary-v0", lambda _: RLTakeoffAviary())
        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 2
        config["env"] = "rl-takeoff-aviary-v0"
        agent = ppo.PPOTrainer(config)
        for i in range(100): # 100
            results = agent.train()
            print("[INFO] {:d}: episode_reward max {:f} min {:f} mean {:f}".format(i, \
                    results["episode_reward_max"], results["episode_reward_min"], results["episode_reward_mean"]))
        policy = agent.get_policy()
        print(policy.model.base_model.summary())
        ray.shutdown()

    #### Show (and record a video of) the model's performance ##########################################
    env = RLTakeoffAviary(gui=True, record=True)
    obs = env.reset()
    start = time.time()
    for i in range(10*env.SIM_FREQ):
        if not RLLIB: action, _states = model.predict(obs, deterministic=True)
        else: action, _states, _dict = policy.compute_single_action(obs)
        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        print()
        print(done)
        print()
        if done: obs = env.reset()
    env.close()
