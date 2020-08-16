import os
import time
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
from ray.rllib.agents import ppo

from utils import *
from gym_pybullet_drones.envs.SingleDroneEnv import DroneModel, SingleDroneEnv

RLLIB = False

if __name__ == "__main__":

    ####################################################################################################
    #### Check out the environment's spaces ############################################################
    ####################################################################################################
    env = gym.make("single-drone-v0")
    print("Action space:", env.action_space)
    print("Observation space:", env.observation_space)
    check_env(env, warn=True, skip_render_check=True) 

    ####################################################################################################
    #### Train the model ###############################################################################
    ####################################################################################################
    if not RLLIB:
        model = A2C(MlpPolicy, env, verbose=1)
        model.learn(total_timesteps=500000)
    else:
        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 0
        agent = ppo.PPOTrainer(config, env="single-drone-v0")
        for i in range(100):
            results = agent.train()
            print("{:d}: episode_reward max {:f} min {:f} mean {:f}".format(i, results["episode_reward_max"], results["episode_reward_min"], results["episode_reward_mean"]))
        policy = agent.get_policy()
        print(policy.model.base_model.summary())

    ####################################################################################################
    #### Show (and record a video of) the model's performance ##########################################
    ####################################################################################################
    env = SingleDroneEnv(normalized_spaces=True, gui=True, record=True)
    obs = env.reset()
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_ID = env.getDroneId()
    start = time.time()
    for i in range(10*env.SIM_FREQ):
        if not RLLIB: action, _states = model.predict(obs, deterministic=True)
        else: action, _states, _dict = policy.compute_single_action(obs)
        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        if done: obs = env.reset()
    env.close()
