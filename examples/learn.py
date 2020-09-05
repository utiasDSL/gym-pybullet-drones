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
from ray.tune import register_env
from ray.rllib.agents import ppo

from utils import *
from gym_pybullet_drones.envs.Aviary import Aviary, DroneModel
from gym_pybullet_drones.envs.RLFunctions import Problem 

RLLIB = False

if __name__ == "__main__":

    #### Check the environment's spaces ################################################################
    env = gym.make("the-aviary-v1", problem=Problem.SA_TAKEOFF)
    print("[INFO] Action space:", env.action_space)
    print("[INFO] Observation space:", env.observation_space)
    check_env(env, warn=True, skip_render_check=True) 

    #### Train the model ###############################################################################
    if not RLLIB:
        model = A2C(MlpPolicy, env, verbose=1)
        model.learn(total_timesteps=500000)
    else:
        ray.shutdown(); ray.init(ignore_reinit_error=True)
        register_env("sa-aviary", lambda _: Aviary(problem=Problem.SA_TAKEOFF))
        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 2
        config["env"] = "sa-aviary"
        agent = ppo.PPOTrainer(config)
        for i in range(100):
            results = agent.train()
            print("[INFO] {:d}: episode_reward max {:f} min {:f} mean {:f}".format(i, \
                    results["episode_reward_max"], results["episode_reward_min"], results["episode_reward_mean"]))
        policy = agent.get_policy()
        print(policy.model.base_model.summary())
        ray.shutdown()

    #### Show (and record a video of) the model's performance ##########################################
    env = Aviary(gui=True, record=True, problem=Problem.SA_TAKEOFF)
    obs = env.reset()
    start = time.time()
    for i in range(10*env.SIM_FREQ):
        if not RLLIB: action, _states = model.predict(obs, deterministic=True)
        else: action, _states, _dict = policy.compute_single_action(obs)
        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        if done: obs = env.reset()
    env.close()
