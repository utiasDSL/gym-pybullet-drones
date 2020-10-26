import os
import time
from datetime import datetime
import argparse
import re
import numpy as np
import gym
import torch
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import A2C
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import TD3
from stable_baselines3 import DDPG
from stable_baselines3.common.policies import ActorCriticPolicy as a2cppoMlpPolicy
from stable_baselines3.common.policies import ActorCriticCnnPolicy as a2cppoCnnPolicy
from stable_baselines3.sac.policies import SACPolicy as sacMlpPolicy
from stable_baselines3.sac import CnnPolicy as sacCnnPolicy
from stable_baselines3.td3 import MlpPolicy as td3ddpgMlpPolicy
from stable_baselines3.td3 import CnnPolicy as td3ddpgCnnPolicy
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.utils.utils import *
from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using TakeoffAviary')
    parser.add_argument('--file',      type=str,       help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()

    #### Load the model from file ######################################################################
    algo = ARGS.file.split("-")[2]
    if algo=='a2c': model = A2C.load(ARGS.file)
    if algo=='ppo': model = PPO.load(ARGS.file)
    if algo=='sac': model = SAC.load(ARGS.file)
    if algo=='td3': model = TD3.load(ARGS.file)
    if algo=='ddpg': model = DDPG.load(ARGS.file)

    #### Recreate the environment ######################################################################
    env_name = ARGS.file.split("-")[1]+"-aviary-v0"
    IMG_OBS = True if ARGS.file.split("-")[3]=='cnn' else False
    DYN_IN = True if ARGS.file.split("-")[4]=='dyn' else False
    test_env = gym.make(env_name, gui=True, record=False, img_obs=IMG_OBS, dyn_input=DYN_IN)

    #### Evaluate the model ############################################################################
    eval_env = gym.make(env_name, img_obs=IMG_OBS, dyn_input=DYN_IN)
    mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)
    print("\n\n\nMean reward ", mean_reward, " +- ", std_reward, "\n\n")

    #### Show (and record a video of) the model's performance ##########################################
    obs = test_env.reset()
    start = time.time()
    for i in range(10*int(test_env.SIM_FREQ/test_env.AGGR_PHY_STEPS)):
        action, _states = model.predict(obs, deterministic=True) # deterministic=False
        obs, reward, done, info = test_env.step(action)
        test_env.render()
        sync(np.floor(i*test_env.AGGR_PHY_STEPS), start, test_env.TIMESTEP)
        # if done: obs = test_env.reset() # OPTIONAL
    test_env.close()

    # use $ tensorboard --logdir /save-<env>-<algo>-<pol>-<input>-<time-date>-tb for the tensorboard results at http://localhost:6006/








