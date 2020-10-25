import os
import time
from datetime import datetime
import argparse
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

from gym_pybullet_drones.utils.utils import *

from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using TakeoffAviary')
    parser.add_argument('--env',        default='takeoff',    type=str,       choices=['takeoff', 'hover', 'flythrugate'],      help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='a2c',        type=str,       choices=['a2c', 'ppo', 'sac', 'td3', 'ddpg'],     help='Help (default: ..)', metavar='')
    parser.add_argument('--pol',        default='mlp',        type=str,       choices=['mlp', 'cnn'],                           help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()
    filename = os.path.dirname(os.path.abspath(__file__))+'/save-'+ARGS.env+'-'+ARGS.algo+'-'+ARGS.pol+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")

    #### Check the environment's spaces ################################################################
    env_name = ARGS.env+"-aviary-v0"
    train_env = gym.make(env_name, img_obs=False) if ARGS.pol=='mlp' else gym.make(env_name, img_obs=True) # TO DO: vector env
    print("[INFO] Action space:", train_env.action_space)
    print("[INFO] Observation space:", train_env.observation_space)
    check_env(train_env, warn=True, skip_render_check=True)
    
    #### On-policy algorithms ##########################################################################
    onpolicy_kwargs = None # e.g. dict(activation_fn=torch.nn.ReLU, net_arch=[128, dict(vf=[256, 256])])
    if ARGS.algo=='a2c': 
        model = A2C(a2cppoMlpPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else A2C(a2cppoCnnPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='ppo': 
        model = PPO(a2cppoMlpPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else PPO(a2cppoCnnPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)

    #### Off-policy algorithms ##########################################################################
    offpolicy_kwargs = None # e.g. dict(net_arch=[256, 256]) # or dict(net_arch=dict(pi=[64, 64], qf=[400, 300]))
    if ARGS.algo=='sac': 
        model = SAC(sacMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else SAC(sacCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='td3': 
        model = TD3(td3ddpgMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else TD3(td3ddpgCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='ddpg': 
        model = DDPG(td3ddpgMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else DDPG(td3ddpgCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    
    #### Train the model ###############################################################################
    model.learn(total_timesteps=1000*100, log_interval=10)

    ### Save
    model.save(filename)
    print(filename)



