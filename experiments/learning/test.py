import os
import time
from datetime import datetime
import argparse
import subprocess
import gym
import torch
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.cmd_util import make_vec_env # Module cmd_util will be renamed to env_util https://github.com/DLR-RM/stable-baselines3/pull/197
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
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, StopTrainingOnRewardThreshold

from gym_pybullet_drones.utils.utils import *
from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary

if __name__ == "__main__":

    # Use as $ python singleagent.py --env <env> --algo <alg> --pol <mlp/cnn> --input <rpm/dyn>

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning experiments script')
    parser.add_argument('--env',        default='takeoff',    type=str,       choices=['takeoff', 'hover', 'flythrugate'],      help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='a2c',        type=str,       choices=['a2c', 'ppo', 'sac', 'td3', 'ddpg'],     help='Help (default: ..)', metavar='')
    parser.add_argument('--pol',        default='mlp',        type=str,       choices=['mlp', 'cnn'],                           help='Help (default: ..)', metavar='')
    parser.add_argument('--input',      default='rpm',        type=str,       choices=['rpm', 'dyn'],                           help='Help (default: ..)', metavar='')    
    parser.add_argument('--cpu',        default='1',          type=int,                                                         help='Help (default: ..)', metavar='')    
    ARGS = parser.parse_args()

    #### Save directory ################################################################################
    filename = os.path.dirname(os.path.abspath(__file__))+'/save-'+ARGS.env+'-'+ARGS.algo+'-'+ARGS.pol+'-'+ARGS.input+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    if not os.path.exists(filename): os.makedirs(filename+'/')

    #### Print out current git commit hash #############################################################
    git_commit = subprocess.check_output(["git", "describe", "--tags"]).strip(); print(git_commit)
    with open(filename+'/git_commit.txt', 'w+') as f: 
        f.write(str(git_commit))
        f.write(str(ARG.env))
        f.write(str(ARG.algo))
        f.write(str(ARG.pol))
        f.write(str(ARG.input))
        f.write(str(ARG.cpu))


