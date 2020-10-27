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
    ARGS = parser.parse_args()
    filename = os.path.dirname(os.path.abspath(__file__))+'/save-'+ARGS.env+'-'+ARGS.algo+'-'+ARGS.pol+'-'+ARGS.input+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")

    #### Check the environment's spaces ################################################################
    env_name = ARGS.env+"-aviary-v0"
    IMG_OBS = True if ARGS.pol=='cnn' else False
    DYN_IN = True if ARGS.input=='dyn' else False
    train_env = gym.make(env_name, img_obs=IMG_OBS, dyn_input=DYN_IN) # TO DO: vector env
    print("[INFO] Action space:", train_env.action_space)
    print("[INFO] Observation space:", train_env.observation_space)
    check_env(train_env, warn=True, skip_render_check=True)
    
    #### On-policy algorithms ##########################################################################
    onpolicy_kwargs = dict(activation_fn=torch.nn.ReLU, net_arch=[256, 128, dict(vf=[64, 32], pi=[64, 32])]) # or None
    if ARGS.algo=='a2c': 
        model = A2C(a2cppoMlpPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else A2C(a2cppoCnnPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='ppo': 
        model = PPO(a2cppoMlpPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else PPO(a2cppoCnnPolicy, train_env, policy_kwargs=onpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)

    #### Off-policy algorithms ##########################################################################
    offpolicy_kwargs = dict(activation_fn=torch.nn.ReLU, net_arch=[256, 128, 64, 32]) # or None # or dict(net_arch=dict(qf=[256, 128, 64, 32], pi=[256, 128, 64, 32]))
    if ARGS.algo=='sac': 
        model = SAC(sacMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else SAC(sacCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='td3': 
        model = TD3(td3ddpgMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else TD3(td3ddpgCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)
    if ARGS.algo=='ddpg': 
        model = DDPG(td3ddpgMlpPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1) if ARGS.pol=='mlp' else DDPG(td3ddpgCnnPolicy, train_env, policy_kwargs=offpolicy_kwargs, tensorboard_log=filename+'-tb/', verbose=1)


    EPISODE_REWARD_THRESHOLD = 100 # TBD


    #### Train the model ###############################################################################
    eval_env = gym.make(env_name, img_obs=IMG_OBS, dyn_input=DYN_IN)
    # checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=filename+'-logs/', name_prefix='rl_model')
    callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=EPISODE_REWARD_THRESHOLD, verbose=1)
    # eval_callback = EvalCallback(eval_env, callback_on_new_best=callback_on_best, verbose=1)
    eval_callback = EvalCallback(eval_env, callback_on_new_best=callback_on_best, verbose=1, best_model_save_path=filename+'-logs/', log_path=filename+'-logs/', eval_freq=500, deterministic=True, render=False)
    model.learn(total_timesteps=int(1e10), callback=eval_callback, log_interval=100)

    ### Save the model #################################################################################
    model.save(filename)
    print(filename)

    # use $ tensorboard --logdir /save-<env>-<algo>-<pol>-<time-date>-tb for the tensorboard results at http://localhost:6006/

