"""Learning script for single agent problems.

Agents are based on `stable_baselines3`'s implementation of A2C, PPO SAC, TD3, DDPG.

Example
-------
To run the script, type in a terminal:

    $ python singleagent.py --env <env> --algo <alg> --obs <ObservationType> --act <ActionType> --cpu <cpu_num>

Notes
-----
Use:
    $ tensorboard --logdir ./results/save-<env>-<algo>-<obs>-<act>-<time-date>/tb/
to see the tensorboard results at:
    http://localhost:6006/

"""
import os
import time
from datetime import datetime
import argparse
import subprocess
import numpy as np
import gym
import torch
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.cmd_util import make_vec_env # Module cmd_util will be renamed to env_util https://github.com/DLR-RM/stable-baselines3/pull/197
from stable_baselines3.common.vec_env import SubprocVecEnv, VecTransposeImage
from stable_baselines3.common.utils import set_random_seed
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

from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary
from gym_pybullet_drones.envs.single_agent_rl.TuneAviary import TuneAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType

import shared_constants

EPISODE_REWARD_THRESHOLD = -0 # Upperbound: rewards are always negative, but non-zero
"""float: Reward threshold to halt the script."""

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning experiments script')
    parser.add_argument('--env',        default='hover',      type=str,             choices=['takeoff', 'hover', 'flythrugate', 'tune'], help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='ppo',        type=str,             choices=['a2c', 'ppo', 'sac', 'td3', 'ddpg'],        help='Help (default: ..)', metavar='')
    parser.add_argument('--obs',        default='kin',        type=ObservationType,                                                      help='Help (default: ..)', metavar='')
    parser.add_argument('--act',        default='one_d_rpm',  type=ActionType,                                                           help='Help (default: ..)', metavar='')
    parser.add_argument('--cpu',        default='1',          type=int,                                                                  help='Help (default: ..)', metavar='')        
    ARGS = parser.parse_args()

    #### Save directory ########################################
    filename = os.path.dirname(os.path.abspath(__file__))+'/results/save-'+ARGS.env+'-'+ARGS.algo+'-'+ARGS.obs.value+'-'+ARGS.act.value+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    if not os.path.exists(filename):
        os.makedirs(filename+'/')

    #### Print out current git commit hash #####################
    git_commit = subprocess.check_output(["git", "describe", "--tags"]).strip()
    with open(filename+'/git_commit.txt', 'w+') as f:
        f.write(str(git_commit))

    #### Warning ###############################################
    if ARGS.env == 'tune' and ARGS.act != ActionType.TUN:
        print("\n\n\n[WARNING] TuneAviary is intended for use with ActionType.TUN\n\n\n")
    if ARGS.act == ActionType.ONE_D_RPM or ARGS.act == ActionType.ONE_D_DYN or ARGS.act == ActionType.ONE_D_PID:
        print("\n\n\n[WARNING] Simplified 1D problem for debugging purposes\n\n\n")
    #### Errors ################################################
        if not ARGS.env in ['takeoff', 'hover']: 
            print("[ERROR] 1D action space is only compatible with Takeoff and HoverAviary")
            exit()
    if ARGS.act == ActionType.TUN and ARGS.env != 'tune' :
        print("[ERROR] ActionType.TUN is only compatible with TuneAviary")
        exit()
    if ARGS.algo in ['sac', 'td3', 'ddpg'] and ARGS.cpu!=1: 
        print("[ERROR] The selected algorithm does not support multiple environments")
        exit()

    #### Uncomment to debug slurm scripts ######################
    # exit()

    env_name = ARGS.env+"-aviary-v0"
    sa_env_kwargs = dict(aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act)
    # train_env = gym.make(env_name, aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act) # single environment instead of a vectorized one    
    if env_name == "takeoff-aviary-v0":
        train_env = make_vec_env(TakeoffAviary,
                                 env_kwargs=sa_env_kwargs,
                                 n_envs=ARGS.cpu,
                                 seed=0
                                 )
    if env_name == "hover-aviary-v0":
        train_env = make_vec_env(HoverAviary,
                                 env_kwargs=sa_env_kwargs,
                                 n_envs=ARGS.cpu,
                                 seed=0
                                 )
    if env_name == "flythrugate-aviary-v0":
        train_env = make_vec_env(FlyThruGateAviary,
                                 env_kwargs=sa_env_kwargs,
                                 n_envs=ARGS.cpu,
                                 seed=0
                                 )
    if env_name == "tune-aviary-v0":
        train_env = make_vec_env(TuneAviary,
                                 env_kwargs=sa_env_kwargs,
                                 n_envs=ARGS.cpu,
                                 seed=0
                                 )
    print("[INFO] Action space:", train_env.action_space)
    print("[INFO] Observation space:", train_env.observation_space)
    # check_env(train_env, warn=True, skip_render_check=True)
    
    #### On-policy algorithms ##################################
    onpolicy_kwargs = dict(activation_fn=torch.nn.ReLU,
                           net_arch=[512, 512, dict(vf=[256, 128], pi=[256, 128])]
                           ) # or None
    if ARGS.algo == 'a2c':
        model = A2C(a2cppoMlpPolicy,
                    train_env,
                    policy_kwargs=onpolicy_kwargs,
                    tensorboard_log=filename+'/tb/',
                    verbose=1
                    ) if ARGS.obs == ObservationType.KIN else A2C(a2cppoCnnPolicy,
                                                                  train_env,
                                                                  policy_kwargs=onpolicy_kwargs,
                                                                  tensorboard_log=filename+'/tb/',
                                                                  verbose=1
                                                                  )
    if ARGS.algo == 'ppo':
        model = PPO(a2cppoMlpPolicy,
                    train_env,
                    policy_kwargs=onpolicy_kwargs,
                    tensorboard_log=filename+'/tb/',
                    verbose=1
                    ) if ARGS.obs == ObservationType.KIN else PPO(a2cppoCnnPolicy,
                                                                  train_env,
                                                                  policy_kwargs=onpolicy_kwargs,
                                                                  tensorboard_log=filename+'/tb/',
                                                                  verbose=1
                                                                  )

    #### Off-policy algorithms #################################
    offpolicy_kwargs = dict(activation_fn=torch.nn.ReLU,
                            net_arch=[512, 512, 256, 128]
                            ) # or None # or dict(net_arch=dict(qf=[256, 128, 64, 32], pi=[256, 128, 64, 32]))
    if ARGS.algo == 'sac':
        model = SAC(sacMlpPolicy,
                    train_env,
                    policy_kwargs=offpolicy_kwargs,
                    tensorboard_log=filename+'/tb/',
                    verbose=1
                    ) if ARGS.obs==ObservationType.KIN else SAC(sacCnnPolicy,
                                                                train_env,
                                                                policy_kwargs=offpolicy_kwargs,
                                                                tensorboard_log=filename+'/tb/',
                                                                verbose=1
                                                                )
    if ARGS.algo == 'td3':
        model = TD3(td3ddpgMlpPolicy,
                    train_env,
                    policy_kwargs=offpolicy_kwargs,
                    tensorboard_log=filename+'/tb/',
                    verbose=1
                    ) if ARGS.obs==ObservationType.KIN else TD3(td3ddpgCnnPolicy,
                                                                train_env,
                                                                policy_kwargs=offpolicy_kwargs,
                                                                tensorboard_log=filename+'/tb/',
                                                                verbose=1
                                                                )
    if ARGS.algo == 'ddpg':
        model = DDPG(td3ddpgMlpPolicy,
                    train_env,
                    policy_kwargs=offpolicy_kwargs,
                    tensorboard_log=filename+'/tb/',
                    verbose=1
                    ) if ARGS.obs==ObservationType.KIN else DDPG(td3ddpgCnnPolicy,
                                                                train_env,
                                                                policy_kwargs=offpolicy_kwargs,
                                                                tensorboard_log=filename+'/tb/',
                                                                verbose=1
                                                                )

    #### Create eveluation environment #########################
    if ARGS.obs == ObservationType.KIN: 
        eval_env = gym.make(env_name,
                            aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS,
                            obs=ARGS.obs,
                            act=ARGS.act
                            )
    elif ARGS.obs == ObservationType.RGB:
        if env_name == "takeoff-aviary-v0": 
            eval_env = make_vec_env(TakeoffAviary,
                                    env_kwargs=sa_env_kwargs,
                                    n_envs=1,
                                    seed=0
                                    )
        if env_nam == "hover-aviary-v0": 
            eval_env = make_vec_env(HoverAviary,
                                    env_kwargs=sa_env_kwargs,
                                    n_envs=1,
                                    seed=0
                                    )
        if env_name == "flythrugate-aviary-v0": 
            eval_env = make_vec_env(FlyThruGateAviary,
                                    env_kwargs=sa_env_kwargs,
                                    n_envs=1,
                                    seed=0
                                    )
        if env_name == "tune-aviary-v0": 
            eval_env = make_vec_env(TuneAviary,
                                    env_kwargs=sa_env_kwargs,
                                    n_envs=1,
                                    seed=0
                                    )
        eval_env = VecTransposeImage(eval_env)

    #### Train the model #######################################
    # checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=filename+'-logs/', name_prefix='rl_model')
    callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=EPISODE_REWARD_THRESHOLD,
                                                     verbose=1
                                                     )
    eval_callback = EvalCallback(eval_env,
                                 callback_on_new_best=callback_on_best,
                                 verbose=1,
                                 best_model_save_path=filename+'/',
                                 log_path=filename+'/',
                                 eval_freq=int(2000/ARGS.cpu),
                                 deterministic=True,
                                 render=False
                                 )
    model.learn(total_timesteps=35000, #int(1e12),
                callback=eval_callback,
                log_interval=100,
                )

    #### Save the model ########################################
    model.save(filename+'/success_model.zip')
    print(filename)

    #### Print training progression ############################
    with np.load(filename+'/evaluations.npz') as data:
        for j in range(data['timesteps'].shape[0]):
            print(str(data['timesteps'][j])+","+str(data['results'][j][0][0]))
