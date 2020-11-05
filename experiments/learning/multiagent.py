import os
import time
import argparse
from datetime import datetime
import subprocess
import pdb
import math
import numpy as np
import pybullet as p
import pickle
import matplotlib.pyplot as plt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym.spaces import Box, Dict
import torch
import torch.nn as nn
from ray.rllib.models.torch.fcnet import FullyConnectedNetwork
import ray
from ray import tune
from ray.tune.logger import DEFAULT_LOGGERS
from ray.tune import register_env
from ray.rllib.agents import ppo
from ray.rllib.agents.ppo import PPOTrainer, PPOTFPolicy
from ray.rllib.examples.policy.random_policy import RandomPolicy
from ray.rllib.utils.test_utils import check_learning_achieved
from ray.rllib.agents.callbacks import DefaultCallbacks
from ray.rllib.models.torch.torch_modelv2 import TorchModelV2
from ray.rllib.models import ModelCatalog
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.env.multi_agent_env import ENV_STATE

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

import common_constants
from ma_common import CustomTorchCentralizedCriticModel, FillInActions, central_critic_observer

######################################################################################################################################################
if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--num_drones', default=2,            type=int,                                                                 help='Number of drones (default: 2)', metavar='')
    parser.add_argument('--env',        default='flock',      type=str,             choices=['leaderfollower', 'flock', 'meetup'],      help='Help (default: ..)', metavar='')
    parser.add_argument('--obs',        default='kin',        type=ObservationType,                                                     help='Help (default: ..)', metavar='')
    parser.add_argument('--act',        default='one_d_rpm',  type=ActionType,                                                          help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='cc',         type=str,             choices=['cc'],                                     help='Help (default: ..)', metavar='')
    parser.add_argument('--workers',    default=0,            type=int,                                                                 help='Help (default: ..)', metavar='')        
    ARGS = parser.parse_args()

    #### Save directory ################################################################################
    filename = os.path.dirname(os.path.abspath(__file__))+'/results/save-'+ARGS.env+'-'+str(ARGS.num_drones)+'-'+ARGS.algo+'-'+ARGS.obs.value+'-'+ARGS.act.value+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    if not os.path.exists(filename): os.makedirs(filename+'/')

    #### Print out current git commit hash #############################################################
    git_commit = subprocess.check_output(["git", "describe", "--tags"]).strip(); print(git_commit)
    with open(filename+'/git_commit.txt', 'w+') as f: f.write(str(git_commit))

    #### Constants, and errors #########################################################################
    if ARGS.obs==ObservationType.KIN:  o = 12
    elif ARGS.obs==ObservationType.RGB: print("[ERROR] ObservationType.RGB for multi-agent systems not yet implemented"); exit()
    else: print("[ERROR] unknown ObservationType"); exit()
    if ARGS.act in [ActionType.ONE_D_RPM, ActionType.ONE_D_DYN, ActionType.ONE_D_PID]: a = 1
    elif ARGS.act in [ActionType.RPM, ActionType.DYN]: a = 4
    elif ARGS.act==ActionType.PID: a = 3
    else: print("[ERROR] unknown ActionType"); exit()
    with open( os.path.dirname(os.path.abspath(__file__))+'/results/obs.txt', 'w+') as f: f.write(str(o))
    with open( os.path.dirname(os.path.abspath(__file__))+'/results/act.txt', 'w+') as f: f.write(str(a))

    #### Uncomment to debug slurm scripts ##############################################################
    # exit()

    #### Initialize Ray Tune ###########################################################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

    #### Register the custom centralized critic model ##################################################
    ModelCatalog.register_custom_model("cc_model", CustomTorchCentralizedCriticModel)

    #### Register the environment ######################################################################
    temp_env_name = "this-aviary-v0"
    if ARGS.env=='flock': register_env(temp_env_name, lambda _: FlockAviary(num_drones=ARGS.num_drones, aggregate_phy_steps=common_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act))
    else: print("[ERROR] not yet implemented"); exit()

    #### Unused env to extract correctly sized action and observation spaces ###########################
    if ARGS.env=='flock': temp_env = FlockAviary(num_drones=ARGS.num_drones, aggregate_phy_steps=common_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act)
    else: print("[ERROR] not yet implemented"); exit()
    observer_space = Dict({
        "own_obs": temp_env.observation_space[0],
        "opponent_obs": temp_env.observation_space[0],
        "opponent_action": temp_env.action_space[0],
    })
    action_space = temp_env.action_space[0]

    #### Set up the trainer's config ###################################################################
    config = ppo.DEFAULT_CONFIG.copy() # For the default config, see github.com/ray-project/ray/blob/master/rllib/agents/trainer.py
    config = {
        "env": temp_env_name,
        "num_workers": 0+ARGS.workers,
        "num_gpus": int(os.environ.get("RLLIB_NUM_GPUS", "0")), # Use GPUs iff `RLLIB_NUM_GPUS` env var set to > 0
        "batch_mode": "complete_episodes",
        "callbacks": FillInActions,
        "framework": "torch",
    }

    #### Set up the model parameters of the trainer's config ###########################################
    config["model"] = { 
        "custom_model": "cc_model",
    }
    
    #### Set up the multiagent parameters of the trainer's config ######################################
    config["multiagent"] = { 
        "policies": {
            "pol0": (None, observer_space, action_space, {"agent_id": 0,}),
            "pol1": (None, observer_space, action_space, {"agent_id": 1,}),
        },
        "policy_mapping_fn": lambda x: "pol0" if x == 0 else "pol1", # # Function mapping agent ids to policy ids
        "observation_fn": central_critic_observer, # See rllib/evaluation/observation_function.py for more info
    }

    #### Ray Tune stopping conditions ##################################################################
    stop = {
        "timesteps_total": 100, # 8000,
        # "episode_reward_mean": 0,
        # "training_iteration": 0,
    }

    #### Train #########################################################################################
    results = tune.run(
        "PPO",
        stop=stop,
        config=config,
        verbose=True,
        checkpoint_at_end=True,
        local_dir=filename,
    )
    # check_learning_achieved(results, 1.0)

    #### Save agent #################################################################################
    checkpoints = results.get_trial_checkpoints_paths(trial=results.get_best_trial('episode_reward_mean',mode='max'), metric='episode_reward_mean')
    with open(filename+'/checkpoint.txt', 'w+') as f: f.write(checkpoints[0][0])

    #### Shut down Ray #################################################################################
    ray.shutdown()






