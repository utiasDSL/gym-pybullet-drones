import argparse
import os
from datetime import datetime
import gym
import numpy as np
from gym.spaces import Box, Dict
import torch
import torch.nn as nn
import ray
from ray import tune
from ray.tune import register_env
from ray.rllib.agents import ppo, sac
from ray.rllib.agents.callbacks import DefaultCallbacks, MultiCallbacks

from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary
from gym_pybullet_drones.envs.multi_agent_rl.NavigationAviary import NavigationAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType

AGGR_PHY_STEPS = 5
ENV_CLS = {
    "flock": FlockAviary,
    "leaderfollower": LeaderFollowerAviary,
    "meetup": MeetupAviary,
    "navigation": NavigationAviary,
}

ALGOTIRHM_CLASS = {
    "ppo": ppo,
    "sac": sac,
}

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--num_drones', default=2, type=int, help='Number of drones (default: 2)', metavar='')
    parser.add_argument('--env', default='leaderfollower',  type=str, choices=list(ENV_CLS.keys()), help='Task (default: leaderfollower)', metavar='')
    parser.add_argument('--obs', default='kin',             type=ObservationType, help='Observation space (default: kin)', metavar='')
    parser.add_argument('--act', default='vel',       type=ActionType, help='Action space (default: one_d_rpm)', metavar='')
    parser.add_argument('--algo', default='ppo',            type=str, choices=list(ALGOTIRHM_CLASS.keys()), help='MARL approach (default: cc)', metavar='')
    parser.add_argument('--workers', default=0,             type=int, help='Number of RLlib workers (default: 0)', metavar='')        
    ARGS = parser.parse_args()

    #### Save directory ########################################
    filename = os.path.dirname(os.path.abspath(__file__))+'/results/save-'+ARGS.env+'-'+str(ARGS.num_drones)+'-'+ARGS.algo+'-'+ARGS.obs.value+'-'+ARGS.act.value+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    if not os.path.exists(filename):
        os.makedirs(filename+'/')

    #### Initialize Ray Tune ###################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

    #### Register the environment ##############################
    temp_env_name = "this-aviary-v0"
    env_cls = ENV_CLS[ARGS.env]
    env_config = dict(
        num_drones=ARGS.num_drones,
        aggregate_phy_steps=AGGR_PHY_STEPS,
        obs=ARGS.obs,
        act=ARGS.act
    )

    eval_env_config = env_config.copy()
    eval_env_config["record"] = True

    register_env(temp_env_name, lambda config: env_cls(**config))
    temp_env: gym.Env = env_cls(**env_config)
    observation_space = temp_env.observation_space[0]
    action_space = temp_env.action_space[0]

    #### Set up the trainer's config ###########################
    config: dict = ALGOTIRHM_CLASS[ARGS.algo].DEFAULT_CONFIG.copy()

    config.update({
        "env": temp_env_name,
        "env_config": env_config,
        "num_workers": 0 + ARGS.workers,
        "num_envs_per_worker": 1,
        "num_gpus": int(os.environ.get("RLLIB_NUM_GPUS", "0")), # Use GPUs iff `RLLIB_NUM_GPUS` env var set to > 0
        "batch_mode": "complete_episodes",
        "framework": "torch",
    })
    if ARGS.algo == "ppo":
        config.update({
            "lambda": 0.95,
            "kl_coeff": 0.5, 
            "train_batch_size": tune.grid_search([65536, 4096]) ,
            "num_sgd_iter": tune.grid_search([5, 15]),
            "sgd_minibatch_size": tune.grid_search([256, 4096]),
            "entropy_coeff": tune.grid_search([0, 0.01]),
            "clip_param": 0.2,
            "grad_clip": 0.5,
            "framework": "torch",})
    elif ARGS.algo == "sac":
        config.update({
            "train_batch_size": 512,
            "target_network_update_freq": 1,
            "timesteps_per_iteration": 1000,
        })
    else: raise ValueError
    
    config.update({
        "evaluation_interval": 20,
        "evaluation_config": {
            "env_config": eval_env_config,
            "explore": False
        },
        "multiagent": { 
            "policies": {
                "shared_policy": (None, temp_env.observation_space[0], action_space, {}),
            },
            "policy_mapping_fn": lambda agent_id, episode, worker, **kwargs: "shared_policy",
        }
    })

    #### Ray Tune stopping conditions ##########################
    stop = {
        "timesteps_total": 10000000,
    }

    #### Train #################################################
    results = tune.run(
        ARGS.algo.upper(),
        stop=stop,
        config=config,
        verbose=True,
        checkpoint_at_end=True,
        checkpoint_freq=50,
        local_dir=filename,
    )

    #### Save agent ############################################
    checkpoints = results.get_trial_checkpoints_paths(
        trial=results.get_best_trial('episode_reward_mean', mode='max'),
        metric='episode_reward_mean'
    )
    with open(filename+'/checkpoint.txt', 'w+') as f:
        f.write(checkpoints[0][0])

    #### Shut down Ray #########################################
    ray.shutdown()
