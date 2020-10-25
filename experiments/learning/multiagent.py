import os
import time
import argparse
from datetime import datetime
import pdb
import math
import numpy as np
import pybullet as p
import pickle
import matplotlib.pyplot as plt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import ray
from ray import tune
from ray.tune import register_env
from ray.rllib.agents import ppo
from ray.rllib.agents.ppo import PPOTrainer, PPOTFPolicy
from ray.rllib.examples.policy.random_policy import RandomPolicy
from ray.rllib.utils.test_utils import check_learning_achieved



from ray.rllib.agents.callbacks import DefaultCallbacks
from ray.rllib.examples.models.centralized_critic_models import YetAnotherCentralizedCriticModel, YetAnotherTorchCentralizedCriticModel
from ray.rllib.models import ModelCatalog
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.env.multi_agent_env import ENV_STATE



from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics

from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary


from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *





class FillInActions(DefaultCallbacks):
    """Fills in the opponent actions info in the training batches."""

    def on_postprocess_trajectory(self, worker, episode, agent_id, policy_id,
                                  policies, postprocessed_batch,
                                  original_batches, **kwargs):
        to_update = postprocessed_batch[SampleBatch.CUR_OBS]
        other_id = 1 if agent_id == 0 else 0
        action_encoder = ModelCatalog.get_preprocessor_for_space(Discrete(2))

        # set the opponent actions into the observation
        _, opponent_batch = original_batches[other_id]
        opponent_actions = np.array([
            action_encoder.transform(a)
            for a in opponent_batch[SampleBatch.ACTIONS]
        ])
        to_update[:, -2:] = opponent_actions

def central_critic_observer(agent_obs, **kw):
    """Rewrites the agent obs to include opponent data for training."""

    new_obs = {
        0: {
            "own_obs": agent_obs[0],
            "opponent_obs": agent_obs[1],
            "opponent_action": 0,  # filled in by FillInActions
        },
        1: {
            "own_obs": agent_obs[1],
            "opponent_obs": agent_obs[0],
            "opponent_action": 0,  # filled in by FillInActions
        },
    }
    return new_obs






if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--num_drones', default=2,         type=int,                                                           help='Number of drones (default: 2)', metavar='')
    parser.add_argument('--env',        default='flock',   type=str,       choices=['leaderfollower', 'flock', 'meetup'],      help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='cc',      type=str,       choices=['cc', 'maddp'],                            help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()
    filename = os.path.dirname(os.path.abspath(__file__))+'/save-'+ARGS.env+'-'+ARGS.algo+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")

    # implement but not necessariliy suitable for continuous actions QMIX,  MADDPG
    # https://docs.ray.io/en/latest/rllib-algorithms.html#multi-agent-methods
    # example: https://github.com/ray-project/ray/blob/master/rllib/examples/two_step_game.py
    
    # use ad-hoc centralized critics instead
    # with RLlibs' A2C, PPO, TD3 / DDPG, SAC
    # example: https://github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic_2.py

    # common parameters?

    #### Initialize Ray Tune ###########################################################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

    if ARGS.algo=='cc':

        ModelCatalog.register_custom_model(
            "cc_model", YetAnotherTorchCentralizedCriticModel
            if args.torch else YetAnotherCentralizedCriticModel
        )

        #### Unused env to extract correctly sized action and observation spaces ###########################
        unused_env = FlockAviary(num_drones=ARGS.num_drones)

        action_space = Discrete(2)
        observer_space = Dict({
            "own_obs": Discrete(6),
            # These two fields are filled in by the CentralCriticObserver, and are
            # not used for inference, only for training.
            "opponent_obs": Discrete(6),
            "opponent_action": Discrete(2),
        })

        #### Register the environment ######################################################################
        register_env("this-flock-aviary-v0", lambda _: FlockAviary(num_drones=ARGS.num_drones))

        #### Set up the trainer's config ###################################################################
        config = ppo.DEFAULT_CONFIG.copy() # for the default config, see github.com/ray-project/ray/blob/master/rllib/agents/trainer.py
        
        config = {
            "env": "this-flock-aviary-v0",
            "batch_mode": "complete_episodes",
            "callbacks": FillInActions,
            # Use GPUs iff `RLLIB_NUM_GPUS` env var set to > 0.
            # "num_gpus": int(os.environ.get("RLLIB_NUM_GPUS", "0")),
            "num_workers": 0,
            #"model": {
            #    "custom_model": "cc_model",
            #},
            "framework": "torch" if args.torch else "tf",
        }
        
        config["multiagent"] = { 
            # Map of type MultiAgentPolicyConfigDict from policy ids to tuples of (policy_cls, obs_space, act_space, config).
            # This defines the observation and action spaces of the policies and any extra config.
            "policies": {
                # "pol0": (PPOTFPolicy, unused_env.observation_space["0"], unused_env.action_space["0"], {"framework": "torch"}),
                # "pol1": (PPOTFPolicy, unused_env.observation_space["1"], unused_env.action_space["1"], {"framework": "torch"}),
                # "pol2": (PPOTFPolicy, unused_env.observation_space["2"], unused_env.action_space["2"], {"framework": "torch"}),
                "pol0": (None, unused_env.observation_space["0"], unused_env.action_space["0"], {}),
                "pol1": (None, unused_env.observation_space["1"], unused_env.action_space["1"], {}),
                # "pol2": (None, unused_env.observation_space["2"], unused_env.action_space["2"], {}),
            },
            # Function mapping agent ids to policy ids.
            "policy_mapping_fn": lambda agent_id: "pol"+str(agent_id),
            # An additional observation function, see rllib/evaluation/observation_function.py for more info.
            # "observation_fn": None,
            "observation_fn": central_critic_observer,
            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic_2.py
            # more principled but complex way to share observations is using `postprocess_trajectory`
            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic.py
        }




    elif ARGS.algo=='maddpg':
        obs_space_dict = {
            "agent_1": Discrete(6),
            "agent_2": Discrete(6),
        }
        act_space_dict = {
            "agent_1": TwoStepGame.action_space,
            "agent_2": TwoStepGame.action_space,
        }
        config = {
            "learning_starts": 100,
            "env_config": {
                "actions_are_logits": True,
            },
            "multiagent": {
                "policies": {
                    "pol1": (None, Discrete(6), TwoStepGame.action_space, {
                        "agent_id": 0,
                    }),
                    "pol2": (None, Discrete(6), TwoStepGame.action_space, {
                        "agent_id": 1,
                    }),
                },
                "policy_mapping_fn": lambda x: "pol1" if x == 0 else "pol2",
            },
            "framework": "torch" if args.torch else "tf",
            # Use GPUs iff `RLLIB_NUM_GPUS` env var set to > 0.
            # "num_gpus": int(os.environ.get("RLLIB_NUM_GPUS", "0")),
        }
        group = False




    #### Ray Tune stopping conditions ##################################################################
    stop = {
        "timesteps_total": 8000,
        # "timesteps_total": 0,
        # "episode_reward_mean": 0,
    }

    #### Train #########################################################################################
    results = tune.run(
        "PPO",
        stop=stop,
        config=config,
        verbose=True,
        checkpoint_at_end=True
    )
    # check_learning_achieved(results, 1.0)

    #### Save agent #################################################################################
    checkpoints = results.get_trial_checkpoints_paths(trial=results.get_best_trial('episode_reward_mean',mode='max'), metric='episode_reward_mean')
    checkpoint_path = checkpoints[0][0]
    print(checkpoint_path)






