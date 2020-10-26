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
from gym.spaces import Box, Dict
import torch
import torch.nn as nn
from ray.rllib.models.torch.fcnet import FullyConnectedNetwork
import ray
from ray import tune
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
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

class FillInActions(DefaultCallbacks):
    def on_postprocess_trajectory(self, worker, episode, agent_id, policy_id, policies, postprocessed_batch, original_batches, **kwargs):
        action_vec_size = 4
        to_update = postprocessed_batch[SampleBatch.CUR_OBS]
        other_id = 1 if agent_id == 0 else 0
        action_encoder = ModelCatalog.get_preprocessor_for_space( 
                                                        Box(-np.inf, np.inf, (action_vec_size,), np.float32) # WHY? UNBOUNDED? # Discrete(2)
                                                        )
        _, opponent_batch = original_batches[other_id]
        opponent_actions = np.array([ action_encoder.transform(a) for a in opponent_batch[SampleBatch.ACTIONS] ])
        to_update[:, -action_vec_size:] = opponent_actions

def central_critic_observer(agent_obs, **kw):
    action_vec_size = 4
    new_obs = {
        0: {
            "own_obs": agent_obs[0],
            "opponent_obs": agent_obs[1],
            "opponent_action": np.zeros(action_vec_size),  # filled in by FillInActions
        },
        1: {
            "own_obs": agent_obs[1],
            "opponent_obs": agent_obs[0],
            "opponent_action": np.zeros(action_vec_size),  # filled in by FillInActions
        },
    }
    return new_obs

class YetAnotherTorchCentralizedCriticModel(TorchModelV2, nn.Module):
    """Multi-agent model that implements a centralized value function.

    It assumes the observation is a dict with 'own_obs' and 'opponent_obs', the
    former of which can be used for computing actions (i.e., decentralized
    execution), and the latter for optimization (i.e., centralized learning).

    This model has two parts:
    - An action model that looks at just 'own_obs' to compute actions
    - A value model that also looks at the 'opponent_obs' / 'opponent_action'
      to compute the value (it does this by using the 'obs_flat' tensor).
    """

    def __init__(self, obs_space, action_space, num_outputs, model_config, name):
        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        nn.Module.__init__(self)
        self.action_model = FullyConnectedNetwork(
            #Box(low=0, high=1, shape=(6, )),  # one-hot encoded Discrete(6)
            Box(low=-1, high=1, shape=(20, )),  # one-hot encoded Discrete(6)
            Box(low=-1, high=1, shape=(4, )), # action_space,
            num_outputs,
            model_config,
            name + "_action"
            )
        self.value_model = FullyConnectedNetwork(
            obs_space, 
            Box(low=-1, high=1, shape=(4, )), # action_space,
            1, 
            model_config, 
            name + "_vf"
            )
        self._model_in = None

    def forward(self, input_dict, state, seq_lens):
        # Store model-input for possible `value_function()` call.
        self._model_in = [input_dict["obs_flat"], state, seq_lens]
        return self.action_model({ "obs": input_dict["obs"]["own_obs"] }, state, seq_lens)

    def value_function(self):
        value_out, _ = self.value_model({ "obs": self._model_in[0] }, self._model_in[1], self._model_in[2])
        return torch.reshape(value_out, [-1])




if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--num_drones', default=2,         type=int,                                                           help='Number of drones (default: 2)', metavar='')
    parser.add_argument('--env',        default='flock',   type=str,       choices=['leaderfollower', 'flock', 'meetup'],      help='Help (default: ..)', metavar='')
    parser.add_argument('--algo',       default='cc',      type=str,       choices=['cc', 'other'],                            help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()
    filename = os.path.dirname(os.path.abspath(__file__))+'/save-'+ARGS.env+'-'+ARGS.algo+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")

    #### Initialize Ray Tune ###########################################################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

    if ARGS.algo=='cc':

        ModelCatalog.register_custom_model(
            "cc_model", YetAnotherTorchCentralizedCriticModel #if args.torch else YetAnotherCentralizedCriticModel
        )

        #### Unused env to extract correctly sized action and observation spaces ###########################
        unused_env = FlockAviary(num_drones=ARGS.num_drones)
        # print(unused_env.action_space["0"]) # Box(-1.0, 1.0, (4,), float32)
        # print(unused_env.observation_space["0"]) # Dict(neighbors:MultiBinary(2), state:Box(-1.0, 1.0, (20,), float32))
        action_space = unused_env.action_space[0]
        observer_space = Dict({
            "own_obs": unused_env.observation_space[0],
            "opponent_obs": unused_env.observation_space[0],
            "opponent_action": unused_env.action_space[0],
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
            "model": {
               "custom_model": "cc_model",
            },
            "framework": "torch", # if args.torch else "tf",
        }
        
        config["multiagent"] = { 
            # Map of type MultiAgentPolicyConfigDict from policy ids to tuples of (policy_cls, obs_space, act_space, config).
            # This defines the observation and action spaces of the policies and any extra config.
            "policies": {
                # "pol0": (PPOTFPolicy, unused_env.observation_space["0"], unused_env.action_space["0"], {"framework": "torch"}),
                "pol0": (None, observer_space, action_space, {"agent_id": 0,}),
                "pol1": (None, observer_space, action_space, {"agent_id": 1,}),
            },
            # Function mapping agent ids to policy ids.
            #"policy_mapping_fn": lambda agent_id: "pol"+str(agent_id),
            "policy_mapping_fn": lambda x: "pol0" if x == 0 else "pol1",
            # An additional observation function, see rllib/evaluation/observation_function.py for more info.
            "observation_fn": central_critic_observer, # "observation_fn": None,
            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic_2.py
            # more principled but complex way to share observations is using `postprocess_trajectory`
            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic.py
        }

        #### Ray Tune stopping conditions ##################################################################
        stop = {
            "timesteps_total": 10, #8000
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

    elif ARGS.algo=='other':
        print("Not yet implemented")
        pass



    # check_learning_achieved(results, 1.0)

    #### Save agent #################################################################################
    checkpoints = results.get_trial_checkpoints_paths(trial=results.get_best_trial('episode_reward_mean',mode='max'), metric='episode_reward_mean')
    checkpoint_path = checkpoints[0][0]
    print(checkpoint_path)

    ray.shutdown()






