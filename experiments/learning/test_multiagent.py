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


from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics

from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary


from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

from multiagent import *

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Multi-agent reinforcement learning experiments script')
    parser.add_argument('--num_drones',     default=2,      type=int,       help='Number of drones (default: 2)', metavar='')
    parser.add_argument('--file',                           type=str,       help='Help (default: ..)', metavar='')
    ARGS = parser.parse_args()

    # /Users/jacopo/ray_results/PPO/PPO_this-flock-aviary-v0_136de_00000_0_2020-10-25_20-20-39/checkpoint_1/checkpoint-1


    #### Initialize Ray Tune ###########################################################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

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

    #### Restore agent #################################################################################
    agent = ppo.PPOTrainer(config=config)
    agent.restore(ARGS.file)

    #### Extract and print policies ####################################################################
    policy0 = agent.get_policy("pol0")
    policy1 = agent.get_policy("pol1")
    #print(policy0.model.base_model.summary())
    #print(policy1.model.base_model.summary())

    #### Create test environment ########################################################################
    env = FlockAviary(num_drones=ARGS.num_drones, gui=True, record=False, obstacles=True)
    obs = env.reset()
    # action = { str(i): np.array([0,0,0,0]) for i in range(ARGS.num_drones) }
    action = { i: np.array([0,0,0,0]) for i in range(ARGS.num_drones) }
    start = time.time()
    for i in range(10*env.SIM_FREQ):

        #### Deploy the policies ###########################################################################
        print("Debug Obs", obs)
        temp = {}
        # temp["0"] = policy0.compute_single_action(np.hstack([ obs["0"]["state"], obs["0"]["neighbors"] ]))
        # temp["1"] = policy1.compute_single_action(np.hstack([ obs["1"]["state"], obs["1"]["neighbors"] ]))
        # print("Debug Act", temp)
        # action = {"0": temp["0"][0], "1": temp["1"][0], "2": temp["2"][0]}
        temp[0] = policy0.compute_single_action(np.hstack([ obs[0], obs[1], action[1] ]))
        temp[1] = policy1.compute_single_action(np.hstack([ obs[1], obs[0], action[0] ]))
        print("Debug Act", temp)
        action = {0: temp[0][0], 1: temp[1][0]}

        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        # if done["__all__"]: obs = env.reset()
    env.close()

    #### Shut down Ray #################################################################################
    ray.shutdown()










