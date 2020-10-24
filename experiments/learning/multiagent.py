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
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.DynCtrlAviary import DynCtrlAviary
from gym_pybullet_drones.envs.VisionCtrlAviary import VisionCtrlAviary

from gym_pybullet_drones.envs.multi_agent_rl.FlockAviary import FlockAviary
from gym_pybullet_drones.envs.multi_agent_rl.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.envs.multi_agent_rl.MeetupAviary import MeetupAviary


from gym_pybullet_drones.envs.multi_agent_rl.NormDynCtrlAviary import NormDynCtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import *

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Ongoing development script')
    parser.add_argument('--drone',              default="cf2x",     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=3,          type=int,           help='Number of drones (default: 5)', metavar='')
    parser.add_argument('--physics',            default="pyb",      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=False,      type=str2bool,      help='Whether to use VisionCtrlAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=True,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=48,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=15,         type=int,           help='Duration of the simulation in seconds (default: 15)', metavar='')
    parser.add_argument('--debug_marl',         default=False,      type=str2bool,      help='Whether to print obs, reward, done of FlockAviary (default: False)', metavar='')
    parser.add_argument('--dyn_ctrl',           default=False,      type=str2bool,      help='Whether to use DynCtrlAviary (default: False)', metavar='')
    parser.add_argument('--log',                default=True,       type=str2bool,      help='Whether to log the simulation (default: True)', metavar='')
    parser.add_argument('--aggregate',          default=True,       type=str2bool,      help='Whether to aggregate physics steps (default: True)', metavar='')
    
    parser.add_argument('--algo',      default="temp",        type=str,       help='Help (default: ..)', metavar='')
    parser.add_argument('--env',       default="temp",        type=str,       help='Help (default: ..)', metavar='')

    ARGS = parser.parse_args()

    #### Initialize Ray Tune ###########################################################################
    ray.shutdown()
    ray.init(ignore_reinit_error=True)

    #### Register the environment ######################################################################
    register_env("flock-aviary-v0", lambda _: FlockAviary(drone_model=ARGS.drone, num_drones=ARGS.num_drones, physics=ARGS.physics, freq=ARGS.simulation_freq_hz))

    #### for the default config, see github.com/ray-project/ray/blob/master/rllib/agents/trainer.py

    #### Set up the trainer's config ###################################################################
    config = ppo.DEFAULT_CONFIG.copy()
    config["num_workers"] = 0
    config["env"] = "flock-aviary-v0"
    #### Unused env to extract correctly sized action and observation spaces ###########################
    unused_env = FlockAviary(num_drones=ARGS.num_drones)
    config["multiagent"] = { # Map of type MultiAgentPolicyConfigDict from policy ids to tuples of (policy_cls, obs_space, act_space, config).
                            # This defines the observation and action spaces of the policies and any extra config.
                            "policies": {
                                "pol0": (PPOTFPolicy, unused_env.observation_space["0"], unused_env.action_space["0"], {"framework": "torch"}),
                                "pol1": (PPOTFPolicy, unused_env.observation_space["1"], unused_env.action_space["1"], {"framework": "torch"}),
                                "pol2": (PPOTFPolicy, unused_env.observation_space["2"], unused_env.action_space["2"], {"framework": "torch"}),
                            },
                            # Function mapping agent ids to policy ids.
                            "policy_mapping_fn": lambda agent_id: "pol"+str(agent_id),
                            # An additional observation function, see rllib/evaluation/observation_function.py for more info.
                            # "observation_fn": None,
                            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic_2.py
                            # more principled but complex way to share observations is using `postprocess_trajectory`
                            # see github.com/ray-project/ray/blob/master/rllib/examples/centralized_critic.py
                            }

    #### Ray Tune stopping conditions ##################################################################
    stop = {
        "timesteps_total": 8000,
    }

    #### Train #########################################################################################
    results = tune.run(
        "PPO",
        stop=stop,
        config=config,
        verbose=True,
        checkpoint_at_end=True)
    # check_learning_achieved(results, 1.0)

    #### Restore agent #################################################################################
    checkpoints = results.get_trial_checkpoints_paths(trial=results.get_best_trial('episode_reward_mean',mode='max'), metric='episode_reward_mean')
    checkpoint_path = checkpoints[0][0]; agent = ppo.PPOTrainer(config=config); agent.restore(checkpoint_path)

    #### Extract and print policies ####################################################################
    policy0 = agent.get_policy("pol0")
    policy1 = agent.get_policy("pol1")
    policy2 = agent.get_policy("pol2")
    print(policy0.model.base_model.summary())
    print(policy1.model.base_model.summary())
    print(policy2.model.base_model.summary())

    #### Create test environment ########################################################################
    env = FlockAviary(drone_model=ARGS.drone, num_drones=ARGS.num_drones, physics=ARGS.physics, freq=ARGS.simulation_freq_hz, gui=True, record=False, obstacles=True)
    obs = env.reset()
    action = { str(i): np.array([0,0,0,0]) for i in range(ARGS.num_drones) }
    start = time.time()
    for i in range(10*env.SIM_FREQ):

        #### Deploy the policies ###########################################################################
        print("Debug Obs", obs)
        temp = {}
        temp["0"] = policy0.compute_single_action(np.hstack([ obs["0"]["state"], obs["0"]["neighbors"] ]))
        temp["1"] = policy1.compute_single_action(np.hstack([ obs["1"]["state"], obs["1"]["neighbors"] ]))
        temp["2"] = policy2.compute_single_action(np.hstack([ obs["2"]["state"], obs["2"]["neighbors"] ]))
        print("Debug Act", temp)
        action = {"0": temp["0"][0], "1": temp["1"][0], "2": temp["2"][0]}

        obs, reward, done, info = env.step(action)
        env.render()
        sync(i, start, env.TIMESTEP)
        if done["__all__"]: obs = env.reset()
    env.close()

    #### Shut down Ray #################################################################################
    ray.shutdown()










