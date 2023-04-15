"""Script demonstrating the use of `gym_pybullet_drones`' Gym interface.

Class TakeoffAviary is used as a learning env for the A2C and PPO algorithms.

Example
-------
In a terminal, run as:

    $ python learn.py

Notes
-----
The boolean argument --rllib switches between `stable-baselines3` and `ray[rllib]`.
This is a minimal working example integrating `gym-pybullet-drones` with 
reinforcement learning libraries `stable-baselines3` and `ray[rllib]`.
It is not meant as a good/effective learning example.

"""
import time
import argparse
import gym
import numpy as np
from stable_baselines3 import A2C
from stable_baselines3.a2c import MlpPolicy
from stable_baselines3.common.env_checker import check_env
import ray
from ray.tune import register_env
from ray.rllib.agents import ppo

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_RLLIB = False
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(rllib=DEFAULT_RLLIB,output_folder=DEFAULT_OUTPUT_FOLDER, gui=DEFAULT_GUI, plot=True, colab=DEFAULT_COLAB, record_video=DEFAULT_RECORD_VIDEO):

    #### Check the environment's spaces ########################
    env = gym.make("takeoff-aviary-v0")
    print("[INFO] Action space:", env.action_space)
    print("[INFO] Observation space:", env.observation_space)
    check_env(env,
              warn=True,
              skip_render_check=True
              )

    #### Train the model #######################################
    if not rllib:
        model = A2C(MlpPolicy,
                    env,
                    verbose=1
                    )
        model.learn(total_timesteps=10000) # Typically not enough
    else:
        ray.shutdown()
        ray.init(ignore_reinit_error=True)
        register_env("takeoff-aviary-v0", lambda _: TakeoffAviary())
        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 2
        config["framework"] = "torch"
        config["env"] = "takeoff-aviary-v0"
        agent = ppo.PPOTrainer(config)
        for i in range(3): # Typically not enough
            results = agent.train()
            print("[INFO] {:d}: episode_reward max {:f} min {:f} mean {:f}".format(i,
                                                                                   results["episode_reward_max"],
                                                                                   results["episode_reward_min"],
                                                                                   results["episode_reward_mean"]
                                                                                   )
                  )
        policy = agent.get_policy()
        ray.shutdown()

    #### Show (and record a video of) the model's performance ##
    env = TakeoffAviary(gui=gui,
                        record=record_video
                        )
    logger = Logger(logging_freq_hz=int(env.SIM_FREQ/env.AGGR_PHY_STEPS),
                    num_drones=1,
                    output_folder=output_folder,
                    colab=colab
                    )
    obs = env.reset()
    start = time.time()
    for i in range(3*env.SIM_FREQ):
        if not rllib:
            action, _states = model.predict(obs,
                                            deterministic=True
                                            )
        else:
            action, _states, _dict = policy.compute_single_action(obs)
        obs, reward, done, info = env.step(action)
        logger.log(drone=0,
                   timestamp=i/env.SIM_FREQ,
                   state=np.hstack([obs[0:3], np.zeros(4), obs[3:15],  np.resize(action, (4))]),
                   control=np.zeros(12)
                   )
        if i%env.SIM_FREQ == 0:
            env.render()
            print(done)
        sync(i, start, env.TIMESTEP)
        if done:
            obs = env.reset()
    env.close()

    if plot:
        logger.plot()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using TakeoffAviary')
    parser.add_argument('--rllib',      default=DEFAULT_RLLIB,        type=str2bool,       help='Whether to use RLlib PPO in place of stable-baselines A2C (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
