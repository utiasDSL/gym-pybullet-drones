import time
import argparse
import gym

from stable_baselines3.common.env_checker import check_env

from stable_baselines3 import A2C
from stable_baselines3.a2c import MlpPolicy as a2cMlpPolicy
from stable_baselines3.a2c import CnnPolicy as a2cCnnPolicy
#
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy as ppoMlpPolicy
#from ray.rllib.agents import ppo
#import ray
#from ray.tune import register_env
#
from stable_baselines3 import SAC
from stable_baselines3.sac import MlpPolicy as sacMlpPolicy
#
from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy as td3MlpPolicy
#
from stable_baselines3 import DDPG

from gym_pybullet_drones.utils.utils import *

from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##########################################
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using TakeoffAviary')
    parser.add_argument('--algo',      default='a2c',        type=str,       choices=['a2c', 'ppo', 'sac', 'td3', 'ddpg'],     help='Help (default: ..)', metavar='')
    parser.add_argument('--env',       default='takeoff',        type=str,       choices=['takeoff', 'hover', 'flythrugate'],     help='Help (default: ..)', metavar='')
    
    # add common parameters
    # learning_rate, gamma, add more ?
    # add policy type
    # mlp vs cnn policy

    ARGS = parser.parse_args()

    #### Check the environment's spaces ################################################################
    #
    #env_name = "takeoff"+"-aviary-v0"
    #env_name = "hover"+"-aviary-v0"
    env_name = ARGS.env+"-aviary-v0"
    #
    train_env = gym.make(env_name)
    print("[INFO] Action space:", train_env.action_space)
    print("[INFO] Observation space:", train_env.observation_space)
    check_env(train_env, warn=True, skip_render_check=True)

    #### Train the model ###############################################################################
    if ARGS.algo == 'a2c': model = A2C(a2cMlpPolicy, train_env, verbose=1)
    model.learn(total_timesteps=500000/1000)

    # OR
    # ray.shutdown(); ray.init(ignore_reinit_error=True)
    # register_env("takeoff-aviary-v0", lambda _: TakeoffAviary())
    # config = ppo.DEFAULT_CONFIG.copy()
    # config["num_workers"] = 2
    # config["env"] = "takeoff-aviary-v0"
    # agent = ppo.PPOTrainer(config)
    # for i in range(100):
    #     results = agent.train()
    #     print("[INFO] {:d}: episode_reward max {:f} min {:f} mean {:f}".format(i, \
    #             results["episode_reward_max"], results["episode_reward_min"], results["episode_reward_mean"]))
    # policy = agent.get_policy()
    # print(policy.model.base_model.summary())
    # ray.shutdown()

    #### Show (and record a video of) the model's performance ##########################################
    #
    test_env = train_env = gym.make(env_name, gui=True, record=False) #TakeoffAviary(gui=True, record=True)
    #
    obs = test_env.reset()
    start = time.time()
    for i in range(10*test_env.SIM_FREQ):
        action, _states = model.predict(obs, deterministic=True)

        # OR
        # action, _states, _dict = policy.compute_single_action(obs)
        obs, reward, done, info = test_env.step(action)
        test_env.render()
        sync(i, start, test_env.TIMESTEP)
        print()
        print(done)
        print()
        if done: obs = test_env.reset()
    test_env.close()








####################################################################################################

# https://stable-baselines3.readthedocs.io/en/master/modules/a2c.html

# import gym

# from stable_baselines3 import A2C
# from stable_baselines3.a2c import MlpPolicy
# from stable_baselines3.common.env_util import make_vec_env

# # Parallel environments
# env = make_vec_env('CartPole-v1', n_envs=4)

# model = A2C(MlpPolicy, env, verbose=1)
# model.learn(total_timesteps=25000)
# model.save("a2c_cartpole")

# del model # remove to demonstrate saving and loading

# model = A2C.load("a2c_cartpole")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

####################################################################################################

# https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

# import gym

# from stable_baselines3 import PPO
# from stable_baselines3.ppo import MlpPolicy
# from stable_baselines3.common.env_util import make_vec_env

# # Parallel environments
# env = make_vec_env('CartPole-v1', n_envs=4)

# model = PPO(MlpPolicy, env, verbose=1)
# model.learn(total_timesteps=25000)
# model.save("ppo_cartpole")

# del model # remove to demonstrate saving and loading

# model = PPO.load("ppo_cartpole")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

####################################################################################################

# https://stable-baselines3.readthedocs.io/en/master/modules/sac.html

# import gym
# import numpy as np

# from stable_baselines3 import SAC
# from stable_baselines3.sac import MlpPolicy

# env = gym.make('Pendulum-v0')

# model = SAC(MlpPolicy, env, verbose=1)
# model.learn(total_timesteps=10000, log_interval=4)
# model.save("sac_pendulum")

# del model # remove to demonstrate saving and loading

# model = SAC.load("sac_pendulum")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render()
#     if done:
#       obs = env.reset()

####################################################################################################

# https://stable-baselines3.readthedocs.io/en/master/modules/td3.html

# import gym
# import numpy as np

# from stable_baselines3 import TD3
# from stable_baselines3.td3.policies import MlpPolicy
# from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

# env = gym.make('Pendulum-v0')

# # The noise objects for TD3
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

# model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1)
# model.learn(total_timesteps=10000, log_interval=10)
# model.save("td3_pendulum")
# env = model.get_env()

# del model # remove to demonstrate saving and loading

# model = TD3.load("td3_pendulum")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

####################################################################################################

# https://stable-baselines3.readthedocs.io/en/master/modules/ddpg.html

# import gym
# import numpy as np

# from stable_baselines3 import DDPG
# from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

# env = gym.make('Pendulum-v0')

# # The noise objects for DDPG
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

# model = DDPG('MlpPolicy', env, action_noise=action_noise, verbose=1)
# model.learn(total_timesteps=10000, log_interval=10)
# model.save("ddpg_pendulum")
# env = model.get_env()

# del model # remove to demonstrate saving and loading

# model = DDPG.load("ddpg_pendulum")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

####################################################################################################


