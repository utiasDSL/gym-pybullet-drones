import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from stable_baselines3 import PPO
from stable_baselines3 import A2C
from stable_baselines3.a2c import MlpPolicy
from stable_baselines3.common.env_checker import check_env

from utils import *
from gym_pybullet_drones.envs.DroneModel import DroneModel
from gym_pybullet_drones.envs.SingleDroneEnv import SingleDroneEnv
from gym_pybullet_drones.envs.MultiDroneEnv import MultiDroneEnv

if __name__ == "__main__":

	####################################################################################################
	#### Create the environment ########################################################################
	####################################################################################################
	env = gym.make('single-drone-v0')
	print("Action space:", env.action_space)
	print("Observation space:", env.observation_space)
	check_env(env, warn=True, skip_render_check=True) 

	####################################################################################################
	#### Train the model ###############################################################################
	####################################################################################################
	model = A2C(MlpPolicy, env, verbose=1)
	model.learn(total_timesteps=500000)
	
	####################################################################################################
	#### Show (and record a video of) the model's performance ##########################################
	####################################################################################################
	env = SingleDroneEnv(gui=True, record=True)
	obs = env.reset()
	PYB_CLIENT = env.getPyBulletClient()
	DRONE_ID = env.getDroneId()
	start = time.time()
	for i in range(10*env.SIM_FREQ):
		action, _states = model.predict(obs, deterministic=True)
		obs, reward, done, info = env.step(action)
		env.render()
		sync_sim(i, start, env.TIMESTEP)
		if done: obs = env.reset()
	env.close()
