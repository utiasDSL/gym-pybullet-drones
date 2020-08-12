from gym.envs.registration import register

register(
	id='single-drone-v0',
	entry_point='gym_pybullet_drones.envs:SingleDroneEnv',
)

register(
	id='multi-drone-v0',
	entry_point='gym_pybullet_drones.envs:MultiDroneEnv',
)
