from gym.envs.registration import register

register(
    id='the-aviary-v1',
    entry_point='gym_pybullet_drones.envs:Aviary',
)
