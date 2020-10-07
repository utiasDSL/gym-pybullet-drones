from setuptools import setup

setup(name='gym_pybullet_drones',
    version='0.3.5',
    install_requires=['gym', 'pybullet', 'stable_baselines3', 'ray[rllib]']
)
