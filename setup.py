from setuptools import setup

setup(name='gym_pybullet_drones',
    version='0.6.0',
    install_requires=['numpy', 'Pillow', 'matplotlib', 'cycler', 'gym', 'pybullet', 'stable_baselines3', 'ray[rllib]']
)
