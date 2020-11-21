from setuptools import setup

setup(name='gym_pybullet_drones',
    version='0.5.2',
    install_requires=['numpy', 'matplotlib', 'Pillow', 'cycler', 'gym', 'pybullet', 'stable_baselines3', 'ray[rllib]']
)
