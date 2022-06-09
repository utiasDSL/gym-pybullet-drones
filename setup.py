import setuptools
from gym_pybullet_drones import *

# read the contents of your README file
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setuptools.setup(name='gym_pybullet_drones',
    version='1.0.0',
    install_requires=[
        'numpy',
        'Pillow',
        'matplotlib',
        'cycler',
        'gym',
        'pybullet',
        'stable_baselines3',
        'ray[rllib]'
        ],
    packages=setuptools.find_packages(where="gym_pybullet_drones"),
    package_dir={'':'gym_pybullet_drones'},
    package_data={'gym_pybullet_drones': ['assets/*.*']},
    long_description=long_description,
    long_description_content_type='test/markdown',
    license='MIT License\nCopyright (c) 2020 Jacopo Panerati',
    entry_points = {
        'console_scripts': ['downwash=examples.downwash:main'],
    }
)