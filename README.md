# gym-pybullet-drones
A simple [OpenAI Gym environment](https://gym.openai.com/envs/#classic_control) based on [PyBullet](https://github.com/bulletphysics/bullet3) to simulate one (or more TBD) quadrotors 

Everything after a `$` is entered on a terminal, everything after `>>>` is passed to a Python interpreter

<img src="images/wp.gif" alt="alt text" width="400"> <img src="images/wp.png" alt="alt text" width="400">




### Requirements
This project was written in Python 3.7.6 on macOS 10.15.5

Major dependencies are `gym` (`$ pip install gym`),  `pybullet`  (`$ pip install pybullet`), 
`stable-baselines3` (`$ pip install stable-baselines3`), and `ffmpeg` 
(only used for video recording, on macOS, install with `$ brew install ffmpeg`, on Linux `$ sudo apt install ffmpeg`)

Using a `conda` environment ([see these instructions](https://github.com/JacopoPan/a-minimalist-guide#install-conda)), 
dependencies (excluding `ffmpeg`), can be installed from file `conda-req-list.txt`
```
$ conda create -n myenv --file /gym-pybullet-drones/gym_pybullet_drones/assets/conda-req-list.txt
```




## Installation
The repo is structured as a [Gym Environment](https://github.com/openai/gym/blob/master/docs/creating-environments.md)
and can be installed with `pip`
```
$ git clone https://github.com/JacopoPan/gym-pybullet-drones.git
$ cd gym-pybullet-drones/
$ pip install -e .
```




## Use
There are 4 main scripts in the repo

`run_physics_test.py` is meant to test PyBullet's forces and torques in `p.WORLD_FRAME` and `p.LINK_FRAME`
```
$ conda activate myenv							# If using a conda environment
$ cd gym-pybullet-drones/
$ python run_physics_test.py 					# use run_physics_test.py for a simple script that only depends on pybullet
```
`run_trace_test.py` runs a comparison with a previos trace saved in `/gym-pybullet-drones/gym_pybullet_drones/assets/trace_1.pkl` **using PID control** implemented in `SingleDroneEnv.control()`
```
$ conda activate myenv										# If using a conda environment
$ cd gym-pybullet-drones/
$ python run_trace_test.py
```
<img src="images/trace_comparison.gif" alt="alt text" width="400"> <img src="images/trace_comparison.png" alt="alt text" width="400">


`run_flight_test.py` runs an independent flight **using PID control** implemented in `SingleDroneEnv.control()`
```
$ conda activate myenv										# If using a conda environment
$ cd gym-pybullet-drones/
$ python run_flight_test.py
```
<img src="images/crash.gif" alt="alt text" width="400"> <img src="images/crash.png" alt="alt text" width="400">


`run_learning_test.py` is a minimal RL example using [A2C](https://stable-baselines3.readthedocs.io/en/master/modules/a2c.html) from `stable-baselines3` to learn how to take off the ground
```
$ conda activate myenv										# If using a conda environment
$ cd gym-pybullet-drones/
$ python run_learning_test.py
```
<img src="images/learn1.gif" alt="alt text" width="400"> <img src="images/learn3.gif" alt="alt text" width="400">
<img src="images/learn2.gif" alt="alt text" width="400"> <img src="images/learn4.gif" alt="alt text" width="400">




## SingleDroneEnv
A single quadrotor enviroment can be created with
```
>>> env = SingleDroneEnv(drone_model=DroneModel.CF2X, \		# See gym-pybullet-drones/gym_pybullet_drones/envs/DroneModel.py for other quadcopter models (remove this comment)
							pybullet=True, \				# Whether to use PyBullet physics or the dynamics in method _noPyBulletDynamics() of gym-pybullet-drones/gym_pybullet_drones/env/SingleDroneEnv.py (remove this comment)
							normalized_spaces=True, \		# Whether to use normalized action and observation spaces—set to True for learning (default), False for flight simulation (remove this comment)
							freq=240, \						# The stepping frequency of the simulation (remove this comment)
							gui=True, \						# Whether to display PyBullet's GUI (remove this comment)
							obstacles=False, \				# Whether to add obstacles to the environment (remove this comment)
							record=False)					# Whether to save a .mp4 video in gym-pybullet-drones/ (remove this comment)
															# See run_flight_test.py for an example
````
Or (having installed the environment with `pip`) using
```
>>> env = gym.make('single-drone-v0')						# See run_learning_test.py for an example
```
Then, the environment can be stepped with
```
>>> obs = env.reset()
>>> for _ in range(10*240):
>>>		obs, reward, done, info = env.step(env.action_space.sample())
>>>		env.render()
>>>		if done: obs = env.reset()
>>> env.close()
```




### Action Space
The action space is a [`Box(4,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py) containing the desired rotation speed of each motor

If `normalized_spaces=True`, ranging from -1 to 1, if `normalized_spaces=False` ranging from 0 to `SingleDroneEnv.MAX_RPM`

### Observation Space
The observation space is a [`Box(20,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py) containing the drone's position, quaternion, roll/pitch/yaw, velocity, angular velocity, and the motors' speeds

Check method `_clipAndNormalizeState()` in `gym-pybullet-drones/gym_pybullet_drones/envs/SingleDroneEnv.py` to understand the difference between `normalized_spaces=False` (using the raw simulation information) and `normalized_spaces=True` (using [-1,1] ranges) 

### Reward
The reward function can be customized in method `_computeReward()` of `gym-pybullet-drones/gym_pybullet_drones/envs/SingleDroneEnv.py`, for example
```
def _computeReward(self, state):
	if state[2] > 0.5: return 1000
	elif state[2] > 0.1: return 100
	else: return -1
```

### Done
The halting conditions can be customized in method `_isDone()` of `gym-pybullet-drones/gym_pybullet_drones/envs/SingleDroneEnv.py`, for example
```
def _isDone(self, state):
	if np.abs(state[0])>.5 or np.abs(state[1])>.5 or np.abs(state[2])>=1 or np.abs(state[7])>np.pi/2 or np.abs(state[8])>np.pi/2 \
				or self.step_counter > 5*self.SIM_FREQ: 
		return True
	else: 
		return False
```




## MultiDroneEnv
TBD
### Actions
TBD
### Observation Space
aaa
### Reward
aaa
### Done
aaa




## References
- Carlos Luis and Jeroome Le Ny (2016) [*Design of a Trajectory Tracking Controller for a Nanoquadcopter*](https://arxiv.org/pdf/1608.05786.pdf)
- Julian Forster (2015) [*System Identification of the Crazyflie 2.0 Nano Quadrocopter*](http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf)
- Benoit Landry (2014) [*Planning and Control for Quadrotor Flight through Cluttered Environments*](http://groups.csail.mit.edu/robotics-center/public_papers/Landry15)
- Nathan Michael, Daniel Mellinger, Quentin Lindsey, and Vijay Kumar (2010) [*The GRASP Multiple Micro UAV Testbed*](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf)

