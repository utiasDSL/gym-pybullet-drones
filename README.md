# gym-pybullet-drones
Simple [OpenAI Gym environment](https://gym.openai.com/envs/#classic_control) based on [PyBullet](https://github.com/bulletphysics/bullet3) to simulate one or more quadrotors 

<img src="files/readme_images/helix.gif" alt="alt text" width="360"> <img src="files/readme_images/helix.png" alt="alt text" width="450">

- The default `DroneModel.CF2X` dynamics are based on [Bitcraze's Crazyflie 2.x nano-quadrotor](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf)

- Everything after a `$` is entered on a terminal, everything after `>>>` is passed to a Python interpreter

- Suggestions and corrections are very welcome in the form of [issues](https://github.com/JacopoPan/gym-pybullet-drones/issues) and [pull requests](https://github.com/JacopoPan/gym-pybullet-drones/pulls), respectively


### Requirements
The repo was written using Python 3.7.6 on macOS 10.15: major dependencies are [`gym`](https://gym.openai.com/docs/),  [`pybullet`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#), 
[`stable-baselines3`](https://stable-baselines3.readthedocs.io/en/master/guide/quickstart.html), [`rllib`](https://docs.ray.io/en/master/rllib.html) and [`ffmpeg`](https://ffmpeg.org) (only used for video recording)
```
$ pip install gym
$ pip install pybullet
$ pip install stable-baselines3
$ pip install `ray[rllib]`
$ brew install ffmpeg                       # on macOS
$ sudo apt install ffmpeg                   # on Linux
```
Using a [`conda` environment](https://github.com/JacopoPan/a-minimalist-guide#install-conda), 
dependencies (except `ffmpeg`), can be installed from file
```
$ cd gym-pybullet-drones/
$ conda create -n myenv --file /files/conda_req_list.txt
```




## Installation
The repo is structured as a [Gym Environment](https://github.com/openai/gym/blob/master/docs/creating-environments.md)
and can be installed with `pip install --editable`
```
$ git clone https://github.com/JacopoPan/gym-pybullet-drones.git
$ cd gym-pybullet-drones/
$ pip install -e .
```




## Use
There are 4 main scripts in `examples/`: `compare.py`, `fly.py`, `learn.py`, and `physics.py`

- `compare.py` replays and compare to a trace saved in [`example_trace.pkl`](https://github.com/JacopoPan/gym-pybullet-drones/tree/master/files/example_trace.pkl)
```
$ conda activate myenv                      # If using a conda environment
$ cd gym-pybullet-drones/examples/
$ python compare.py
```
<img src="files/readme_images/trace_comparison.gif" alt="alt text" width="360"> <img src="files/readme_images/trace_comparison.png" alt="alt text" width="450">

- `fly.py` runs an independent flight **using PID control** implemented in class [`Control`](https://github.com/JacopoPan/gym-pybullet-drones/tree/master/gym_pybullet_drones/envs/Control.py)
```
$ conda activate myenv                      # If using a conda environment
$ cd gym-pybullet-drones/examples/
$ python fly.py
```
> Tip: use the GUI's sliders and button `Use GUI RPM` to override the control with interactive inputs

<img src="files/readme_images/wp.gif" alt="alt text" width="360"> <img src="files/readme_images/wp.png" alt="alt text" width="450">

<img src="files/readme_images/crash.gif" alt="alt text" width="360"> <img src="files/readme_images/crash.png" alt="alt text" width="450">

> `downwash.py` is a flight script with only 2 drones, to test the downwash model

<img src="files/readme_images/downwash.gif" alt="alt text" width="360"> <img src="files/readme_images/downwash.png" alt="alt text" width="450">

- `learn.py` is an **RL example** to learn take-off using `stable-baselines3`'s [A2C](https://stable-baselines3.readthedocs.io/en/master/modules/a2c.html) or `rllib`'s [PPO](https://docs.ray.io/en/master/rllib-algorithms.html#ppo)
```
$ conda activate myenv                      # If using a conda environment
$ cd gym-pybullet-drones/examples/
$ python learn.py
```
<img src="files/readme_images/learn1.gif" alt="alt text" width="400"> <img src="files/readme_images/learn2.gif" alt="alt text" width="400">
<img src="files/readme_images/learn3.gif" alt="alt text" width="400"> <img src="files/readme_images/learn4.gif" alt="alt text" width="400">

- `physics.py` is an accessory script that can be used to understand PyBullet's force and torque APIs for different [URDF links](http://wiki.ros.org/urdf/XML/link) in `p.WORLD_FRAME` and `p.LINK_FRAME`
```
$ conda activate myenv                      # If using a conda environment
$ cd gym-pybullet-drones/examples/
$ python physics.py
```
> Tip: also check the examples in [pybullet-examples](https://github.com/JacopoPan/pybullet-examples)


## Aviary
A `gym.Env` flight arena for one (ore more) quadrotor can be created with `Aviary()`—see [`fly.py`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/examples/fly.py) for an example
```
>>> env = Aviary( \
>>>       drone_model=DroneModel.CF2X, \    # See DroneModel Enum class for other quadcopter models (remove this comment)
>>>       num_drones=1, \                   # Number of drones (remove this comment)
>>>       visibility_radius=np.inf, \       # Distance at which drones are considered neighbors, only used for multiple drones (remove this comment)
>>>       initial_xyzs=None, \              # Initial XYZ positions of the drones (remove this comment)
>>>       initial_rpys=None, \              # Initial roll, pitch, and yaw of the drones in radians (remove this comment)
>>>       physics: Physics=Physics.PYB, \   # Choice of (PyBullet) physics implementation (remove this comment)
>>>       normalized_spaces=True, \         # Whether to use normalized action and observation spaces—use True for learning (default), False for simulation (remove this comment)
>>>       freq=240, \                       # Stepping frequency of the simulation (remove this comment)
>>>       gui=True, \                       # Whether to display PyBullet's GUI (remove this comment)
>>>       obstacles=False, \                # Whether to add obstacles to the environment (remove this comment)
>>>       record=False, \                   # Whether to save a .mp4 video in gym-pybullet-drones/files/ (remove this comment)
>>>       problem: Problem=Problem.DEFAULT) # Choice of reward and done functions in class ProblemSpecificFunctions (remove this comment)
````
Or using `gym.make()`—see [`learn.py`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/examples/learn.py) for an example
```
>>> env = gym.make('the-aviary-v1')         # See learn.py
```
Then, the environment can be stepped with
```
>>> obs = env.reset()
>>> for _ in range(10*240):
>>>     obs, reward, done, info = env.step(env.action_space.sample())
>>>     env.render()
>>>     if done: obs = env.reset()
>>> env.close()
```




### Action Space
The action space is a [`Dict()`](https://github.com/openai/gym/blob/master/gym/spaces/dict.py) of [`Box(4,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py)'s containing the desired control inputs

Keys are `"0"`, `"1"`, .., `"n"`—where `n` is the number of drones

For all `Physics` implementations—except `PYB_PM` and `PYB_KIN`—these are the rotation speeds of all motors ranging from `-1` to `1` if  `normalized_spaces=True`, or from `0` to `SingleDroneEnv.MAX_RPM` otherwise

For `physics=Physics.PYB_PM` and `PYB_KIN`, the control inputs are the desired acceleration and velocity, respectively




### Observation Space
The observation space is a [`Dict()`](https://github.com/openai/gym/blob/master/gym/spaces/dict.py) of pairs `{"state": Box(20,), "neighbors": MultiBinary(num_drones)}`

Keys are `"0"`, `"1"`, .., `"n"`—where `n` is the number of drones

Each [`Box(20,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py) contains the drone's
- X, Y, Z position in `WORLD_FRAME` (3 value, meters unless normalized)
- Quaternion orientation in `WORLD_FRAME` (4 values)
- Roll, pitch and yaw angles in `WORLD_FRAME` (3 values, radians unless normalized)
- The velocity vector in `WORLD_FRAME` (3 values, m/s unless normalized)
- Angular velocities in `WORLD_FRAME` (3 values, rad/s unless normalized)
- Motors' speeds (4 values, RPM)

Check [`ProblemSpecificFunctions.clipAndNormalizeState()`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/ProblemSpecificFunctions.py) for the mapping from raw simulation data to normalized observations in the `[-1,1]` range (i.e., when `normalized_spaces==True`)

Each [`MultiBinary(num_drones)`](https://github.com/openai/gym/blob/master/gym/spaces/multi_binary.py) contains the drone's own row of the multi-robot system adjacency matrix


### NOTE: Single-agent Spaces
When `num_drones==1`, action and observations spaces are simplified to [`Box(4,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py) and [`Box(20,)`](https://github.com/openai/gym/blob/master/gym/spaces/box.py), respectively 




## ProblemSpecificFunctions
Class [`ProblemSpecificFunctions`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/ProblemSpecificFunctions.py) contains implementations of *reward*, *done*, and *normalization* functions that can be selected when create an environment with `problem=Problem.CUSTOM`




### Reward
Reward functions can/should be modified in class [`ProblemSpecificFunctions`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/ProblemSpecificFunctions.py), for example
```
>>> def rewardFunction(self, obs):
>>>     if self.PROBLEM==Problem.DEFAULT:
>>>     ...
>>>     elif self.PROBLEM==Problem.CUSTOM:  # Use with: >>> env = Aviary(problem=Problem.CUSTOM)
>>>         height = obs[2]
>>>         if height > 0.5: return 1000
>>>         elif height > 0.1: return 100
>>>         else: return -1
>>>     else: print("[ERROR] unknown user")
```




### Done
Stopping conditions can/should be modified in class [`ProblemSpecificFunctions`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/ProblemSpecificFunctions.py), for example
```
>>> def doneFunction(self, obs, sim_time):
>>>     if self.PROBLEM==Problem.DEFAULT:
>>>     ...
>>>     elif self.PROBLEM==Problem.CUSTOM:  # Use with: >>> env = Aviary(problem=Problem.CUSTOM)
>>>         x = obs[0]; y = obs[1]; z = obs[2] 
>>>         roll = obs[7]; pitch = obs[8]
>>>         if np.abs(x)>.5 or np.abs(y)>.5 or z>=1 \
>>>                 or np.abs(roll)>np.pi/2 or np.abs(pitch)>np.pi/2 \
>>>                 or sim_time > 5: 
>>>             return True
>>>         else: 
>>>             return False
>>>     else: print("[ERROR] unknown user")
```




### Drag, Ground Effect, and Downwash Models
Simple drag, ground effect, and downwash models can be included in the simulation initializing `Aviary()` with `physics=Physics.PYB_GND_DRAG_DW`, these are based on the system identification of [Forster (2015)](http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf) (Eq. 4.2), the analytical model used as a baseline for comparison by [Shi et al. (2019)](https://arxiv.org/pdf/1811.08027.pdf) (Eq. 15), and [DSL](https://www.dynsyslab.org/vision-news/)'s experimental work

Check the implementations of `_drag()`, `_groundEffect()`, and `_downwash()` in class [`Aviary`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/Aviary.py) for more detail




## Control
Class [`Control`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/Control.py) contains implementations of 2 PID controllers to command the quadcopters, for example
```   
>>> ctrl = [Control(env, control_type=ControlType.PID) for i in range(num_drones)]  # Initialize "num_drones" controllers
>>> ...
>>> for i in range(num_drones):                                                     # Compute control for each drone
>>>     action[str(i)], _, _ = ctrl[i].computeControlFromState( \                   # Write the action in a dictionary
>>>                                    control_timestep=env.TIMESTEP, \
>>>                                    state=obs[str(i)]["state"], \
>>>                                    target_pos=TARGET_POS)
```




## Logger
Class [`Logger`](https://github.com/JacopoPan/gym-pybullet-drones/blob/master/gym_pybullet_drones/envs/Logger.py) contains helper functions to save and plot simulation data, for example
```
>>> logger = Logger(duration_sec=T, simulation_freq_hz=freq, num_drones=num_drones) # Initialize the logger
>>> ...
>>> for i in range(NUM_DRONES):             # Log information for each drone
>>>     logger.log(drone=i, \
>>>                timestamp=K/env.SIM_FREQ, \
>>>                state= obs[str(i)]["state"], \
>>>                control=np.hstack([ TARGET_POS, np.zeros(9) ]))   
>>> ...
>>> logger.save()                           # Save data to file
>>> logger.plot()                           # Plot data
```




## Citation
If you wish, please refer to this work as
```
@MISC{gym-pybullet-drones2020,
        author =   {Panerati, Jacopo and Zheng, Hehui and Zhou, SiQi and Xu, James and Prorok, Amanda and Sch\"{o}llig, Angela P.},
        title =    {Learning to Fly: a Gym environment based on PyBullet to simulate and learn the control of multiple nano-quadcopters},
        howpublished = {\url{https://github.com/JacopoPan/gym-pybullet-drones}},
        year = {2020}
}
```




## References
- Nathan Michael, Daniel Mellinger, Quentin Lindsey, Vijay Kumar (2010) [*The GRASP Multiple Micro UAV Testbed*](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf)
- Benoit Landry (2014) [*Planning and Control for Quadrotor Flight through Cluttered Environments*](http://groups.csail.mit.edu/robotics-center/public_papers/Landry15)
- Julian Forster (2015) [*System Identification of the Crazyflie 2.0 Nano Quadrocopter*](http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf)
- Carlos Luis and Jeroome Le Ny (2016) [*Design of a Trajectory Tracking Controller for a Nanoquadcopter*](https://arxiv.org/pdf/1608.05786.pdf)
- Guanya Shi, Xichen Shi, Michael O’Connell, Rose Yu, Kamyar Azizzadenesheli, Animashree Anandkumar, Yisong Yue, Soon-Jo Chung (2019)
[*Neural Lander: Stable Drone Landing Control Using Learned Dynamics*](https://arxiv.org/pdf/1811.08027.pdf)

-----

University of Toronto's [Dynamic Systems Lab](https://github.com/utiasDSL) — [Vector Institute](https://github.com/VectorInstitute) — University of Cambridge's [Prorok Lab](https://github.com/proroklab) — [Mitacs](https://www.mitacs.ca/en/projects/multi-agent-reinforcement-learning-decentralized-uavugv-cooperative-exploration)
