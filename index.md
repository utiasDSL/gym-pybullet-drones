[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/-zyrmneaz88/maxresdefault.jpg)](https://www.youtube.com/watch?v=-zyrmneaz88)




## Citation
```
@MISC{panerati2021learning,
      title={Learning to Fly---a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control}, 
      author={Jacopo Panerati and Hehui Zheng and SiQi Zhou and James Xu and Amanda Prorok and Angela P. Schoellig},
      year={2021},
      eprint={2103.02142},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```




[Simple](https://en.wikipedia.org/wiki/KISS_principle) OpenAI [Gym environment](https://gym.openai.com/envs/#classic_control) based on [PyBullet](https://github.com/bulletphysics/bullet3) for multi-agent reinforcement learning

The default `DroneModel.CF2X` dynamics are based on [Bitcraze's Crazyflie 2.x nano-quadrotor](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf)

<img src="files/readme_images/helix.gif" alt="formation flight" width="360"> <img src="files/readme_images/helix.png" alt="control info" width="450">




## Requirements and Installation
The repo was written using *Python 3* on *macOS 10.15* and tested on *macOS 11*, *Ubuntu 18.04*




### On *macOS* and *Ubuntu*
Major dependencies are [`gym`](https://gym.openai.com/docs/),  [`pybullet`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#), 
[`stable-baselines3`](https://stable-baselines3.readthedocs.io/en/master/guide/quickstart.html), and [`rllib`](https://docs.ray.io/en/master/rllib.html)

```
pip3 install --upgrade numpy matplotlib Pillow cycler 
pip3 install --upgrade gym pybullet stable_baselines3 'ray[rllib]'
```
Video recording requires to have [`ffmpeg`](https://ffmpeg.org) installed, on *macOS*
```
$ brew install ffmpeg
```
On *Ubuntu*
```
$ sudo apt install ffmpeg
```
The repo is structured as a [Gym Environment](https://github.com/openai/gym/blob/master/docs/creating-environments.md)
and can be installed with `pip install --editable`
```
$ git clone https://github.com/utiasDSL/gym-pybullet-drones.git
$ cd gym-pybullet-drones/
$ pip3 install -e .
```




### On *Windows*
Check out these step-by-step [instructions](https://github.com/utiasDSL/gym-pybullet-drones/tree/master/assignments#on-windows) written by Dr. Karime Pereida for *Windows 10*




## Use, Examples, and Documentation

**Check out file [`README.md`](https://github.com/utiasDSL/gym-pybullet-drones/blob/master/README.md) in the the repo's [`master` branch](https://github.com/utiasDSL/gym-pybullet-drones) for further details**

### Track
<img src="files/readme_images/wp.gif" alt="sparse way points flight" width="360"> <img src="files/readme_images/wp.png" alt="control info" width="450">

### RGB, Depth, and Segmentation Views
<img src="files/readme_images/rgb.gif" alt="rgb view" width="270"> <img src="files/readme_images/dep.gif" alt="depth view" width="270"> <img src="files/readme_images/seg.gif" alt="segmentation view" width="270">

### Downwash
<img src="files/readme_images/downwash.gif" alt="downwash example" width="360"> <img src="files/readme_images/downwash.png" alt="control info" width="450">

### Learn
<img src="files/readme_images/learn1.gif" alt="learning 1" width="400"><img src="files/readme_images/learn2.gif" alt="learning 2" width="400">
<img src="files/readme_images/learn3.gif" alt="learning 3" width="400"><img src="files/readme_images/learn4.gif" alt="learning 4" width="400">

### Debug
<img src="files/readme_images/crash.gif" alt="yaw saturation" width="360"> <img src="files/readme_images/crash.png" alt="control info" width="450">

### Compare
<img src="files/readme_images/trace_comparison.gif" alt="pid flight on sine trajectroy" width="360"> <img src="files/readme_images/trace_comparison.png" alt="control info" width="450">





## References
- Nathan Michael, Daniel Mellinger, Quentin Lindsey, Vijay Kumar (2010) [*The GRASP Multiple Micro UAV Testbed*](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf)
- Benoit Landry (2014) [*Planning and Control for Quadrotor Flight through Cluttered Environments*](http://groups.csail.mit.edu/robotics-center/public_papers/Landry15)
- Julian Forster (2015) [*System Identification of the Crazyflie 2.0 Nano Quadrocopter*](http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf)
- Carlos Luis and Jeroome Le Ny (2016) [*Design of a Trajectory Tracking Controller for a Nanoquadcopter*](https://arxiv.org/pdf/1608.05786.pdf)
- Shital Shah, Debadeepta Dey, Chris Lovett, and Ashish Kapoor (2017) [*AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles*](https://arxiv.org/pdf/1705.05065.pdf)
- Eric Liang, Richard Liaw, Philipp Moritz, Robert Nishihara, Roy Fox, Ken Goldberg, Joseph E. Gonzalez, Michael I. Jordan, and Ion Stoica (2018) [*RLlib: Abstractions for Distributed Reinforcement Learning*](https://arxiv.org/pdf/1712.09381.pdf)
- Guanya Shi, Xichen Shi, Michael O’Connell, Rose Yu, Kamyar Azizzadenesheli, Animashree Anandkumar, Yisong Yue, and Soon-Jo Chung (2019)
[*Neural Lander: Stable Drone Landing Control Using Learned Dynamics*](https://arxiv.org/pdf/1811.08027.pdf)
- Antonin Raffin, Ashley Hill, Maximilian Ernestus, Adam Gleave, Anssi Kanervisto, and Noah Dormann (2019) [*Stable Baselines3*](https://github.com/DLR-RM/stable-baselines3)
- Mikayel Samvelyan, Tabish Rashid, Christian Schroeder de Witt, Gregory Farquhar, Nantas Nardelli, Tim G. J. Rudner, Chia-Man Hung, Philip H. S. Torr, Jakob Foerster, and Shimon Whiteson (2019) [*The StarCraft Multi-Agent Challenge*](https://arxiv.org/pdf/1902.04043.pdf)
- Yunlong Song, Selim Naji, Elia Kaufmann, Antonio Loquercio, and Davide Scaramuzza (2020) [*Flightmare: A Flexible Quadrotor Simulator*](https://arxiv.org/pdf/2009.00563.pdf)
- C. Karen Liu and Dan Negrut (2020) [*The Role of Physics-Based Simulators in Robotics*](https://www.annualreviews.org/doi/pdf/10.1146/annurev-control-072220-093055)




## Bonus GIF for Scrolling this Far

<img src="files/readme_images/2020.gif" alt="formation flight" width="360"> <img src="files/readme_images/2020.png" alt="control info" width="450">

-----

> UofT's [Dynamic Systems Lab](https://github.com/utiasDSL) / [Vector Institute](https://github.com/VectorInstitute) / Cambridge's [Prorok Lab](https://github.com/proroklab) / [Mitacs](https://www.mitacs.ca/en/projects/multi-agent-reinforcement-learning-decentralized-uavugv-cooperative-exploration)


