# gym-pybullet-drones

This is a minimalist refactoring of the original `gym-pybullet-drones` repository, designed for compatibility with [`gymnasium`](https://github.com/Farama-Foundation/Gymnasium) and [`stable-baselines3` 2.0](https://github.com/DLR-RM/stable-baselines3/pull/1327).

If you prefer to access the original codebase, please `git checkout [paper|old-master]` after cloning the repo, and refer to the respective `README.md`.

<img src="files/readme_images/helix.gif" alt="formation flight" width="350"> <img src="files/readme_images/helix.png" alt="control info" width="450">

## Installation

```sh
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones/

conda create -n drones python=3.10
conda activate drones

pip3 install --upgrade pip
pip3 install -e .
```

## Use

### PID position control example

```sh
cd gym_pybullet_drones/examples/
python3 pid.py
```

### PPO RL example

```sh
cd gym_pybullet_drones/examples/
python3 learn.py
```

### Betaflight SITL example

In one terminal, run the Betaflight SITL binary

```sh
git clone  https://github.com/betaflight/betaflight
cd betaflight/
make TARGET=SITL # comment out this line: https://github.com/betaflight/betaflight/blob/c41b458e5891205da50465caeec0c1aa83beeb0c/src/main/main.c#L52
betaflight/obj/main/betaflight_SITL.elf
```

In another terminal, run the example

```sh
cd gym_pybullet_drones/examples/
python3 beta.py
```

## Troubleshooting

- On Ubuntu, with an NVIDIA card, if you receive a "Failed to create and OpenGL context" message, launch `nvidia-settings` and under "PRIME Profiles" select "NVIDIA (Performance Mode)", reboot and try again.

Run all tests from the top folder with

```sh
pytest tests/
```

## Citation

If you wish, please cite [our IROS 2021 paper](https://arxiv.org/abs/2103.02142) ([and original codebase](https://github.com/utiasDSL/gym-pybullet-drones/tree/paper)) as

```bibtex
@INPROCEEDINGS{panerati2021learning,
      title={Learning to Fly---a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control}, 
      author={Jacopo Panerati and Hehui Zheng and SiQi Zhou and James Xu and Amanda Prorok and Angela P. Schoellig},
      booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      year={2021},
      volume={},
      number={},
      pages={7512-7519},
      doi={10.1109/IROS51168.2021.9635857}
}
```

## References

- Nathan Michael, Daniel Mellinger, Quentin Lindsey, Vijay Kumar (2010) [*The GRASP Multiple Micro UAV Testbed*](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf)
- Benoit Landry (2014) [*Planning and Control for Quadrotor Flight through Cluttered Environments*](http://groups.csail.mit.edu/robotics-center/public_papers/Landry15)
- Julian Forster (2015) [*System Identification of the Crazyflie 2.0 Nano Quadrocopter*](http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf)
- Carlos Luis and Jeroome Le Ny (2016) [*Design of a Trajectory Tracking Controller for a Nanoquadcopter*](https://arxiv.org/pdf/1608.05786.pdf)
- Shital Shah, Debadeepta Dey, Chris Lovett, and Ashish Kapoor (2017) [*AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles*](https://arxiv.org/pdf/1705.05065.pdf)
- Eric Liang, Richard Liaw, Philipp Moritz, Robert Nishihara, Roy Fox, Ken Goldberg, Joseph E. Gonzalez, Michael I. Jordan, and Ion Stoica (2018) [*RLlib: Abstractions for Distributed Reinforcement Learning*](https://arxiv.org/pdf/1712.09381.pdf)
- Antonin Raffin, Ashley Hill, Maximilian Ernestus, Adam Gleave, Anssi Kanervisto, and Noah Dormann (2019) [*Stable Baselines3*](https://github.com/DLR-RM/stable-baselines3)
- Guanya Shi, Xichen Shi, Michael O’Connell, Rose Yu, Kamyar Azizzadenesheli, Animashree Anandkumar, Yisong Yue, and Soon-Jo Chung (2019)
[*Neural Lander: Stable Drone Landing Control Using Learned Dynamics*](https://arxiv.org/pdf/1811.08027.pdf)
- Mikayel Samvelyan, Tabish Rashid, Christian Schroeder de Witt, Gregory Farquhar, Nantas Nardelli, Tim G. J. Rudner, Chia-Man Hung, Philip H. S. Torr, Jakob Foerster, and Shimon Whiteson (2019) [*The StarCraft Multi-Agent Challenge*](https://arxiv.org/pdf/1902.04043.pdf)
- C. Karen Liu and Dan Negrut (2020) [*The Role of Physics-Based Simulators in Robotics*](https://www.annualreviews.org/doi/pdf/10.1146/annurev-control-072220-093055)
- Yunlong Song, Selim Naji, Elia Kaufmann, Antonio Loquercio, and Davide Scaramuzza (2020) [*Flightmare: A Flexible Quadrotor Simulator*](https://arxiv.org/pdf/2009.00563.pdf)

## TODO

- [ ] Revise IMU transforms in Betaflight SITL exampl (+ `_dynamics()`) @ykeuter
- [ ] Add motor delay @JacopoPan / @spencerteetaert
- [ ] Add `cffirmware` SITL support @spencerteetaert
- [ ] Replace `BaseSingleAgentAviary` and `BaseMultiAgentAviary` with a single `RLAviary` @JacopoPan
- [ ] Add a multi-agent MDP with 2-drone chase through a gate @JacopoPan

## Desiderata

- [ ] Add `yaml` configuration files
- [ ] Replace `rpy` with quaternions (and `ang_vel` with body rates) in `obs`
- [ ] Vectorize any remaining loop @JacopoPan

-----
> University of Toronto's [Dynamic Systems Lab](https://github.com/utiasDSL) / [Vector Institute](https://github.com/VectorInstitute) / University of Cambridge's [Prorok Lab](https://github.com/proroklab)
