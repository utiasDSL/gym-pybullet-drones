"""Script demonstrating the use of `gym_pybullet_drones`'s Gymnasium interface.

Classes HoverAviary and MultiHoverAviary are used as learning envs for the PPO algorithm.

Example
-------
In a terminal, run as:

    $ python learn.py --multiagent false
    $ python learn.py --multiagent true

Notes
-----
This is a minimal working example integrating `gym-pybullet-drones` with
reinforcement learning library `stable-baselines3`.

"""
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold, StopTrainingOnMaxEpisodes, \
    StopTrainingOnNoModelImprovement
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.envs.RlHoverAviary import RlHoverAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.envs.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.MultiHoverAviary import MultiHoverAviary

from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import ObservationType, ActionType

DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

DEFAULT_OBS = ObservationType('kin') # 'kin' or 'rgb'
DEFAULT_ACT = ActionType('vel') # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'
DEFAULT_AGENTS = 2
DEFAULT_MA = True
duration_sec = 50
simulation_freq_hz = 300


@dataclass(frozen=True)
class TimeData:
  T: float # длительность во времени
  fs: int # частота дискретизации
  n: int = field(init=False)# число отсчетов
  dt: float = field(init=False) # длительность отсчета времени
  t: np.ndarray = field(init=False) # отсчеты времени

  def __post_init__(self):
    object.__setattr__(self, 'n', int(self.T * self.fs))
    object.__setattr__(self, 'dt', 1/self.fs)
    object.__setattr__(self, 't', np.arange(self.n) * self.dt)

  def sample(self, fs):
    return TimeData(T=self.T, fs=fs)

def run(multiagent=DEFAULT_MA,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        gui=DEFAULT_GUI,
        plot=True,
        colab=DEFAULT_COLAB,
        record_video=DEFAULT_RECORD_VIDEO,
        local=True):

    # создаем файл
    filename = os.path.join(output_folder, 'save-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S"))
    if not os.path.exists(filename):
        os.makedirs(filename+'/')

    INIT_XYZS = np.array([
        [0, 10, 10],
        [0, 50, 10]
    ])
    INIT_RPYS = np.array([
        [0, 0, 0],
        [0, 0, np.pi / 3]
    ])


    # в зависимости много аппартов или один создаем средудля обучения и среду для оценки

    train_env = make_vec_env(RlHoverAviary,
                             env_kwargs=dict(num_drones=DEFAULT_AGENTS,
                                             initial_xyzs=INIT_XYZS,
                                             initial_rpys=INIT_RPYS,
                                             obs=DEFAULT_OBS,
                                             act=DEFAULT_ACT
                                             ),
                             n_envs=1,
                             seed=0)

    eval_env = RlHoverAviary(num_drones=DEFAULT_AGENTS,
                                initial_xyzs=INIT_XYZS,
                                initial_rpys=INIT_RPYS,
                                obs=DEFAULT_OBS,
                                act=DEFAULT_ACT)

    #### Check the environment's spaces ########################
    print('[INFO] Action space:', train_env.action_space)
    print('[INFO] Observation space:', train_env.observation_space)

    #### Train the model #######################################
    # создаем модель с PPO
    model = PPO('MlpPolicy',
                train_env,
                # tensorboard_log=filename+'/tb/',
                verbose=1)

    #### Target cumulative rewards (problem-dependent) ##########
    target_reward = 7000

    callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=target_reward, verbose=1)
    #callback_on_best = StopTrainingOnNoModelImprovement(max_no_improvement_evals=3, min_evals=10, verbose=1)
    #stop_traning = StopTrainingOnMaxEpisodes(max_episodes=5, verbose=1)
    eval_callback = EvalCallback(eval_env,
                                 callback_on_new_best=callback_on_best,
                                 verbose=1,
                                 n_eval_episodes=1,
                                 best_model_save_path=filename+'/',
                                 log_path=filename+'/',
                                 eval_freq=int(1000),
                                 deterministic=True,
                                 render=False)
    model.learn(total_timesteps=int(1e5) if local else int(1e2), # shorter training in GitHub Actions pytest
                callback=eval_callback,
                log_interval=100)

    #### Save the model ########################################
    model.save(filename+'/final_model.zip')
    print(filename)

    #### Print training progression ############################
    with np.load(filename+'/evaluations.npz') as data:
        for j in range(data['timesteps'].shape[0]):
            print(str(data['timesteps'][j])+","+str(data['results'][j][0]))

    ############################################################
    ############################################################
    ############################################################
    ############################################################
    ############################################################

    if local:
        input("Press Enter to continue...")

    # if os.path.isfile(filename+'/final_model.zip'):
    #     path = filename+'/final_model.zip'
    if os.path.isfile(filename+'/best_model.zip'):
        path = filename+'/best_model.zip'
    else:
        print("[ERROR]: no model under the specified path", filename)
    model = PPO.load(path)

    #### Show (and record a video of) the model's performance ##

    test_env = RlHoverAviary(gui=gui,
                                    num_drones=DEFAULT_AGENTS,
                                    initial_xyzs=INIT_XYZS,
                                    initial_rpys=INIT_RPYS,
                                    obs=DEFAULT_OBS,
                                    act=DEFAULT_ACT,
                                    record=record_video)
    test_env_nogui = RlHoverAviary(num_drones=DEFAULT_AGENTS, initial_xyzs=INIT_XYZS,
                                             initial_rpys=INIT_RPYS, obs=DEFAULT_OBS, act=DEFAULT_ACT)

    logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
                num_drones=DEFAULT_AGENTS,
                output_folder=output_folder,
                colab=colab
                )

    mean_reward, std_reward = evaluate_policy(model,
                                              test_env_nogui,
                                              n_eval_episodes=10
                                              )
    print("\n\n\nMean reward ", mean_reward, " +- ", std_reward, "\n\n")

    obs, info = test_env.reset(seed=42, options={})
    start = time.time()
    for i in range((test_env.EPISODE_LEN_SEC)*test_env.CTRL_FREQ):
        action, _states = model.predict(obs,
                                        deterministic=True
                                        )
        obs, reward, terminated, truncated, info = test_env.step(action)
        #obs2 = obs.squeeze()
        #act2 = action.squeeze()
        #print("Obs:", obs, "\tAction", action, "\tReward:", reward, "\tTerminated:", terminated, "\tTruncated:", truncated)
        if DEFAULT_OBS == ObservationType.KIN:
            for d in range(DEFAULT_AGENTS):
                logger.log(drone=d,
                    timestamp=i/test_env.CTRL_FREQ,
                    state=np.hstack([obs[d][0:3],
                                        np.zeros(4),
                                        obs[d][3:15],
                                        action[d]
                                        ]),
                    control=np.zeros(12)
                    )

        test_env.render()
        #print(terminated)
        if gui:
            sync(i, start, test_env.CTRL_TIMESTEP)
        #if terminated:
            #obs = test_env.reset(seed=42, options={})
    test_env.close()

    if plot and DEFAULT_OBS == ObservationType.KIN:
        logger.plot_trajct(trajs=test_env.trajs)

if __name__ == '__main__':
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script')
    parser.add_argument('--multiagent',         default=DEFAULT_MA,            type=str2bool,      help='Whether to use example LeaderFollower instead of Hover (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,           type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,  type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB,         type=bool,          help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
