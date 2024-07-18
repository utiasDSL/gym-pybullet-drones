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
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.evaluation import evaluate_policy

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
DEFAULT_ACT = ActionType('discrete_2d') # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'
DEFAULT_AGENTS = 2
DEFAULT_MA = False

def run(multiagent=DEFAULT_MA, output_folder=DEFAULT_OUTPUT_FOLDER, gui=DEFAULT_GUI, plot=True, colab=DEFAULT_COLAB, record_video=DEFAULT_RECORD_VIDEO, local=True):
    DEFAULT_OBS = ObservationType('kin') # 'kin' or 'rgb'

    filename = os.path.join(output_folder, '/app/results/save-07.15.2024_13.40.57')
    if not os.path.exists(filename):
        os.makedirs(filename+'/')

    if not multiagent:
        train_env = make_vec_env(HoverAviary,
                                 env_kwargs=dict(obs=DEFAULT_OBS, act=DEFAULT_ACT),
                                 n_envs=1,
                                 seed=0
                                 )
        eval_env = HoverAviary(obs=DEFAULT_OBS, act=DEFAULT_ACT)
    else:
        train_env = make_vec_env(MultiHoverAviary,
                                 env_kwargs=dict(num_drones=DEFAULT_AGENTS, obs=DEFAULT_OBS, act=DEFAULT_ACT),
                                 n_envs=1,
                                 seed=0
                                 )
        eval_env = MultiHoverAviary(num_drones=DEFAULT_AGENTS, obs=DEFAULT_OBS, act=DEFAULT_ACT)

    #### Check the environment's spaces ########################
    print('[INFO] Action space:', train_env.action_space)
    print('[INFO] Observation space:', train_env.observation_space)

    #### Print training progression ############################
    best = 0
    with np.load(filename+'/evaluations.npz') as data:
        for j in range(data['timesteps'].shape[0]):
            print(str(data['timesteps'][j])+","+str(data['results'][j][0]))
            if data['results'][j][0] > best:
                best = data['results'][j][0]

    print("Best reward:", best)

    ############################################################
    ############################################################
    ############################################################
    ############################################################
    ############################################################

    # if os.path.isfile(filename+'/final_model.zip'):
    #     path = filename+'/final_model.zip'
    if os.path.isfile(filename+'/best_model.zip'):
        path = filename+'/best_model.zip'
    else:
        print("[ERROR]: no model un5der the specified path", filename)
    model = SAC.load(path)

    #### Show (and record a video of) the model's performance ##
    if not multiagent:
        test_env = HoverAviary(gui=gui,
                               obs=DEFAULT_OBS,
                               act=DEFAULT_ACT,
                               record=record_video, 
                               initial_xyzs=np.array([[0,0,0.1125]]))
        test_env_nogui = HoverAviary(obs=DEFAULT_OBS, act=DEFAULT_ACT)
    else:
        test_env = MultiHoverAviary(gui=gui,
                                        num_drones=DEFAULT_AGENTS,
                                        obs=DEFAULT_OBS,
                                        act=DEFAULT_ACT,
                                        record=record_video)
        test_env_nogui = MultiHoverAviary(num_drones=DEFAULT_AGENTS, obs=DEFAULT_OBS, act=DEFAULT_ACT)
    logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
                num_drones=DEFAULT_AGENTS if multiagent else 1,
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
        obs2 = obs.squeeze()
        act2 = action.squeeze()
        print("Obs:", obs, "\tAction", action, "\tReward:", reward, "\tTerminated:", terminated, "\tTruncated:", truncated)
        
        if i == 5*test_env.CTRL_FREQ:
            obs3 = obs2

        #TODO THIS IS JUST FOR TESTING!
        DEFAULT_OBS = ObservationType.POS_RPY

        if DEFAULT_OBS == ObservationType.KIN:
            if not multiagent:
                logger.log(drone=0,
                    timestamp=i/test_env.CTRL_FREQ,
                    state=np.hstack([obs2[0:3],
                                        np.zeros(4),
                                        obs2[3:15],
                                        act2[1],
                                        act2[1],
                                        act2[0],
                                        act2[0]
                                        ]),
                    control=np.zeros(12)
                    )
            else:
                for d in range(DEFAULT_AGENTS):
                    logger.log(drone=d,
                        timestamp=i/test_env.CTRL_FREQ,
                        state=np.hstack([obs2[d][0:3],
                                            np.zeros(4),
                                            obs2[d][3:15],
                                            act2[d]
                                            ]),
                        control=np.zeros(12)
                        )
        elif DEFAULT_OBS == ObservationType.POS_RPY:
            if DEFAULT_ACT == ActionType.DISCRETE_2D or DEFAULT_ACT == ActionType.DISCRETE_3D:
                action_to_take = np.argmax(act2)
                strong = 0.05
                weak = 0.025
                rpm = np.zeros(4)
                if action_to_take == 0: #STRONG LEFT
                    rpm[:2] = np.repeat(test_env.HOVER_RPM * (1+strong), 2)
                    rpm[2:] = np.repeat(test_env.HOVER_RPM, 2)
                elif action_to_take == 1: #WEAK LEFT
                    rpm[:2] = np.repeat(test_env.HOVER_RPM * (1+weak), 2)
                    rpm[2:] = np.repeat(test_env.HOVER_RPM, 2)
                    print("\n\n\n\nweak was selected!!\n\n\n\n")
                elif action_to_take == 2: #UP
                    rpm[:] = np.repeat(test_env.HOVER_RPM * (1+strong), 4)
                elif action_to_take == 3: #HOVER
                    rpm[:] = np.repeat(test_env.HOVER_RPM, 4)
                elif action_to_take == 4: #DOWN
                    rpm[:] = np.repeat(test_env.HOVER_RPM * (1-strong), 4)
                elif action_to_take == 5: #WEAK RIGHT
                    rpm[:2] = np.repeat(test_env.HOVER_RPM, 2)
                    rpm[2:] = np.repeat(test_env.HOVER_RPM * (1+weak), 2)
                    print("\n\n\n\nweak was selected!!\n\n\n\n")
                elif action_to_take == 6: #STRONG RIGHT
                    rpm[:2] = np.repeat(test_env.HOVER_RPM, 2)
                    rpm[2:] = np.repeat(test_env.HOVER_RPM * (1+strong), 2)
                elif action_to_take == 7: #STRONG FORWARD
                    rpm[0] = rpm[2] = test_env.HOVER_RPM * (1+strong)
                    rpm[1] = rpm[3] = test_env.HOVER_RPM
                elif action_to_take == 6: #WEAK RIGHT
                    rpm[:2] = np.repeat(test_env.HOVER_RPM, 2)
                    rpm[2:] = np.repeat(test_env.HOVER_RPM * (1+strong), 2)
                logger.log(drone=0,
                    timestamp=i/test_env.CTRL_FREQ,
                    state=np.hstack([obs2[0:3],
                                        np.zeros(4),
                                        obs2[3:6],
                                        np.zeros(5),
                                        reward,
                                        rpm[:]
                                        ]),
                    control=np.zeros(12)
                    )
        test_env.render()
        print(terminated)
        sync(i, start, test_env.CTRL_TIMESTEP)
        if terminated:
            obs = test_env.reset(seed=42, options={})
    test_env.close()

    print(obs3)

    #if plot and DEFAULT_OBS == ObservationType.KIN:
    logger.plot()
    input("Press Enter to continue...")

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
