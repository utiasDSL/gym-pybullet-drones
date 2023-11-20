"""Script demonstrating the use of `gym_pybullet_drones`'s Gymnasium interface.

Class HoverAviary is used as a learning env for the PPO algorithm.

Example
-------
In a terminal, run as:

    $ python learn.py

Notes
-----
This is a minimal working example integrating `gym-pybullet-drones` with 
reinforcement learning library `stable-baselines3`.

"""
import time
import argparse
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.policies import ActorCriticPolicy as a2cppoMlpPolicy
from stable_baselines3.common.policies import ActorCriticCnnPolicy as a2cppoCnnPolicy
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.envs.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.LeaderFollowerAviary import LeaderFollowerAviary
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import ObservationType, ActionType

DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

DEFAULT_ALGO = 'ppo'
DEFAULT_OBS = ObservationType('kin')
DEFAULT_ACT = ActionType('rpm')

def run(output_folder=DEFAULT_OUTPUT_FOLDER, gui=DEFAULT_GUI, plot=True, colab=DEFAULT_COLAB, record_video=DEFAULT_RECORD_VIDEO):

    MULTI_AGENT = False

    sa_env_kwargs = dict(obs=DEFAULT_OBS, act=DEFAULT_ACT)

    if not MULTI_AGENT:
        # train_env = gym.make('hover-aviary-v0')
        train_env = make_vec_env(HoverAviary,
                                 env_kwargs=sa_env_kwargs,
                                 n_envs=2,
                                 seed=0
                                 )
        eval_env = gym.make('hover-aviary-v0')
    else:
        train_env = gym.make('leaderfollower-aviary-v0')
    #### Check the environment's spaces ########################
    print('[INFO] Action space:', train_env.action_space)
    print('[INFO] Observation space:', train_env.observation_space)

    #### Train the model #######################################
    onpolicy_kwargs = dict(activation_fn=torch.nn.ReLU,
                           net_arch=[512, 512, dict(vf=[256, 128], pi=[256, 128])]
                           ) # or None

    # model = PPO('MlpPolicy',
    #             train_env,
    #             verbose=1
    #             )
    model = PPO(a2cppoMlpPolicy, # or a2cppoCnnPolicy
                train_env,
                # policy_kwargs=onpolicy_kwargs,
                # tensorboard_log=filename+'/tb/',
                verbose=1
                )
    callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=1000,
                                                     verbose=1
                                                     )
    eval_callback = EvalCallback(eval_env,
                                 callback_on_new_best=callback_on_best,
                                 verbose=1,
                                 # best_model_save_path=filename+'/',
                                 # log_path=filename+'/',
                                 eval_freq=int(2000),
                                 deterministic=True,
                                 render=False
                                 )
    model.learn(total_timesteps=10000, #int(1e12),
                callback=eval_callback,
                log_interval=100,
                )
    # model.learn(total_timesteps=10000) # Typically not enough

    #### Save the model ########################################
    # model.save(filename+'/success_model.zip')
    # print(filename)

    # #### Print training progression ############################
    # with np.load(filename+'/evaluations.npz') as data:
    #     for j in range(data['timesteps'].shape[0]):
    #         print(str(data['timesteps'][j])+","+str(data['results'][j][0]))








    # if os.path.isfile(exp+'/success_model.zip'):
    #     path = exp+'/success_model.zip'
    # elif os.path.isfile(exp+'/best_model.zip'):
    #     path = exp+'/best_model.zip'
    # else:
    #     print("[ERROR]: no model under the specified path", exp)
    # model = PPO.load(path)



    #### Show (and record a video of) the model's performance ##
    if not MULTI_AGENT:
        test_env = HoverAviary(gui=gui,
                      record=record_video
                     )
        test_env_nogui = HoverAviary()
        logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
                    num_drones=1,
                    output_folder=output_folder,
                    colab=colab
                    )
    else:
        test_env = LeaderFollowerAviary(gui=gui,
                               record=record_video
                              )
        test_env_nogui = LeaderFollowerAviary()
        logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
                    num_drones=2,
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
    for i in range(3*test_env.CTRL_FREQ):
        action, _states = model.predict(obs,
                                        deterministic=True
                                        )
        obs, reward, terminated, truncated, info = test_env.step(action)
        obs2 = obs.squeeze()
        act2 = action.squeeze()
        print("Obs:", obs, "\tAction", action, "\tReward:", reward, "\tTerminated:", terminated, "\tTruncated:", truncated)
        if not MULTI_AGENT:
            logger.log(drone=0,
                   timestamp=i/test_env.CTRL_FREQ,
                   state=np.hstack([obs2[0:3],
                                    np.zeros(4),
                                    obs2[3:15],
                                    act2
                                    ]),
                   control=np.zeros(12)
                   )
        else:
            for d in range(2):
                logger.log(drone=d,
                    timestamp=i/test_env.CTRL_FREQ,
                    state=np.hstack([obs2[d][0:3],
                                        np.zeros(4),
                                        obs2[d][3:15],
                                        act2[d]
                                        ]),
                    control=np.zeros(12)
                    )
        test_env.render()
        print(terminated)
        sync(i, start, test_env.CTRL_TIMESTEP)
        if terminated:
            obs = test_env.reset(seed=42, options={})
    test_env.close()

    if plot:
        logger.plot()

if __name__ == '__main__':
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning example script using HoverAviary')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
