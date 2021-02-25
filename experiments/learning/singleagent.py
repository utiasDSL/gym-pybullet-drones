"""Learning script for single agent problems.


Example
-------
To run the script, type in a terminal:

    $ python singleagent.py --env <env> --algo <alg> --obs <ObservationType> --act <ActionType> --cpu <cpu_num>

"""
import os
import time
from datetime import datetime
import argparse
import pathlib
import ruamel.yaml as yaml
import subprocess
import gym
import numpy as np
import torch
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.cmd_util import make_vec_env # Module cmd_util will be renamed to env_util https://github.com/DLR-RM/stable-baselines3/pull/197
from stable_baselines3.common.vec_env import SubprocVecEnv, VecTransposeImage
import algos

from gym_pybullet_drones.envs.single_agent_rl.TakeoffAviary import TakeoffAviary
from gym_pybullet_drones.envs.single_agent_rl.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.single_agent_rl.FlyThruGateAviary import FlyThruGateAviary
from gym_pybullet_drones.envs.single_agent_rl.TuneAviary import TuneAviary
from gym_pybullet_drones.envs.single_agent_rl.BaseSingleAgentAviary import ActionType, ObservationType

import shared_constants

EPISODE_REWARD_THRESHOLD = -0 # Upperbound: rewards are always negative, but non-zero
"""float: Reward threshold to halt the script."""


def eval_policy(policy, train_env, seed, eval_episodes=10):
	eval_env = train_env
	eval_env.seed(seed + 100)

	avg_reward = 0.
	for _ in range(eval_episodes):
		state, done = eval_env.reset(), False
		while not done:
			action = policy.select_action(np.array(state))
			state, reward, done, _ = eval_env.step(action)
			avg_reward += reward

	avg_reward /= eval_episodes

	print("---------------------------------------")
	print(f"Evaluation over {eval_episodes} episodes: {avg_reward}") #:.3f
	print("---------------------------------------")
	return avg_reward


def build_parser():
    parser = argparse.ArgumentParser(description='Single agent reinforcement learning experiments script')
    parser.add_argument('--configs', type=str, default='configs/configs.yaml',
                     nargs='+', required=True)
    parser.add_argument("--save_model", action="store_true")        # Save model and optimizer parameters
    args, remaining = parser.parse_known_args()
    config_ = yaml.safe_load((pathlib.Path(__file__).parent / 'configs/configs.yaml').read_text())
    parser = argparse.ArgumentParser()
    for key, value in config_.items():
        if key=='obs':
            parser.add_argument(f'--{key}', type=ObservationType, default=value)
        elif key=='act':
            parser.add_argument(f'--{key}', type=ActionType, default=value)
        else:
            arg_type = algos.utils.args_type(value)
            parser.add_argument(f'--{key}', type=arg_type, default=arg_type(value))
    return parser

if __name__ == "__main__":

    parser = build_parser()
    ARGS = parser.parse_args()

    #### Save directory ########################################
    filename = os.path.dirname(os.path.abspath(__file__))+'/results/save-'+ARGS.env+'-'+ARGS.algo+'-'+ARGS.obs.value+'-'+ARGS.act.value+'-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    if not os.path.exists(filename):
        os.makedirs(filename+'/')


    #### Warning ###############################################
    if ARGS.env == 'tune' and ARGS.act != ActionType.TUN:
        print("\n\n\n[WARNING] TuneAviary is intended for use with ActionType.TUN\n\n\n")
    if ARGS.act == ActionType.ONE_D_RPM or ARGS.act == ActionType.ONE_D_DYN or ARGS.act == ActionType.ONE_D_PID:
        print("\n\n\n[WARNING] Simplified 1D problem for debugging purposes\n\n\n")
    #### Errors ################################################
        if not ARGS.env in ['takeoff', 'hover']: 
            print("[ERROR] 1D action space is only compatible with Takeoff and HoverAviary")
            exit()
    if ARGS.act == ActionType.TUN and ARGS.env != 'tune' :
        print("[ERROR] ActionType.TUN is only compatible with TuneAviary")
        exit()
    if ARGS.algo in ['sac', 'td3', 'ddpg'] and ARGS.cpu!=1: 
        print("[ERROR] The selected algorithm does not support multiple environments")
        exit()

    #### Uncomment to debug slurm scripts ######################
    # exit()

    env_name = ARGS.env+"-aviary-v0"
    sa_env_kwargs = dict(aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act)
    # train_env = gym.make(env_name, aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS, obs=ARGS.obs, act=ARGS.act) # single environment instead of a vectorized one    
    if env_name == "takeoff-aviary-v0":
        train_env_name = TakeoffAviary

    elif env_name == "hover-aviary-v0":
        train_env_name = HoverAviary

    elif env_name == "flythrugate-aviary-v0":
        train_env_name = FlyThruGateAviary

    elif env_name == "tune-aviary-v0":
        train_env_name = TuneAviary

    train_env = make_vec_env(train_env_name,
                                env_kwargs=sa_env_kwargs,
                                n_envs=ARGS.cpu,
                                seed=0
                                )
    print("[INFO] Action space:", train_env.action_space)
    print("[INFO] Observation space:", train_env.observation_space)
    # check_env(train_env, warn=True, skip_render_check=True)

    state_dim = train_env.observation_space.shape[0]
    action_dim = train_env.action_space.shape[0] 
    max_action = float(train_env.action_space.high[0])

    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": ARGS.discount,
        "tau": ARGS.tau,
        "policy_noise" : ARGS.policy_noise * max_action,
        "noise_clip" : ARGS.noise_clip * max_action,
        "policy_freq" : ARGS.policy_freq
    }

    model = getattr(algos, ARGS.algo)(**kwargs)


    if ARGS.load_model != "":
        policy_file = filename if ARGS.load_model == "default" else ARGS.load_model
        model.load(policy_file)

    replay_buffer = algos.utils.ReplayBuffer(state_dim, action_dim)

    # Evaluate untrained policy
    evaluations = [eval_policy(model, train_env, ARGS.seed)]

    state, done = train_env.reset(), False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    for t in range(int(ARGS.max_timesteps)):
        
        episode_timesteps += 1

        # Select action randomly or according to policy
        if t < ARGS.start_timesteps:
            action = train_env.action_space.sample()
        else:
            action = (
                model.select_action(np.array(state))
                + np.random.normal(0, max_action * ARGS.expl_noise, size=action_dim)
            ).clip(-max_action, max_action)

        # Perform action
        next_state, reward, done, _ = train_env.step(action) 
        done_bool = float(done) #float(done) if episode_timesteps < train_env._max_episode_steps else 0

        # Store data in replay buffer
        replay_buffer.add(state, action, next_state, reward, done_bool)

        state = next_state
        episode_reward += reward

        # Train agent after collecting sufficient data
        if t >= ARGS.start_timesteps:
            model.train(replay_buffer, ARGS.batch_size)

        if done: 
            # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward}")
            # Reset environment
            state, done = train_env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1 

        # Evaluate episode
        if (t + 1) % ARGS.eval_freq == 0:
            evaluations.append(eval_policy(model, train_env, ARGS.seed))
            np.save(filename, evaluations)
            if ARGS.save_model: model.save(filename)






####################################################################################################################
####################################################################################################################
####################################################################################################################
####################################################################################################################
####################################################################################################################



    # #### On-policy algorithms ##################################
    # onpolicy_kwargs = dict(activation_fn=torch.nn.ReLU,
    #                        net_arch=[512, 512, dict(vf=[256, 128], pi=[256, 128])]
    #                        ) # or None
    # if ARGS.algo == 'a2c':
    #     model = A2C(a2cppoMlpPolicy,
    #                 train_env,
    #                 policy_kwargs=onpolicy_kwargs,
    #                 tensorboard_log=filename+'/tb/',
    #                 verbose=1
    #                 ) if ARGS.obs == ObservationType.KIN else A2C(a2cppoCnnPolicy,
    #                                                               train_env,
    #                                                               policy_kwargs=onpolicy_kwargs,
    #                                                               tensorboard_log=filename+'/tb/',
    #                                                               verbose=1
    #                                                               )
    # if ARGS.algo == 'ppo':
    #     model = PPO(a2cppoMlpPolicy,
    #                 train_env,
    #                 policy_kwargs=onpolicy_kwargs,
    #                 tensorboard_log=filename+'/tb/',
    #                 verbose=1
    #                 ) if ARGS.obs == ObservationType.KIN else PPO(a2cppoCnnPolicy,
    #                                                               train_env,
    #                                                               policy_kwargs=onpolicy_kwargs,
    #                                                               tensorboard_log=filename+'/tb/',
    #                                                               verbose=1
    #                                                               )

    # #### Off-policy algorithms #################################
    # offpolicy_kwargs = dict(activation_fn=torch.nn.ReLU,
    #                         net_arch=[512, 512, 256, 128]
    #                         ) # or None # or dict(net_arch=dict(qf=[256, 128, 64, 32], pi=[256, 128, 64, 32]))
    # if ARGS.algo == 'sac':
    #     model = SAC(sacMlpPolicy,
    #                 train_env,
    #                 policy_kwargs=offpolicy_kwargs,
    #                 tensorboard_log=filename+'/tb/',
    #                 verbose=1
    #                 ) if ARGS.obs==ObservationType.KIN else SAC(sacCnnPolicy,
    #                                                             train_env,
    #                                                             policy_kwargs=offpolicy_kwargs,
    #                                                             tensorboard_log=filename+'/tb/',
    #                                                             verbose=1
    #                                                             )
    # if ARGS.algo == 'td3':
    #     model = TD3(td3ddpgMlpPolicy,
    #                 train_env,
    #                 policy_kwargs=offpolicy_kwargs,
    #                 tensorboard_log=filename+'/tb/',
    #                 verbose=1
    #                 ) if ARGS.obs==ObservationType.KIN else TD3(td3ddpgCnnPolicy,
    #                                                             train_env,
    #                                                             policy_kwargs=offpolicy_kwargs,
    #                                                             tensorboard_log=filename+'/tb/',
    #                                                             verbose=1
    #                                                             )
    # if ARGS.algo == 'ddpg':
    #     model = DDPG(td3ddpgMlpPolicy,
    #                 train_env,
    #                 policy_kwargs=offpolicy_kwargs,
    #                 tensorboard_log=filename+'/tb/',
    #                 verbose=1
    #                 ) if ARGS.obs==ObservationType.KIN else DDPG(td3ddpgCnnPolicy,
    #                                                             train_env,
    #                                                             policy_kwargs=offpolicy_kwargs,
    #                                                             tensorboard_log=filename+'/tb/',
    #                                                             verbose=1
    #                                                             )

    #### Create eveluation environment #########################
    # if ARGS.obs == ObservationType.KIN: 
    #     eval_env = gym.make(env_name,
    #                         aggregate_phy_steps=shared_constants.AGGR_PHY_STEPS,
    #                         obs=ARGS.obs,
    #                         act=ARGS.act
    #                         )
    # elif ARGS.obs == ObservationType.RGB:
    #     if env_name == "takeoff-aviary-v0": 
    #         eval_env = make_vec_env(TakeoffAviary,
    #                                 env_kwargs=sa_env_kwargs,
    #                                 n_envs=1,
    #                                 seed=0
    #                                 )
    #     if env_nam == "hover-aviary-v0": 
    #         eval_env = make_vec_env(HoverAviary,
    #                                 env_kwargs=sa_env_kwargs,
    #                                 n_envs=1,
    #                                 seed=0
    #                                 )
    #     if env_name == "flythrugate-aviary-v0": 
    #         eval_env = make_vec_env(FlyThruGateAviary,
    #                                 env_kwargs=sa_env_kwargs,
    #                                 n_envs=1,
    #                                 seed=0
    #                                 )
    #     if env_name == "tune-aviary-v0": 
    #         eval_env = make_vec_env(TuneAviary,
    #                                 env_kwargs=sa_env_kwargs,
    #                                 n_envs=1,
    #                                 seed=0
    #                                 )
    #     eval_env = VecTransposeImage(eval_env)

    #### Train the model #######################################
    # checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=filename+'-logs/', name_prefix='rl_model')
    # callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=EPISODE_REWARD_THRESHOLD,
    #                                                  verbose=1
    #                                                  )
    # eval_callback = EvalCallback(eval_env,
    #                              callback_on_new_best=callback_on_best,
    #                              verbose=1,
    #                              best_model_save_path=filename+'/',
    #                              log_path=filename+'/',
    #                              eval_freq=int(5000/ARGS.cpu),
    #                              deterministic=True,
    #                              render=False
    #                              )
    # model.learn(total_timesteps=int(1e12),
    #             callback=eval_callback,
    #             log_interval=100
    #             )

    ### Save the model #########################################
    # model.save(filename+'/success_model.zip') # Possibly never achieved
    # print(filename)
