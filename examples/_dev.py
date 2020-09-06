import os
import time
from datetime import datetime
import pdb
import math
import numpy as np
import pybullet as p
import pickle
import matplotlib.pyplot as plt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import ray
from ray import tune
from ray.tune import register_env
from ray.rllib.agents import ppo
from ray.rllib.agents.ppo import PPOTrainer, PPOTFPolicy
from ray.rllib.examples.policy.random_policy import RandomPolicy
from ray.rllib.utils.test_utils import check_learning_achieved

from utils import *
from gym_pybullet_drones.envs.Aviary import DroneModel, Physics, Aviary, RLlibMAAviary
from gym_pybullet_drones.envs.Logger import Logger
from gym_pybullet_drones.envs.Control import ControlType, Control
from gym_pybullet_drones.envs.RLFunctions import Problem, RLFunctions 

DRONE = DroneModel.CF2X
NUM_DRONES = 3
PHYSICS = Physics.PYB
SIMULATION_FREQ_HZ = 240

if __name__ == "__main__":

    ####################################################################################################
    #### Flight with control example ###################################################################
    ####################################################################################################
    if True:

        AGGR_PHY_STEPS = 3
        CONTROL_FREQ_HZ = 48*AGGR_PHY_STEPS
        DURATION_SEC = 10
        

        #### Initialize the simulation #####################################################################
        H = .1; H_STEP = .05; R = .3; INIT_XYZS = np.array([ [R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(NUM_DRONES) ])
        env = RLlibMAAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, visibility_radius=10, \
                        freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=False, record=False, obstacles=True)

        #### Initialize a circular trajectory ##############################################################
        PERIOD = 10; NUM_WP = CONTROL_FREQ_HZ*PERIOD; TARGET_POS = np.zeros((NUM_WP,3))
        for i in range(NUM_WP): TARGET_POS[i,:] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0,0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0,1], INIT_XYZS[0,2]  
        wp_counters = np.array([ int((i*NUM_WP/6)%NUM_WP) for i in range(NUM_DRONES) ])
        
        #### Initialize the logger #########################################################################
        logger = Logger(simulation_freq_hz=SIMULATION_FREQ_HZ, num_drones=NUM_DRONES)

        #### Initialize the controllers ####################################################################    
        ctrl = [Control(env, control_type=ControlType.PID) for i in range(NUM_DRONES)]

        #### Initialize RL functions ####################################################################### 
        #RL_FUNCTIONS = RLFunctions(env.getPyBulletClient(), num_drones=NUM_DRONES, gui=True, problem=Problem.MA_FLOCK)

        #### Run the simulation ############################################################################
        CTRL_EVERY_N_STEPS= int(np.floor(env.SIM_FREQ/CONTROL_FREQ_HZ))
        action = { str(i): np.array([0,0,0,0]) for i in range(NUM_DRONES) } if NUM_DRONES>1 else np.array([0,0,0,0])
        START = time.time(); temp_action = {}
        for i in range(int(DURATION_SEC*env.SIM_FREQ/AGGR_PHY_STEPS)):

            #### Step the simulation ###########################################################################
            obs, reward, done, info = env.step(action)

    ##############################
    ##############################
    ##############################
            # print("Obs", obs)
            # print("Norm Obs", {str(j): RL_FUNCTIONS.clipAndNormalizeState(obs[str(j)]["state"], env.step_counter) for j in range(NUM_DRONES)})
            # print("Reward", {str(j): RL_FUNCTIONS.rewardFunction(obs) for j in range(NUM_DRONES) })
            # print("Done", RL_FUNCTIONS.doneFunction(obs, env.step_counter/env.SIM_FREQ))
            # print("Info", {str(j): {} for j in range(NUM_DRONES) })
            # print()
    ##############################
    ##############################
    ##############################

            #### Transform 1-drone obs into the Dict format of multiple drones to simplify the code ############
            if NUM_DRONES==1: obs = {"0": {"state": obs}}

            #### Compute control at the desired frequency @@@@@#################################################       
            if i%CTRL_EVERY_N_STEPS==0:

                #### Compute control for the current waypoint ######################################################
                for j in range(NUM_DRONES): 
                    temp_action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP, \
                                                                                    state=obs[str(j)]["state"], \
                                                                                    target_pos=np.hstack([TARGET_POS[wp_counters[j],0:2], H+j*H_STEP]))
                    
                    #### Transform multi-drone actions into the Box format of a single drone to simplify the code ######
                    action = temp_action if NUM_DRONES>1 else temp_action["0"]

                #### Go to the next waypoint and loop ##############################################################
                for j in range(NUM_DRONES): wp_counters[j] = wp_counters[j] + 1 if wp_counters[j]<(NUM_WP-1) else 0

            #### Log the simulation ############################################################################
            for j in range(NUM_DRONES): logger.log(drone=j, timestamp=i/env.SIM_FREQ, state= obs[str(j)]["state"], control=np.hstack([ TARGET_POS[wp_counters[j],0:2], H+j*H_STEP, np.zeros(9) ]))   
            
            #### Printout ######################################################################################
            if i%int(env.SIM_FREQ/5)==0: env.render()
            
            #### Sync the simulation ###########################################################################
            # sync(i, START, env.TIMESTEP)   
        
        #### Close the environment #########################################################################
        env.close()
        
        #### Plot the simulation results ###################################################################
        logger.plot()

    ####################################################################################################
    #### Learning example ##############################################################################
    ####################################################################################################
    else:

        #### WIP ###########################################################################################
        #### (partially) based on: https://github.com/ray-project/ray/issues/9123
        #### use ENV_STATE? https://github.com/ray-project/ray/blob/master/rllib/examples/env/two_step_game.py

        ray.shutdown()
        ray.init(ignore_reinit_error=True)
        print("Dashboard URL: http://{}".format(ray.get_webui_url()))

        
        #### Set up the trainer's config ###################################################################
        register_env("ma-aviary", lambda _: RLlibMAAviary(drone_model=DRONE, num_drones=NUM_DRONES, physics=PHYSICS, \
                                                            freq=SIMULATION_FREQ_HZ, problem=Problem.MA_FLOCK))

        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 0
        config["env"] = "ma-aviary"
        
        env = RLlibMAAviary(num_drones=NUM_DRONES, problem=Problem.MA_FLOCK)
        config["multiagent"] = { # Map of type MultiAgentPolicyConfigDict from policy ids to tuples of (policy_cls, obs_space, act_space, config).
                                # This defines the observation and action spaces of the policies and any extra config.
                                "policies": {
                                    "pol0": (PPOTFPolicy, env.observation_space["0"], env.action_space["0"], {"framework": "torch"}),
                                    "pol1": (PPOTFPolicy, env.observation_space["1"], env.action_space["1"], {"framework": "torch"}),
                                    "pol2": (PPOTFPolicy, env.observation_space["2"], env.action_space["2"], {"framework": "torch"}),
                                },
                                # Function mapping agent ids to policy ids.
                                "policy_mapping_fn": lambda agent_id: "pol"+str(agent_id),
                                # An additional observation function, see rllib/evaluation/observation_function.py for more info.
                                # "observation_fn": None,
                                }

        stop = {
            "timesteps_total": 8000,
        }

        #### Train #########################################################################################
        results = tune.run(
            "PPO", 
            stop=stop, 
            config=config, 
            verbose=True,
            checkpoint_at_end=True)
        # check_learning_achieved(results, 1.0)
        
        #### Restore agent #################################################################################
        checkpoints = results.get_trial_checkpoints_paths(trial=results.get_best_trial('episode_reward_mean'), metric='episode_reward_mean')
        checkpoint_path = checkpoints[0][0]; agent = ppo.PPOTrainer(config=config); agent.restore(checkpoint_path)

        #### Extract and print policies ####################################################################
        policy0 = agent.get_policy("pol0")
        policy1 = agent.get_policy("pol1")
        policy2 = agent.get_policy("pol2")
        print(policy0.model.base_model.summary())
        print(policy1.model.base_model.summary())
        print(policy2.model.base_model.summary())

        #### Create test environment ########################################################################
        env = RLlibMAAviary(drone_model=DRONE, num_drones=NUM_DRONES, physics=PHYSICS, freq=SIMULATION_FREQ_HZ, \
                                gui=True, record=False, problem=Problem.MA_FLOCK, obstacles=True)
        obs = env.reset()
        action = { str(i): np.array([0,0,0,0]) for i in range(NUM_DRONES) } 
        start = time.time()
        for i in range(10*env.SIM_FREQ):

            #### Deploy the policies ###########################################################################
            print("Debug Obs", obs)
            temp = {}
            temp["0"] = policy0.compute_single_action(np.hstack([ obs["0"]["state"], obs["0"]["neighbors"] ]))
            temp["1"] = policy1.compute_single_action(np.hstack([ obs["1"]["state"], obs["1"]["neighbors"] ]))
            temp["2"] = policy2.compute_single_action(np.hstack([ obs["2"]["state"], obs["2"]["neighbors"] ]))
            print("Debug Act", temp)
            action = {"0": temp["0"][0], "1": temp["1"][0], "2": temp["2"][0]}
            
            obs, reward, done, info = env.step(action)
            env.render()
            sync(i, start, env.TIMESTEP)
            if done["__all__"]: obs = env.reset()
        env.close()

        ray.shutdown()










