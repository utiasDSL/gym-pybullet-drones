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
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionCtrlAviary import VisionCtrlAviary
from gym_pybullet_drones.envs.MARLFlockAviary import MARLFlockAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger

DRONE = DroneModel.CF2X
PHYSICS = Physics.PYB
SIMULATION_FREQ_HZ = 240
GUI = False
DEBUG_MARL = False
PART = 1

LOG = True
NUM_DRONES = 1
VISION = True
AGGREGATE = True

if __name__ == "__main__":

    ####################################################################################################
    #### Part 1 of 2 of _dev.py: control with CtrlAviary and printout of MARLFlockAviary's RL functions 
    ####################################################################################################
    if PART==1:

        DURATION_SEC = 60
        CONTROL_FREQ_HZ = 48
        AGGR_PHY_STEPS = int(SIMULATION_FREQ_HZ/CONTROL_FREQ_HZ) if AGGREGATE else 1

        #### Initialize the simulation #####################################################################
        H = .1; H_STEP = .05; R = .3; INIT_XYZS = np.array([ [R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(NUM_DRONES) ])
        
        if VISION: 
            env = VisionCtrlAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, visibility_radius=10, \
                        freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=GUI, record=False, obstacles=True)
        else:
            env = CtrlAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, visibility_radius=10, \
                        freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=GUI, record=False, obstacles=True)

        #### Initialize a circular trajectory ##############################################################
        PERIOD = 10; NUM_WP = int(CONTROL_FREQ_HZ)*PERIOD; TARGET_POS = np.zeros((NUM_WP,3))
        for i in range(NUM_WP): TARGET_POS[i,:] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0,0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0,1], INIT_XYZS[0,2]  
        wp_counters = np.array([ int((i*NUM_WP/6)%NUM_WP) for i in range(NUM_DRONES) ])
        
        #### Initialize the logger #########################################################################
        if LOG: logger = Logger(logging_freq_hz=int(SIMULATION_FREQ_HZ/AGGR_PHY_STEPS), num_drones=NUM_DRONES)

        #### Initialize the controllers ####################################################################    
        ctrl = [DSLPIDControl(env) for i in range(NUM_DRONES)]

        #### Debug environment used to print out the MARL's problem obs, reward and done ################### 
        if NUM_DRONES>1 and DEBUG_MARL:
            debug_env = MARLFlockAviary(drone_model=DRONE, num_drones=NUM_DRONES, initial_xyzs=INIT_XYZS, physics=PHYSICS, visibility_radius=10, \
                                    freq=SIMULATION_FREQ_HZ, aggregate_phy_steps=AGGR_PHY_STEPS, gui=False, record=False, obstacles=True)

        #### Run the simulation ############################################################################
        CTRL_EVERY_N_STEPS= int(np.floor(env.SIM_FREQ/CONTROL_FREQ_HZ))
        action = { str(i): np.array([0,0,0,0]) for i in range(NUM_DRONES) }
        START = time.time(); temp_action = {}
        for i in range(0, int(DURATION_SEC*env.SIM_FREQ), AGGR_PHY_STEPS):

            #### Step the simulation ###########################################################################
            obs, reward, done, info = env.step(action)

            #### Debugging MARLFlockAviary's obs, reward and done during a CtrlAviary controlled flight ########
            if DEBUG_MARL: 
                print("CtrlAviary obs", obs)
                marl_obs = {str(i): {"state": debug_env._clipAndNormalizeState(obs[str(i)]["state"]), "neighbors": obs[str(i)]["neighbors"] } for i in range(NUM_DRONES) }
                print("MARLFlockAviary obs", marl_obs)
                print("MARLFlockAviary reward", debug_env._computeReward(marl_obs))
                print("MARLFlockAviary done", debug_env._computeDone(marl_obs))
            
            #### Compute control at the desired frequency @@@@@#################################################       
            if i%CTRL_EVERY_N_STEPS==0:

                #### Compute control for the current way point #####################################################
                for j in range(NUM_DRONES): 
                    action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP, \
                                                                                    state=obs[str(j)]["state"], \
                                                                                    target_pos=np.hstack([TARGET_POS[wp_counters[j],0:2], H+j*H_STEP]))

                #### Go to the next way point and loop #############################################################
                for j in range(NUM_DRONES): wp_counters[j] = wp_counters[j] + 1 if wp_counters[j]<(NUM_WP-1) else 0

            #### Log the simulation ############################################################################
            if LOG: 
                for j in range(NUM_DRONES): logger.log(drone=j, timestamp=i/env.SIM_FREQ, state= obs[str(j)]["state"], control=np.hstack([ TARGET_POS[wp_counters[j],0:2], H+j*H_STEP, np.zeros(9) ]))   
            
            #### Printout ######################################################################################
            if i%int(env.SIM_FREQ)==0: env.render()
            
            #### Sync the simulation ###########################################################################
            if GUI: sync(i, START, env.TIMESTEP)   
        
        #### Close the environment #########################################################################
        env.close()
        
        #### Plot the simulation results ###################################################################
        logger.plot()

    ####################################################################################################
    #### Part 2 of 2 of _devp.y: training and testing MARLFlockAviary as an RLlib MultiAgentEnv ########
    ####################################################################################################
    elif PART==2:

        #### WIP notes #####################################################################################
        #### use ENV_STATE: github.com/ray-project/ray/blob/master/rllib/examples/env/two_step_game.py #####

        #### Initialize Ray Tune ###########################################################################
        ray.shutdown()
        ray.init(ignore_reinit_error=True)
        print("Dashboard URL: http://{}".format(ray.get_webui_url()))

        #### Register the environment ######################################################################
        register_env("marl-flock-aviary-v0", lambda _: MARLFlockAviary(drone_model=DRONE, num_drones=NUM_DRONES, physics=PHYSICS, freq=SIMULATION_FREQ_HZ))

        #### Set up the trainer's config ###################################################################
        config = ppo.DEFAULT_CONFIG.copy()
        config["num_workers"] = 0
        config["env"] = "marl-flock-aviary-v0"
        #### Unused env to extract correctly sized action and observation spaces ###########################
        unused_env = MARLFlockAviary(num_drones=NUM_DRONES)
        config["multiagent"] = { # Map of type MultiAgentPolicyConfigDict from policy ids to tuples of (policy_cls, obs_space, act_space, config).
                                # This defines the observation and action spaces of the policies and any extra config.
                                "policies": {
                                    "pol0": (PPOTFPolicy, unused_env.observation_space["0"], unused_env.action_space["0"], {"framework": "torch"}),
                                    "pol1": (PPOTFPolicy, unused_env.observation_space["1"], unused_env.action_space["1"], {"framework": "torch"}),
                                    "pol2": (PPOTFPolicy, unused_env.observation_space["2"], unused_env.action_space["2"], {"framework": "torch"}),
                                },
                                # Function mapping agent ids to policy ids.
                                "policy_mapping_fn": lambda agent_id: "pol"+str(agent_id),
                                # An additional observation function, see rllib/evaluation/observation_function.py for more info.
                                # "observation_fn": None,
                                }

        #### Ray Tune stopping conditions ##################################################################
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
        env = MARLFlockAviary(drone_model=DRONE, num_drones=NUM_DRONES, physics=PHYSICS, freq=SIMULATION_FREQ_HZ, gui=True, record=False, obstacles=True)
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

        #### Shut down Ray #################################################################################
        ray.shutdown()










