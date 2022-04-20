from onpolicy.envs.drone import NavigationAviary, ActionType
import numpy as np
from ray.rllib.agents import ppo

env_config = dict(num_drones=2, record=False, record_path="./record/", aggregate_phy_steps=5, act=ActionType.VEL, episode_len_sec=10, goal_reset=5)
env = NavigationAviary(**env_config)

config = {}
config.update({
    "framework": "torch",
    "multiagent": { 
        "policies": {
            "shared_policy": (None, env.observation_space[0], env.action_space[0], {}),
        },
        "policy_mapping_fn": lambda agent_id, episode, worker, **kwargs: "shared_policy",
    },
})
trainer = ppo.PPOTrainer(config)
trainer.restore("/home/xbt/0424/gym-pybullet-drones/experiments/learning/results/save-navigation-2-ppo-kin-vel-04.06.2022_01.10.41/PPO/PPO_this-aviary-v0_54b6b_00003_3_num_sgd_iter=15,sgd_minibatch_size=1024,train_batch_size=120000_2022-04-06_05-09-57/checkpoint_000083/checkpoint-83")

# obs = env.reset()
# total_reward = 0
# for _ in range(1200):
#     action = trainer.compute_actions(obs, policy_id="shared_policy", explore=True)
#     obs, reward, done, info = env.step(action)
#     total_reward += np.sum(list(reward.values()))
#     if done["__all__"]: break

# print(total_reward, info)