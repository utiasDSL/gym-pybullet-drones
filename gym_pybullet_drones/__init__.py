from gym.envs.registration import register

register(
    id='ctrl-aviary-v0',
    entry_point='gym_pybullet_drones.envs:CtrlAviary',
)

register(
    id='dyn-ctrl-aviary-v0',
    entry_point='gym_pybullet_drones.envs:DynCtrlAviary',
)

register(
    id='vision-ctrl-aviary-v0',
    entry_point='gym_pybullet_drones.envs:VisionCtrlAviary',
)




register(
    id='takeoff-aviary-v0',
    entry_point='gym_pybullet_drones.envs.single_agent_rl:TakeoffAviary',
)




register(
    id='flock-aviary-v0',
    entry_point='gym_pybullet_drones.envs.multi_agent_rl:FlockAviary',
)

register(
    id='norm-dyn-ctrl-aviary-v0',
    entry_point='gym_pybullet_drones.envs.multi_agent_rl:NormDynCtrlAviary',
)


