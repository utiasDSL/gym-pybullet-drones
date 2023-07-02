from gymnasium.envs.registration import register

register(
    id='ctrl-aviary-v0',
    entry_point='gym_pybullet_drones.envs:CtrlAviary',
)

register(
    id='velocity-aviary-v0',
    entry_point='gym_pybullet_drones.envs:VelocityAviary',
)

register(
    id='hover-aviary-v0',
    entry_point='gym_pybullet_drones.envs.single_agent_rl:HoverAviary',
)

register(
    id='flythrugate-aviary-v0',
    entry_point='gym_pybullet_drones.envs.single_agent_rl:FlyThruGateAviary',
)

register(
    id='flock-aviary-v0',
    entry_point='gym_pybullet_drones.envs.multi_agent_rl:FlockAviary',
)

register(
    id='leaderfollower-aviary-v0',
    entry_point='gym_pybullet_drones.envs.multi_agent_rl:LeaderFollowerAviary',
)
