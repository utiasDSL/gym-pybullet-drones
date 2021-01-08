"""Script comparing a gym_pybullet_drones simulation to a trace file.

It is meant to compare/validate the behaviour of a pyhsics implementation.

Example
-------
In a terminal, run as:

    $ python compare.py

Notes
-----
The comparison is along a 2D trajectory in the X-Z plane, between x == +1 and -1.

"""
import os
import time
import argparse
import pickle
import numpy as np

from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Trace comparison script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--physics',        default="pyb",               type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',            default=False,               type=str2bool,      help='Whether to use PyBullet GUI (default: False)', metavar='')
    parser.add_argument('--record_video',   default=False,               type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--trace_file',     default="example_trace.pkl", type=str,           help='Pickle file with the trace to compare to (default: "example_trace.pkl")', metavar='')
    ARGS = parser.parse_args()

    #### Load a trace and control reference from a .pkl file ###
    with open(os.path.dirname(os.path.abspath(__file__))+"/../files/"+ARGS.trace_file, 'rb') as in_file:
        TRACE_TIMESTAMPS, TRACE_DATA, TRACE_CTRL_REFERENCE, _, _, _ = pickle.load(in_file)

    #### Compute trace's parameters ############################
    DURATION_SEC = int(TRACE_TIMESTAMPS[-1])
    SIMULATION_FREQ_HZ = int(len(TRACE_TIMESTAMPS)/TRACE_TIMESTAMPS[-1])

    #### Initialize the simulation #############################
    env = CtrlAviary(drone_model=DroneModel.CF2X,
                     num_drones=1,
                     initial_xyzs=np.array([0, 0, .1]).reshape(1, 3),
                     physics=ARGS.physics,
                     freq=SIMULATION_FREQ_HZ,
                     gui=ARGS.gui,
                     record=ARGS.record_video,
                     obstacles=False
                     )
    INITIAL_STATE = env.reset()
    action = {"0": np.zeros(4)}
    pos_err = 9999.

    #### TRACE_FILE starts at [0,0,0], sim starts at [0,0,INITIAL_STATE[2]]
    TRACE_CTRL_REFERENCE[:, 2] = INITIAL_STATE["0"]["state"][2]

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=SIMULATION_FREQ_HZ,
                    num_drones=2,
                    duration_sec=DURATION_SEC
                    )

    #### Initialize the controller #############################
    ctrl = DSLPIDControl(drone_model=env.DRONE_MODEL)

    #### Run the comparison ####################################
    START = time.time()
    for i in range(DURATION_SEC*env.SIM_FREQ):

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        #### Compute next action using the set points from the trace
        action["0"], pos_err, yaw_err = ctrl.computeControlFromState(control_timestep=env.TIMESTEP,
                                                                     state=obs["0"]["state"],
                                                                     target_pos=TRACE_CTRL_REFERENCE[i, 0:3],
                                                                     target_vel=TRACE_CTRL_REFERENCE[i, 3:6]
                                                                     )

        #### Re-arrange the trace for consistency with the logger
        trace_obs = np.hstack([TRACE_DATA[i, 0:3], np.zeros(4), TRACE_DATA[i, 6:9], TRACE_DATA[i, 3:6], TRACE_DATA[i, 9:12], TRACE_DATA[i, 12:16]])

        #### Log the trace #########################################
        logger.log(drone=0,
                   timestamp=TRACE_TIMESTAMPS[i],
                   state=trace_obs,
                   control=np.hstack([TRACE_CTRL_REFERENCE[i, :], np.zeros(6)])
                   )

        #### Log the simulation ####################################
        logger.log(drone=1,
                   timestamp=i/env.SIM_FREQ,
                   state=obs["0"]["state"],
                   control=np.hstack([TRACE_CTRL_REFERENCE[i, :], np.zeros(6)])
                   )

        #### Printout ##############################################
        if i%env.SIM_FREQ == 0: 
            env.render()

        #### Sync the simulation ###################################
        if ARGS.gui: 
            sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()

    #### Plot the simulation results ###########################
    logger.plot(pwm=True)
