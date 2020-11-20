"""Script demonstrating PyBullet's forces and torques.

Uncomment the appropriate calls to `p.applyExternalForce()`
and `p.applyExternalTorque()` in `p.WORLD_FRAME` or `p.LINK_FRAME`.

Example
-------
In a terminal, run as:

    $ python physics.py

Notes
-----
This script is for educational purposes and debugging only.

"""
import time
import argparse
import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.BaseAviary import DroneModel
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.utils import sync, str2bool

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Debugging script for PyBullet applyExternalForce() and applyExternalTorque() PyBullet')
    parser.add_argument('--duration_sec',   default=30,     type=int,       help='Duration of the simulation in seconds (default: 30)', metavar='')
    parser.add_argument('--num_resets',     default=1,      type=int,       help='Number of times the simulation is reset to its initial conditions (default: 2)', metavar='')
    ARGS = parser.parse_args()

    #### Initialize the simulation #############################
    env = CtrlAviary(drone_model=DroneModel.CF2X,
                     initial_xyzs=np.array([-.7, -.5, .3]).reshape(1, 3),
                     initial_rpys=np.array([0, -30*(np.pi/180), 0]).reshape(1, 3),
                     gui=True,
                     obstacles=True, 
                     user_debug_gui=False
                     )

    #### Get PyBullet's and drone's ids ########################
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()

    #### Make the drone weightless #############################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)

    #### Draw base frame #######################################
    env._showDroneLocalAxes(0)

    #### Run the simulation ####################################
    START = time.time()
    for i in range(ARGS.duration_sec*env.SIM_FREQ):

        #### Apply x-axis force to the base ########################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply y-axis force to the base ########################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply z-axis force to the base ########################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To propeller 0 ########################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalForce(DRONE_IDS[0], linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To the center of mass #############################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply x-axis torque to the base #######################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply y-axis torque to the base #######################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply z-axis torque to the base #######################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To propeller 0 ########################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To the center of mass #################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalTorque(DRONE_IDS[0], linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Step, sync the simulation, printout the state #########
        p.stepSimulation(physicsClientId=PYB_CLIENT)
        sync(i, START, env.TIMESTEP)
        if i%env.SIM_FREQ == 0:
            env.render()

        #### Reset #################################################
        if i > 1 and i%((ARGS.duration_sec/(ARGS.num_resets+1))*env.SIM_FREQ) == 0:
            env.reset()
            p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)

    #### Close the environment #################################
    env.close()
