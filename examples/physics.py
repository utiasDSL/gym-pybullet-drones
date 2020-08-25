import os
import time
import pdb
import math
import numpy as np
import pybullet as p

from utils import *
from gym_pybullet_drones.envs.Aviary import DroneModel, Aviary

DURATION_SEC = 60
NUM_RESETS = 0

if __name__ == "__main__":
    
    #### Initialize the simulation #####################################################################
    env = Aviary(drone_model=DroneModel.CF2X, initial_xyzs=np.array([-.7, -.5, .3]).reshape(1,3), \
                    initial_rpys=np.array([0, -30*(np.pi/180), 0]).reshape(1,3), gui=True, obstacles=True); env.reset()

    #### Get PyBullet's and drone's ids ################################################################
    PYB_CLIENT = env.getPyBulletClient(); DRONE_IDS = env.getDroneIds()

    #### Make the drone weightless #####################################################################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)
    
    #### Run the simulation ############################################################################
    START = time.time()
    for i in range(DURATION_SEC*env.SIM_FREQ):
        
        #### Apply x-axis force to the base ################################################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply y-axis force to the base ################################################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Apply z-axis force to the base ################################################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To propeller 0 ################################################################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalForce(DRONE_IDS[0], linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To the center of mass #########################################################################
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_IDS[0], linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        #### Apply x-axis torque to the base ###############################################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        #### Apply y-axis torque to the base ###############################################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        #### Apply z-axis torque to the base ###############################################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To propeller 0 ################################################################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        #### To the center of mass #########################################################################
        # p.applyExternalTorque(DRONE_IDS[0], linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalTorque(DRONE_IDS[0], linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        #### Draw base frame ###############################################################################
        env._showDroneFrame(0)

        #### Step, printout the state, and sync the simulation #############################################
        p.stepSimulation(physicsClientId=PYB_CLIENT); env.render(); sync(i, START, env.TIMESTEP)

        #### Reset #########################################################################################
        if i>1 and i%((DURATION_SEC/(NUM_RESETS+1))*env.SIM_FREQ)==0: env.reset()

    #### Close the environment #########################################################################
    env.close()

