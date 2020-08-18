import os
import time
import pdb
import math
import numpy as np
import pybullet as p

from utils import *
from gym_pybullet_drones.envs.SingleDroneEnv import DroneModel, SingleDroneEnv

DURATION_SEC = 60
NUM_RESTARTS = 0

if __name__ == "__main__":
    start = time.time()
    env = SingleDroneEnv(drone_model=DroneModel.CF2X, initial_xyz=[-.7,-.5,.3], initial_rpy=[0,-30*(np.pi/180),0], gui=True, record=False, obstacles=True)
    env.reset()
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_ID = env.getDroneId()

    ####################################################################################################
    #### Make the drone weightless #####################################################################
    ####################################################################################################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)
    for i in range(DURATION_SEC*env.SIM_FREQ):
        
        ####################################################################################################
        #### Apply x-axis force to the base ################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[1e-4,0.,0,], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        ####################################################################################################
        #### Apply y-axis force to the base ################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[0.,1e-4,0.], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        ####################################################################################################
        #### Apply z-axis force to the base ################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_ID, linkIndex=-1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        ####################################################################################################
        #### To propeller 0 ################################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalForce(DRONE_ID, linkIndex=0, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        ####################################################################################################
        #### To the center of mass #########################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalForce(DRONE_ID, linkIndex=4, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        ####################################################################################################
        #### Apply x-axis torque to the base ###############################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[5e-6,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        ####################################################################################################
        #### Apply y-axis torque to the base ###############################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[0.,5e-6,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        
        ####################################################################################################
        #### Apply z-axis torque to the base ###############################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_ID, linkIndex=-1, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        ####################################################################################################
        #### To propeller 0 ################################################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(DRONE_ID, linkIndex=0, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        ####################################################################################################
        #### To the center of mass #########################################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalTorque(DRONE_ID, linkIndex=4, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)

        ####################################################################################################
        #### Step, printout, and sync ######################################################################
        ####################################################################################################
        p.stepSimulation(physicsClientId=PYB_CLIENT)
        env._showDroneFrame()
        env.render()
        sync(i, start, env.TIMESTEP)

        ####################################################################################################
        #### Reset #########################################################################################
        ####################################################################################################
        if i>1 and i%((DURATION_SEC/(NUM_RESTARTS+1))*env.SIM_FREQ)==0: env.reset()

    env.close()

