import os
import time
import pdb
import math
import numpy as np
import pybullet as p

from utils import *
from gym_pybullet_drones.envs.DroneModel import DroneModel
from gym_pybullet_drones.envs.SingleDroneEnv import SingleDroneEnv
from gym_pybullet_drones.envs.MultiDroneEnv import MultiDroneEnv

DURATION_SEC = 60
NUM_RESTARTS = 0

########################################################################################################################
# applyExternalTorque()'s WORLD_FRAME and LINK_FRAME appear to be swapped: github.com/bulletphysics/bullet3/issues/1949 
########################################################################################################################

if __name__ == "__main__":
    start = time.time()
    env = SingleDroneEnv(drone_model=DroneModel.CF2X, gui=True, record=False, obstacles=True)
    env.reset()
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_ID = env.getDroneId()

    ####################################################################################################
    #### Make the drone weightless #####################################################################
    ####################################################################################################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)
    
    ####################################################################################################
    #### Set the initial pose ##########################################################################
    ####################################################################################################
    p.resetBasePositionAndOrientation(DRONE_ID, posObj=[-.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,0,0], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    # p.resetBasePositionAndOrientation(DRONE_ID, posObj=[.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,0,60*env.DEG2RAD], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    # p.resetBasePositionAndOrientation(DRONE_ID, posObj=[-.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,60*env.DEG2RAD,0], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)

    for i in range(DURATION_SEC*env.SIM_FREQ):
        
        ####################################################################################################
        #### Apply z-axis force ############################################################################
        ####################################################################################################
        # p.applyExternalForce(DRONE_ID, -1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)     # CORRECT
        # p.applyExternalForce(DRONE_ID, -1, forceObj=[0.,0.,1e-4], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT) # INCORRECT
        
        ####################################################################################################
        #### Apply x-axis torque ###########################################################################
        ####################################################################################################
        p.applyExternalTorque(DRONE_ID, -1, torqueObj=[5e-6,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(DRONE_ID, -1, torqueObj=[5e-6,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT
        
        ####################################################################################################
        #### Apply y-axis torque ###########################################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, -1, torqueObj=[0.,5e-6,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(DRONE_ID, -1, torqueObj=[0.,5e-6,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT
        
        ####################################################################################################
        #### Apply z-axis torque ###########################################################################
        ####################################################################################################
        # p.applyExternalTorque(DRONE_ID, -1, torqueObj=[0.,0.,5e-6], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(DRONE_ID, -1, torqueObj=[0.,0.,5e-6], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT

        ####################################################################################################
        #### Step, printout, and sync ######################################################################
        ####################################################################################################
        p.stepSimulation(physicsClientId=PYB_CLIENT)
        env._showFrame()
        env.render()
        sync(i, start, env.TIMESTEP)

        ####################################################################################################
        #### Reset #########################################################################################
        ####################################################################################################
        if i>1 and i%((DURATION_SEC/(NUM_RESTARTS+1))*env.SIM_FREQ)==0: env.reset()

    env.close()
