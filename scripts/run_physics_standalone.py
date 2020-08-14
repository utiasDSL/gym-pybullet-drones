import time
import numpy as np
import pybullet as p
import pybullet_data

DURATION_SEC = 10
NUM_RESTARTS = 0

########################################################################################################################
# applyExternalTorque()'s WORLD_FRAME and LINK_FRAME appear to be swapped: github.com/bulletphysics/bullet3/issues/1949 
########################################################################################################################

if __name__ == "__main__":
    START = time.time()
    X_AX = -1; Y_AX = -1; Z_AX = -1
    PYB_CLIENT = p.connect(p.GUI, key=0)
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0,0.0,0.0], physicsClientId=PYB_CLIENT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=PYB_CLIENT)
    ROBOT_ID = p.loadURDF("duck_vhacd.urdf",[0,0,0.1], p.getQuaternionFromEuler([0,0,0]), physicsClientId=PYB_CLIENT)
    
    ####################################################################################################
    #### Make the drone weightless #####################################################################
    ####################################################################################################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)
    
    ####################################################################################################
    #### Set the initial pose ##########################################################################
    ####################################################################################################
    # p.resetBasePositionAndOrientation(ROBOT_ID, posObj=[-.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,0,0], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    # p.resetBasePositionAndOrientation(ROBOT_ID, posObj=[-.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,0,60*(np.pi/180)], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    p.resetBasePositionAndOrientation(ROBOT_ID, posObj=[-.7,-.5,.3], ornObj=p.getQuaternionFromEuler([0,60*(np.pi/180),0], physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    
    for i in range(DURATION_SEC*240):

        ####################################################################################################
        #### Draw the local frame ##########################################################################
        ####################################################################################################
        X_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0.1,0,0],lineColorRGB=[1,0,0],parentObjectUniqueId=ROBOT_ID, parentLinkIndex=-1, replaceItemUniqueId=X_AX, physicsClientId=PYB_CLIENT)
        Y_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0.1,0],lineColorRGB=[0,1,0],parentObjectUniqueId=ROBOT_ID, parentLinkIndex=-1, replaceItemUniqueId=Y_AX, physicsClientId=PYB_CLIENT)
        Z_AX = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,0.1],lineColorRGB=[0,0,1],parentObjectUniqueId=ROBOT_ID, parentLinkIndex=-1, replaceItemUniqueId=Z_AX, physicsClientId=PYB_CLIENT)
        
        ####################################################################################################
        #### Apply z-axis force ############################################################################
        ####################################################################################################
        # p.applyExternalForce(ROBOT_ID, -1, forceObj=[0.,0.,1e-2], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)     # CORRECT
        # p.applyExternalForce(ROBOT_ID, -1, forceObj=[0.,0.,1e-2], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT) # INCORRECT
        
        ####################################################################################################
        #### Apply x-axis torque ###########################################################################
        ####################################################################################################
        p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[5e-4,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[5e-4,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT
        
        ####################################################################################################
        #### Apply y-axis torque ###########################################################################
        ####################################################################################################
        # p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[0.,5e-4,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[0.,5e-4,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT
        
        ####################################################################################################
        #### Apply z-axis torque ###########################################################################
        ####################################################################################################
        # p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[0.,0.,5e-4], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)             # CORRECT
        # p.applyExternalTorque(ROBOT_ID, -1, torqueObj=[0.,0.,5e-4], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)             # INCORRECT
        
        ####################################################################################################
        #### Step, printout, and sync ######################################################################
        ####################################################################################################
        p.stepSimulation(physicsClientId=PYB_CLIENT)
        pos, quat = p.getBasePositionAndOrientation(ROBOT_ID, physicsClientId=PYB_CLIENT)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(ROBOT_ID, physicsClientId=PYB_CLIENT)
        print("——— step number {:d}".format(i), 
            "——— X {:.2f} Y {:.2f} Z {:.2f} vel. {:.2f}".format(pos[0],pos[1],pos[2], np.linalg.norm(vel)), 
            "——— roll {:.2f} pitch {:.2f} yaw {:.2f}".format(rpy[0]*(180/np.pi), rpy[1]*(180/np.pi), rpy[2]*(180/np.pi)),
            "——— ang. vel. {:.2f} {:.2f} {:.2f} ——— ".format(ang_v[0]*(180/np.pi), ang_v[1]*(180/np.pi), ang_v[2]*(180/np.pi)), end="\r")
        elapsed = time.time()-START
        if elapsed < i/240: time.sleep(i/240-elapsed)

        ####################################################################################################
        #### Reset #########################################################################################
        ####################################################################################################            
        if i>1 and i%((DURATION_SEC/(NUM_RESTARTS+1))*240)==0: p.resetSimulation(physicsClientId=PYB_CLIENT)

    p.disconnect(physicsClientId=PYB_CLIENT)
    
