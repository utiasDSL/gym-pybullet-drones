import time
import numpy as np
import pybullet as p
import pybullet_data

DURATION_SEC = 5


if __name__ == "__main__":

    START = time.time()
    PYB_CLIENT = p.connect(p.GUI, key=0)
    p.setRealTimeSimulation(0, physicsClientId=PYB_CLIENT)
    p.setTimeStep(1/240, physicsClientId=PYB_CLIENT)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0,0.0,0.0], physicsClientId=PYB_CLIENT)
    ROBOT_1 = p.loadURDF("../assets/box.urdf",[0,0,0], p.getQuaternionFromEuler([0,0,0]), physicsClientId=PYB_CLIENT)
    ROBOT_2 = p.loadURDF("../assets/box.urdf",[0,0,0], p.getQuaternionFromEuler([0,0,0]), physicsClientId=PYB_CLIENT)
    X_AX_1 = -1; Y_AX_1 = -1; Z_AX_1 = -1;
    X_AX_2 = -1; Y_AX_2 = -1; Z_AX_2 = -1; 
    TITLE = -1; TEXT_1 = -1; TEXT_2 = -1; TEXT_3 = -1
    #### Make objects weightless ###########################################################################################
    p.setGravity(0, 0, 0, physicsClientId=PYB_CLIENT)
    #### Set initial poses ##################################################################################################
    p.resetBasePositionAndOrientation(ROBOT_1,
                                      posObj=[-.2,-.2,.5], 
                                      ornObj=p.getQuaternionFromEuler(
                                                [0*(np.pi/180),0*(np.pi/180),0*(np.pi/180)],
                                                physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    p.resetBasePositionAndOrientation(ROBOT_2,
                                      posObj=[1.,.0,.0],
                                      ornObj=p.getQuaternionFromEuler(
                                                [0*(np.pi/180),0*(np.pi/180),0*(np.pi/180)],
                                                physicsClientId=PYB_CLIENT), physicsClientId=PYB_CLIENT)
    ########################################################################################################################
    # VIDEO = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName="~/Desktop/video.mp4", physicsClientId=PYB_CLIENT)
    for i in range(DURATION_SEC*240):

        X_AX_1 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0.1,0,0],lineColorRGB=[1,0,0],parentObjectUniqueId=ROBOT_1, parentLinkIndex=-1, replaceItemUniqueId=X_AX_1, physicsClientId=PYB_CLIENT)
        Y_AX_1 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0.1,0],lineColorRGB=[0,1,0],parentObjectUniqueId=ROBOT_1, parentLinkIndex=-1, replaceItemUniqueId=Y_AX_1, physicsClientId=PYB_CLIENT)
        Z_AX_1 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,0.1],lineColorRGB=[0,0,1],parentObjectUniqueId=ROBOT_1, parentLinkIndex=-1, replaceItemUniqueId=Z_AX_1, physicsClientId=PYB_CLIENT)
        
        X_AX_2 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0.1,0,0],lineColorRGB=[1,0,0],parentObjectUniqueId=ROBOT_2, parentLinkIndex=-1, replaceItemUniqueId=X_AX_2, physicsClientId=PYB_CLIENT)
        Y_AX_2 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0.1,0],lineColorRGB=[0,1,0],parentObjectUniqueId=ROBOT_2, parentLinkIndex=-1, replaceItemUniqueId=Y_AX_2, physicsClientId=PYB_CLIENT)
        Z_AX_2 = p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,0.1],lineColorRGB=[0,0,1],parentObjectUniqueId=ROBOT_2, parentLinkIndex=-1, replaceItemUniqueId=Z_AX_2, physicsClientId=PYB_CLIENT)

        #### X Thrust ############################################################################################################
        #TITLE = p.addUserDebugText("X FORCE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        #p.applyExternalForce(ROBOT_1, -1, forceObj=[5e-2,0,0], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT) 
        #p.applyExternalForce(ROBOT_2, -1, forceObj=[5e-2,0,0], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 

        #### Y Thrust ############################################################################################################
        #TITLE = p.addUserDebugText("Y FORCE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        #p.applyExternalForce(ROBOT_1, -1, forceObj=[0.,5e-2,0], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        #p.applyExternalForce(ROBOT_2, -1, forceObj=[0.,5e-2,0], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 

        #### Z Thrust ############################################################################################################
        #TITLE = p.addUserDebugText("Z FORCE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        #p.applyExternalForce(ROBOT_1, -1, forceObj=[0.,0.,5e-2], posObj=[0.,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        #p.applyExternalForce(ROBOT_2, -1, forceObj=[0.,0.,5e-2], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 
        
        #### Roll ##############################################################################################################        
        # TITLE = p.addUserDebugText("X TORQUE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(ROBOT_1, -1, torqueObj=[5e-5,0.,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT) 
        # p.applyExternalTorque(ROBOT_2, -1, torqueObj=[5e-5,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 
        
        #### Pitch #############################################################################################################
        #TITLE = p.addUserDebugText("Y TORQUE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(ROBOT_1, -1, torqueObj=[0.,5e-5,0.], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        #p.applyExternalTorque(ROBOT_2, -1, torqueObj=[0.,5e-5,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 
        
        #### Yaw ###############################################################################################################
        #TITLE = p.addUserDebugText("Z TORQUE", textPosition=[0,0,1], textColorRGB=[1,0,0], lifeTime=1, textSize=2, replaceItemUniqueId=TITLE, physicsClientId=PYB_CLIENT)
        # p.applyExternalTorque(ROBOT_1, -1, torqueObj=[0.,0.,5e-5], flags=p.WORLD_FRAME, physicsClientId=PYB_CLIENT)
        #p.applyExternalTorque(ROBOT_2, -1, torqueObj=[0.,0.,5e-5], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        ########################################################################################################################

        # TEXT_1 = p.addUserDebugText("    p.WORLD_FRAME", textPosition=[0,0,0], textColorRGB=[1,0,0], lifeTime=1, textSize=1, parentObjectUniqueId=ROBOT_1, parentLinkIndex=-1, replaceItemUniqueId=TEXT_1, physicsClientId=PYB_CLIENT)
        # TEXT_2 = p.addUserDebugText("    p.LINK_FRAME", textPosition=[0,0,0], textColorRGB=[1,0,0], lifeTime=1, textSize=1, parentObjectUniqueId=ROBOT_2, parentLinkIndex=-1, replaceItemUniqueId=TEXT_2, physicsClientId=PYB_CLIENT)

        #### Get state #########################################################################################################
        p.applyExternalTorque(ROBOT_1, -1, torqueObj=[0.,5e-5,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT)
        p.applyExternalForce(ROBOT_1, -1, forceObj=[5e-2,0,0], posObj=[0.,0.,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) 

        pos, quat = p.getBasePositionAndOrientation(ROBOT_1, physicsClientId=PYB_CLIENT)
        rpj = p.getEulerFromQuaternion(quat, physicsClientId=PYB_CLIENT)
        vel, ang_vel = p.getBaseVelocity(ROBOT_1, physicsClientId=PYB_CLIENT)
    
        # invPos, invOrn = p.invertTransform(pos, quat)

        # pos2, quat2 = p.getBasePositionAndOrientation(ROBOT_2, physicsClientId=PYB_CLIENT)
        # rel_pos, rel_or = p.multiplyTransforms(invPos, invOrn, pos2, quat2)

        TEXT_3 = p.addUserDebugText(f'{vel}\n {ang_vel}', textPosition=[-1.25,0,1], textColorRGB=[0,0,1], lifeTime=1, textSize=1, replaceItemUniqueId=TEXT_3, physicsClientId=PYB_CLIENT)

        ########################################################################################################################

        p.stepSimulation(physicsClientId=PYB_CLIENT)
        elapsed = time.time()-START
        if elapsed < i/240: time.sleep(i/240-elapsed)    

    # p.stopStateLogging(VIDEO, physicsClientId=PYB_CLIENT)



