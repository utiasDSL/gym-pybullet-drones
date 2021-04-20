# Adds a ball/sphere to the environment. For testing when the drone shoots the ball at the goal.

import pybullet as pb
import time
import pybullet_data
physicsClient = pb.connect(pb.GUI)#or pb.DIRECT for non-graphical version
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = pb.getQuaternionFromEuler([0,0,0])
sphere_id = pb.loadURDF("sphere2.urdf",
    [0, 2, .5],
    pb.getQuaternionFromEuler([0,0,0]),
    physicsClientId=physicsClient
)
pb.changeDynamics(sphere_id, -1, mass=0.1)
print("sphere id:", sphere_id)
first_time = True
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    if first_time:
        pb.applyExternalForce(
            sphere_id,
            -1,
            forceObj=[0, 0, 50],
            posObj=[0, 0, 0],
            flags=pb.LINK_FRAME,
            physicsClientId=physicsClient
        )
        print("pushed the ball up?")
        first_time = False
    pb.stepSimulation()
    time.sleep(1./2000.)
# cubePos, cubeOrn = pb.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
pb.disconnect()
