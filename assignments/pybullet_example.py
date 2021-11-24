"""Minimal standalone PyBullet example.
"""
import pybullet as p
import pybullet_data
from time import sleep

def main():
    PYB_CLIENT = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8, physicsClientId=PYB_CLIENT)
    p.setRealTimeSimulation(0, physicsClientId=PYB_CLIENT)
    p.setTimeStep(1/240, physicsClientId=PYB_CLIENT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=PYB_CLIENT)
    p.loadURDF("plane.urdf", physicsClientId=PYB_CLIENT)
    p.loadURDF("duck_vhacd.urdf", [0, 0, 1], physicsClientId=PYB_CLIENT)
    for _ in range(240*10):
        p.stepSimulation(physicsClientId=PYB_CLIENT)
        sleep(1/(240*10))

if __name__ == "__main__":
    main()