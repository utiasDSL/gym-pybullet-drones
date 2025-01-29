from cmath import sqrt
import numpy as np
from constants import *

def drawLine(pBullet):
    p1 = [0, 0, 0]
    p2 = [1, 1, 1]
    pBullet.addUserDebugLine(p1, p2, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0)

    p1 = [0, 0, 0]
    p2 = [2, 2, 2]
    pBullet.addUserDebugLine(p1, p2, lineColorRGB=[1, 0, 0], lineWidth=2.0, lifeTime=0)

    p1 = [0, 0, 0]
    p2 = [0, 1, 5]
    pBullet.addUserDebugLine(p1, p2, lineColorRGB=[1, 1, 1], lineWidth=2.0, lifeTime=0)

    p1 = [0.4, 0.4, 0]
    p2 = [1/2, 1/2, 0.1]
    pBullet.addUserDebugLine(p1, p2, lineColorRGB=[1, 0, 0], lineWidth=5.0, lifeTime=0)
    #p.addUserDebugText("o", textPosition = p2, textOrientation = [ 0, 0, 0, 1 ],   textColorRGB=[1, 0, 0], textSize = 1, lifeTime = 0)

def getPosition(obs):
    return np.array(obs[0:3])

def getLinearVelocity(obs):
    return obs[10:13]

def drawVectorVelocity(drone, obs, pBullet):
    pos = getPosition(obs)
    p1 = pos
    vel = getLinearVelocity(obs)

    p2 = p1 + vel*0.5
    pBullet.addUserDebugLine(p1, p2, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0.3)

def drawSquare(pBullet, translation = [0,0,0] , color = [1,0,0], z_offset = 0.1):

    width = 2.0
    n_squares = 4
    off = (1/2)/n_squares

    for i in range(n_squares):
        
        p1 = addPoints([0 + off ,0 + off ,z_offset], translation )
        p2 = addPoints([1 - off ,0 + off ,z_offset], translation )
        pBullet.addUserDebugLine(p1, p2, lineColorRGB=color, lineWidth=width, lifeTime=0)

        p3 = addPoints([1 - off,1 - off,z_offset], translation )
        pBullet.addUserDebugLine(p2, p3, lineColorRGB=color, lineWidth=width, lifeTime=0)

        p4 = addPoints([0 + off ,1 - off,z_offset], translation )
        pBullet.addUserDebugLine(p3, p4, lineColorRGB=color,  lineWidth=width, lifeTime=0)

        pBullet.addUserDebugLine(p4, p1, lineColorRGB=color, lineWidth=width, lifeTime=0)
        off += off

def calculate_initial_pose(num_drones, simulation_freq_hz, control_freq_hz, aggregate):

    INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2) + 1, R*np.sin((i/6)*2*np.pi+np.pi/2)-R + 1, H+i*H_STEP + INITIAL_Z] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])
    AGGR_PHY_STEPS = int(simulation_freq_hz/control_freq_hz) if aggregate else 1

    return INIT_XYZS, INIT_RPYS, AGGR_PHY_STEPS

def addPoints(p1, p2):
    p_out = [0,0,0]
    for p in range(len(p1)):
        p_out[p] = p1[p] + p2[p]

    return p_out

def distancePoint(p1, p2):
    x = p1[0] - p2[0]
    y = p1[1] - p2[1]
    z = p1[2] - p2[2]

    return np.sqrt( x**2 + y**2 + z**2 )
