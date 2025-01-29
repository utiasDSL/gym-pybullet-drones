
from enum import Enum

# Simulation Parameters
SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080
PIX2M = 0.01  # factor to convert from pixels to meters
M2PIX = 100.0  # factor to convert from meters to pixels
NUM_DRONES = 3# Number of simultaneous drones
SIZE_DRONE = 18
SIZE_TRACK = 40
RESOLUTION = 50 # Of grid
NUM_OBSTACLES = 30
RADIUS_OBSTACLES = 40
TIME_MAX_SIMULATION = 40 # Time to stop simulation in case the conditions are not completed
AVOID_OBSTACLES = RADIUS_OBSTACLES * 1.6
RADIUS_CENTER_CELL = 3.5
OFFSET_ALTITUDE = 5
# Sample Time Parameters
FREQUENCY = 60.0  # simulation frequency
SAMPLE_TIME = 1.0 / FREQUENCY  # simulation sample time
SIM_FREQ = FREQUENCY

# Behavior Parameters
FORWARD_SPEED = 2  # default linear speed when going forward
ANGULAR_SPEED = 1.5# default angular speed
SEEK_FORCE = 0.5 # max seek force
RADIUS_TARGET = 1*M2PIX 
MASS = 10 # Drone Mass, used to calculate force
HOP_AHEAD = 60 # distance of prevision
AVOID_DISTANCE = 30 # distance to avoid collision

# Colors
BLACK = (0,0,0)
LIGHT_BLUE = (224, 255, 255)
BLUE = (0,0,255)
RED = (255,0,0)
GREEN = (0, 255, 0)
YELLOW = (255,215,0)
BLUE_2 = (100,149,237)

class MouseClick(Enum):
    LEFT = int(0)
    RIGHT = int(2)
    MIDDLE = int(1)