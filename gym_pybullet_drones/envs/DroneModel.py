from enum import Enum

####################################################################################################
#### Drone models ##################################################################################
####################################################################################################
class DroneModel(Enum):
    HB = 0                   # Generic quadrotor (with AscTec Hummingbird intertial properties)
    CF2X = 1                 # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = 2                 # Bitcraze Craziflie 2.0 in the + configuration
