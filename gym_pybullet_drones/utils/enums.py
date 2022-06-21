from enum import Enum

class DroneModel(Enum):
    """Drone models enumeration class."""

    CF2X = "cf2x"   # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = "cf2p"   # Bitcraze Craziflie 2.0 in the + configuration
    HB = "hb"       # Generic quadrotor (with AscTec Hummingbird inertial properties)

################################################################################

class Physics(Enum):
    """Physics implementations enumeration class."""

    PYB = "pyb"                         # Base PyBullet physics update
    DYN = "dyn"                         # Update with an explicit model of the dynamics
    PYB_GND = "pyb_gnd"                 # PyBullet physics update with ground effect
    PYB_DRAG = "pyb_drag"               # PyBullet physics update with drag
    PYB_DW = "pyb_dw"                   # PyBullet physics update with downwash
    PYB_GND_DRAG_DW = "pyb_gnd_drag_dw" # PyBullet physics update with ground effect, drag, and downwash

################################################################################

class ImageType(Enum):
    """Camera capture image type enumeration class."""

    RGB = 0     # Red, green, blue (and alpha)
    DEP = 1     # Depth
    SEG = 2     # Segmentation by object id
    BW = 3      # Black and white

################################################################################