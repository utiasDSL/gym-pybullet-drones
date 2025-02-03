import pybullet as p

class UAV:
    def __init__(self, position=[0, 0, 1]):
        self.uav_id = p.loadURDF("cf2x.urdf", position, useFixedBase=False)
        print(f"UAV with {self.uav_id} ID created")

    def apply_thrust(self, thrust, link_index=-1):
        p.applyExternalForce(self.uav_id, link_index, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
    
    def get_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.uav_id)
        return pos, orn