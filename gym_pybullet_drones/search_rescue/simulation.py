import pybullet as p
import pybullet_data
import time
from vehicle import UAV
import random

SIM_TIME = 999999
SIM_FREQ = 240

class Simulation:
    def __init__(self, gui=True, num_uavs = 3, time_step=1./SIM_FREQ):
        self.gui = gui
        self.time_step = time_step
        self.uavs = []
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        self._load_environment()
        self.create_uav(num_uavs)
    
    def _load_environment(self):
        self.plane_id = p.loadURDF("plane.urdf")
    
    def step(self):
        p.stepSimulation()
        if self.gui:
            self.process_input()
            time.sleep(self.time_step)
    
    def reset(self):
        p.resetSimulation()
        self._load_environment()
    
    def create_uav(self, num_uavs = 1, position = [0, 0 , 0.2]):
        for _ in range(num_uavs):
           self.uavs.append(UAV(position)) 

    def apply_force(self, force, position, link_index=-1):
        p.applyExternalForce(self.drone_id, link_index, force, position, p.WORLD_FRAME)
    
    def process_input(self):
        # Get mouse click position (returns x, y screen coordinates)
        mouse_events = p.getMouseEvents()

        for event in mouse_events:
            event_type, mouse_x, mouse_y, button_index, button_state = event

            if event_type == 2 and button_index == 0 and button_state == 3:  # Click detected and Left button
                # Get the mouse position (in screen coordinates)
                print(f"Left click position (screen) -> : ({mouse_x}, {mouse_y})")
                for uav in self.uavs:
                    uav.apply_thrust(2*10)

            if event_type == 2 and button_index == 2 and button_state == 3:  # Click detected and Right button
                # creates a new drone with right click
                print(f"Right click position (screen) -> : ({mouse_x}, {mouse_y})")
                self.create_uav(position=[random.uniform(-1,1), random.uniform(-1,1) , random.uniform(0.1,1)])

    def get_drone_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        return pos, orn
    
    def disconnect(self):
        p.disconnect()
    
if __name__ == "__main__":
    sim = Simulation()
    try:
        for _ in range(SIM_TIME):
            sim.step()
    finally:
        sim.disconnect()