import os
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

class CustomCtrlAviary(CtrlAviary):
    """
    Unified Environment Class.
    Combines obstacle layout from env_RRT.py with the robust loading 
    and visualization from new_env.py.
    """

    def __init__(self, obstacles_pos, *args, **kwargs):
        if obstacles_pos is None:
            self.obstacles_pos = np.zeros((0, 3), dtype=float)
        else:
            self.obstacles_pos = np.asarray(obstacles_pos, dtype=float)
            self.obstacles_pos = np.atleast_2d(self.obstacles_pos)
            if self.obstacles_pos.shape[1] != 3:
                raise ValueError(f"obstacles_pos must have shape (N,3). Got {self.obstacles_pos.shape}.")

        super().__init__(*args, **kwargs)
        # Keep track of visual spheres for debugging
        self.obstacle_sphere_ids = []
        self.drone_sphere_id = None
        

    def _addObstacles(self):
        """
        Override BaseAviary._addObstacles().
        Adds the maze obstacles and visualizes their bounding spheres.
        """
        self.obstacle_ids = []

        # Load the entire maze at once
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "../gym_pybullet_drones/assets/maze.urdf")

        self.maze_id = p.loadURDF(
            urdf_path,               # Path to the file you just created
            [0, 0, 0],                 # Base position (0,0,0)
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,         # Important: keeps the maze static
            physicsClientId=self.CLIENT
        )
        # self.obstacle_ids.append(self.maze_id)

        # Define the maze layout (Your 4 pillars)
        # We use the list of positions you used in RRT

        for i in range(self.obstacles_pos.shape[0]):
            self._add_single_cube(self.obstacles_pos[i])

        # Visualize the safety spheres (useful for MPC debugging)
        obstacles_info = self.get_obstacles_info()
        self._draw_obstacle_spheres(obstacles_info)


    def _add_single_cube(self, pos):
        # Helper to add a single cube and store its ID.
        #self.obstacle_pos = pos
        # Robust path finding using os (fixes the 'file not found' error)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Go up two levels to find gym_pybullet_drones if necessary, or adjust relative to project root
        urdf_path = os.path.join(script_dir, "../gym_pybullet_drones/assets/big_box.urdf")
        
        # Fallback: if running from root, path might differ. Check existence.
        if not os.path.exists(urdf_path):
             # Try simpler path if running from root
             urdf_path = "./gym_pybullet_drones/assets/big_box.urdf"

        obs_id = p.loadURDF(
            urdf_path,
            pos,
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
            physicsClientId=self.CLIENT
        )
        self.obstacle_ids.append(obs_id)

        
    def get_obstacles_info(self):
        """Returns position and radius of all obstacles for the MPC."""
        obstacles_info = []

        p.performCollisionDetection(physicsClientId=self.CLIENT)

        for obs_id in self.obstacle_ids:
            pos, _ = p.getBasePositionAndOrientation(obs_id, physicsClientId=self.CLIENT)
            
            # Get Dimensions to calculate bounding sphere
            # visual_data = p.getVisualShapeData(obs_id, physicsClientId=self.CLIENT)[0]
            aabb_min, aabb_max = p.getAABB(obs_id, physicsClientId=self.CLIENT)
            extents = np.array(aabb_max) - np.array(aabb_min)
            # Radius is half the diagonal of the bounding box
            r_sphere = 0.5 * np.linalg.norm(extents)

            obstacles_info.append({
                'pos': np.array(pos),
                'r': float(r_sphere)
            })
            
        return obstacles_info
    

    def _draw_obstacle_spheres(self, obstacles_info):
        """Draws red translucent spheres around obstacles."""
        for sid in getattr(self, "obstacle_sphere_ids", []):
            p.removeBody(sid, physicsClientId=self.CLIENT)
        self.obstacle_sphere_ids = []

        for info in obstacles_info:
            vis_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=info['r'],
                rgbaColor=[1, 0, 0, 0.2],
                physicsClientId=self.CLIENT
            )
            body_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=vis_id,
                basePosition=info['pos'],
                physicsClientId=self.CLIENT
            )
            self.obstacle_sphere_ids.append(body_id)


    def drone_radius(self):
        """Returns approximate radius of the drone (CF2X)."""
        radius_drone = 0.06
        h_drone_cylinder = 0.025
        return np.sqrt((radius_drone) ** 2 + (h_drone_cylinder / 2) ** 2)
    

    def step(self, action):
        """Standard step, but updates visual markers."""
        obs, reward, terminated, truncated, info = super().step(action)
        return obs, reward, terminated, truncated, info