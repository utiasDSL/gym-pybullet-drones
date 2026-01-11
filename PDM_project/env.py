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
        # Ensure obstacles_pos is a valid numpy array of shape (N, 3)
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

        # 1. LOAD STATIC MAZE (Base Environment)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "../gym_pybullet_drones/assets/maze.urdf")

        self.maze_id = p.loadURDF(
            urdf_path,               
            [0, 0, 0],               
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,         
            physicsClientId=self.CLIENT
        )

        # 2. LOAD STATIC OBSTACLES (Hardcoded Walls/Pillars)

        # Format: [x, y, z]
        static_cubes_coords = [
            # Obstacle Cluster 1
            [7.0, -1.0, 0.5], [7.0, 0.0, 0.5], 
            #[7.0, 0.0, 1.5],
            [7.0, 1.0, 0.5],  [7.0, 1.0, 1.5], [7.0, 1.0, 2.5],

            # Obstacle Cluster 2
            [2.0, 5.0, 2.5],
            [2.0, 6.0, 0.5], [2.0, 6.0, 1.5], [2.0, 6.0, 2.5],
            [2.0, 7.0, 2.5],

            # Obstacle Cluster 3
            #[2.0, 11.0, 0.5], [2.0, 11.0, 1.5],
            #[2.0, 12.0, 0.5], [2.0, 12.0, 1.5],
            #[2.0, 13.0, 0.5], [2.0, 13.0, 1.5]
            [2.0, 11.0, 0.5],[2.0, 11.0, 1.5],
            [2.0, 12.0, 0.5],
            [2.0, 13.0, 0.5]
        ]

        for pos in static_cubes_coords:
            p.loadURDF(
                "cube_no_rotation.urdf",
                pos,
                p.getQuaternionFromEuler([0, 0, 0]),
                physicsClientId=self.CLIENT
            )

        # 3. LOAD RANDOM OBSTACLES (Passed from RRT/Main)

        for i in range(self.obstacles_pos.shape[0]):
            self._add_single_cube(self.obstacles_pos[i])

        # 4. VISUALIZATION

        # Visualize the safety spheres (useful for MPC debugging)
        obstacles_info = self.get_obstacles_info()
        self._draw_obstacle_spheres(obstacles_info)


    def _add_single_cube(self, pos):
        """Helper to add a single dynamic cube and store its ID."""
        
        script_dir = os.path.dirname(os.path.abspath(__file__))

        urdf_path = os.path.join(script_dir, "../gym_pybullet_drones/assets/big_box.urdf")

        if not os.path.exists(urdf_path):
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
        """
        Returns position and radius of all dynamic obstacles for the MPC.
        This calculates the bounding sphere for collision checking.
        """
        obstacles_info = []

        p.performCollisionDetection(physicsClientId=self.CLIENT)

        for obs_id in self.obstacle_ids:
            pos, _ = p.getBasePositionAndOrientation(obs_id, physicsClientId=self.CLIENT)
            
            # Get Dimensions to calculate bounding sphere
            aabb_min, aabb_max = p.getAABB(obs_id, physicsClientId=self.CLIENT)
            extents = np.array(aabb_max) - np.array(aabb_min)
            
            # Radius is half the diagonal of the bounding box
            r_sphere = 0.5 * np.linalg.norm(extents)

            obstacles_info.append({
                'pos': np.array(pos),
                'r': float(r_sphere)
            })
            
        return obstacles_info        # this is a list of dictionaries with keys 'pos' and 'r'
    

    def _draw_obstacle_spheres(self, obstacles_info):
        """Draws red translucent spheres around obstacles for visual debugging."""
        
        # Clear previous spheres if they exist
        for sid in getattr(self, "obstacle_sphere_ids", []):
            p.removeBody(sid, physicsClientId=self.CLIENT)
        self.obstacle_sphere_ids = []

        for info in obstacles_info:
            vis_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=info['r'],
                rgbaColor=[1, 0, 0, 0.2], # Red, translucent
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
        """Standard step wrapper."""
        obs, reward, terminated, truncated, info = super().step(action)
        return obs, reward, terminated, truncated, info