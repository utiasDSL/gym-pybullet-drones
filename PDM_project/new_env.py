"""Custom CtrlAviary environment with specific obstacle layout."""
import pkg_resources
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

class CustomCtrlAviary(CtrlAviary):
    """CtrlAviary with my own obstacle layout."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Keep track of visual spheres
        self.obstacle_sphere_ids = []
        self.drone_sphere_id = None

    def _addObstacles(self):
        """
        Override BaseAviary._addObstacles().
        This is called automatically from _housekeeping() if self.OBSTACLES == True.
        """
        # Initialize a list to keep track of all obstacle IDs
        self.obstacle_ids = []

        # Define your specific layout here by calling the helper function
        self._add_single_cube([1.0, 0.5, 1.0])
        self._add_single_cube([0.5, 1.0, 1.0])
        self._add_single_cube([3.0, 3.0, 0.5])
        self._add_single_cube([3.0, 3.0, 1.5])

        # After creating obstacles, draw their enclosing spheres
        obstacles_info = self.get_obstacles_info()
        self._draw_obstacle_spheres(obstacles_info)

    def _add_single_cube(self, pos):
        """
        Helper to add a single cube and store its ID.
        """
        obs_id = p.loadURDF(
            "../gym_pybullet_drones/assets/box.urdf",
            pos,
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
            physicsClientId=self.CLIENT
        )
        self.obstacle_ids.append(obs_id)

    def get_obstacles_info(self):
        """
        Returns a list containing the position and bounding-sphere radius of ALL obstacles.
        
        Returns:
            list of dicts: [{'pos': np.array([x,y,z]), 'r': float}, ...]
        """
        obstacles_info = []

        for obs_id in self.obstacle_ids:
            # Get Position
            pos, _ = p.getBasePositionAndOrientation(obs_id, physicsClientId=self.CLIENT)
            
            # Get Dimensions (Visual Shape)
            visual_data = p.getVisualShapeData(obs_id, physicsClientId=self.CLIENT)[0]
            geometry_type = visual_data[2]
            raw_dimensions = np.array(visual_data[3])

            aabb_min, aabb_max = p.getAABB(obs_id, physicsClientId=self.CLIENT)
            extents = np.array(aabb_max) - np.array(aabb_min)   # [Lx, Ly, Lz]

            # bounding sphere from AABB
            r_sphere = 0.5 * np.linalg.norm(extents)
            print(r_sphere)

            
            # PyBullet returns half-extents for boxes, so multiply by 2 for full length/width/height
            # if geometry_type == p.GEOM_BOX:
            # dimensions = raw_dimensions * 2
            # # bounding sphere radius of the box
            # r_sphere = np.sqrt(
            #     (dimensions[0] / 2.0) ** 2
            #     + (dimensions[1] / 2.0) ** 2
            #     + (dimensions[2] / 2.0) ** 2
            # )
            # else:
            #     dimensions = raw_dimensions  # Fallback for non-box shapes
            #     print("SEI UN COGLIONE")
            #     r_sphere = 0.0

            obstacles_info.append({
                'pos': np.array(pos),
                'r': float(r_sphere)
            })
            
        return obstacles_info

    def _draw_obstacle_spheres(self, obstacles_info):
        """Create translucent spheres around each obstacle based on its radius."""
        # Remove old spheres if they exist
        for sid in getattr(self, "obstacle_sphere_ids", []):
            p.removeBody(sid, physicsClientId=self.CLIENT)
        self.obstacle_sphere_ids = []

        for info in obstacles_info:
            if info['r'] <= 0.0:
                continue
            vis_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=info['r'],
                rgbaColor=[1, 0, 0, 0.2],  # red, semi-transparent
                physicsClientId=self.CLIENT
            )
            body_id = p.createMultiBody(
                baseMass=0,  # purely visual
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=vis_id,
                basePosition=info['pos'],
                baseOrientation=[0, 0, 0, 1],
                physicsClientId=self.CLIENT
            )
            self.obstacle_sphere_ids.append(body_id)

    def drone_radius(self):
        """Returns an approximate radius of the drone for obstacle avoidance."""
        radius_drone = 0.06
        h_drone_cylinder = 0.025
        r_drone = np.sqrt((radius_drone) ** 2 + (h_drone_cylinder / 2) ** 2)
        return r_drone

    def _update_drone_sphere(self):
        """Create or move a translucent sphere around the drone."""
        # Assume one drone; if multiple, adapt index
        drone_id = self.DRONE_IDS[0]

        pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.CLIENT)
        r_drone = float(self.drone_radius())

        if self.drone_sphere_id is None:
            # Create the visual sphere for the drone
            vis_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=r_drone,
                rgbaColor=[0, 0, 1, 0.2],  # blue, semi-transparent
                physicsClientId=self.CLIENT
            )
            self.drone_sphere_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=vis_id,
                basePosition=pos,
                baseOrientation=[0, 0, 0, 1],
                physicsClientId=self.CLIENT
            )
        else:
            # Just move it to the new position
            p.resetBasePositionAndOrientation(
                self.drone_sphere_id,
                pos,
                [0, 0, 0, 1],
                physicsClientId=self.CLIENT
            )

    def step(self, action):
        # Gymnasium-style API: 5 return values
        obs, reward, terminated, truncated, info = super().step(action)
        self._update_drone_sphere()   # keep the visual sphere attached to the drone
        return obs, reward, terminated, truncated, info


    # (rest of your comments / methods unchanged...)


# """Custom CtrlAviary environment with specific obstacle layout."""
# import pkg_resources
# import numpy as np
# import pybullet as p
# from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

# class CustomCtrlAviary(CtrlAviary):
#     """CtrlAviary with my own obstacle layout."""

#     def _addObstacles(self):
#         """
#         Override BaseAviary._addObstacles().
#         This is called automatically from _housekeeping() if self.OBSTACLES == True.
#         """
#         # Initialize a list to keep track of all obstacle IDs
#         self.obstacle_ids = []

#         # Define your specific layout here by calling the helper function
#         self._add_single_cube([1.0, 1.0, 0.5])
#         self._add_single_cube([1.0, 1.0, 1.5])
#         self._add_single_cube([3.0, 3.0, 0.5])
#         self._add_single_cube([3.0, 3.0, 1.5])


#     def _add_single_cube(self, pos):
#         """
#         Helper to add a single cube and store its ID.
#         """
#         obs_id = p.loadURDF(
#             "cube_no_rotation.urdf",
#             pos,
#             p.getQuaternionFromEuler([0, 0, 0]),
#             physicsClientId=self.CLIENT
#         )
#         self.obstacle_ids.append(obs_id)
        

#     def get_obstacles_info(self):
#         """
#         Returns a list containing the position and dimensions of ALL obstacles.
        
#         Returns:
#             list of dicts: [{'pos': np.array([x,y,z]), 'dim': np.array([l,w,h])}, ...]
#         """
#         obstacles_info = []

#         for obs_id in self.obstacle_ids:
#             # Get Position
#             pos, _ = p.getBasePositionAndOrientation(obs_id, physicsClientId=self.CLIENT)
            
#             # Get Dimensions (Visual Shape)
#             # getVisualShapeData returns a list; we assume the cube has 1 visual shape (index 0)
#             visual_data = p.getVisualShapeData(obs_id, physicsClientId=self.CLIENT)[0]
#             geometry_type = visual_data[2]
#             raw_dimensions = np.array(visual_data[3])
            
#             # PyBullet returns half-extents for boxes, so multiply by 2 for full length/width/height
#             if geometry_type == p.GEOM_BOX:
#                 dimensions = raw_dimensions * 2
#                 r_sphere = np.sqrt((dimensions[0]/2)**2 + (dimensions[1]/2)**2 + (dimensions[2]/2)**2)  
#             else:
#                 dimensions = raw_dimensions # Fallback for non-box shapes
#                 r_sphere = 0.0

#             obstacles_info.append({
#                 'pos': np.array(pos),
#                 'r': np.array(r_sphere) #serve np.array?
#             })
            
#         return obstacles_info

#     def drone_radius(self):
#         """Returns an approximate radius of the drone for obstacle avoidance."""
#         radius_drone = 0.6
#         h_drone_cylinder = 0.25
#         r_drone = np.sqrt((radius_drone)**2 + (h_drone_cylinder/2)**2)
#         return r_drone
#         '''
#         p.loadURDF(
#             "cube_no_rotation.urdf",
#             [1.0, 4.0, 0.0],
#             p.getQuaternionFromEuler([0,0,0]),
#             physicsClientId=self.CLIENT
#         )
        
#         p.loadURDF(
#             "duck_vhacd.urdf",
#             [-.5, -.5, .05],
#             p.getQuaternionFromEuler([0, 0, 0]),
#             physicsClientId=self.CLIENT
#         )

#         #Code taken from https://github.com/phuongboi/drone-racing-using-reinforcement-learning
#         g1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
#                    [0, 1, 0],
#                    p.getQuaternionFromEuler([0, 0, np.pi/2]),
#                    physicsClientId=self.CLIENT
#                    )
#         g2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
#                    [0.5, 3, 0],
#                    p.getQuaternionFromEuler([0, 0, np.pi/2]),
#                    physicsClientId=self.CLIENT
#                    )
#         g3 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
#                    [-0.5, 5, 0],
#                    p.getQuaternionFromEuler([0, 0, np.pi/2]),
#                    physicsClientId=self.CLIENT
#                    )
#         g4 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
#                    [-0.5, 6, 0],
#                    p.getQuaternionFromEuler([0, 0, np.pi/2]),
#                    physicsClientId=self.CLIENT
#                    )

#         self.GATE_IDs = np.array([g1, g2, g3, g4])
#         '''
        
#     '''
#     def _preprocessAction(self,
#                           action
#                           ):
#         """Pre-processes the action passed to .step() into motors' RPMs.

#         It can be the same as in CtrlAviary or customized depending on the action format (the output of the MPC).

#         Parameters
#         ----------
#         action : ndarray | dict[..]
#             The input action for one or more drones, to be translated into RPMs.

#         """
#         #raise NotImplementedError
#     '''
#     '''
#     Some comments:
#     this class is an inheritance of CtrlAviary which is by itslef an inheritance of BaseAviary. It overrides the _addObstacles() method to define a custom layout of obstacles in the simulation environment.
#     The other functions such as get_action_space(), get_observation_space() are inherited from the parent classes and do not need to be redefined here unless specific custom behavior is desired.
#     However it can be noted that get_observation_space() is returning every observation (pos, vel, rot, ang vel) which for our linearized model is not necessary. So when using MPC we need to pick only the relevant states.
#     We can still use the .step() fucntion and we pass the action (motor commands) computed by the MPC to it. The step function will take care of applying the action and returning the full observation, from which we can extract the relevant states for our model.
#     In the step function a pre_process of the actions is done. Preporcess here means converting the output of the MPC to the format expected by the simulator (e.g. scaling motor commands). This is expressed in the method  named _physics that expects rpm as input.
#     The output of the MPC can be choosen depending on the model and on what we are optimizing. For example we can output desired roll, pitch, yaw rate and thrust, and then convert these to motor commands before passing them to the step function. (we also clip the motor commands to be within the valid range).
#     In the .step function the next state is computed using _physics method that applies the motor commands and simulates the drone dynamics for one time step + p.stepsimulation where the actual physics step is done.
#     This last p.stepsimulation integrates the equations of motion with the current forces/torques and updates the state of the drone in the simulator.
#     We can also use the dynamic method which use the dynamic model and compute everyintg in the same method (maybe more similar to dynamic model). We decide what to use when we make the instance of the class in main.py by choosing the physics parameter (either DYN or PYB).
#     There are methods to store the state such as _updateAndStoreKinematicInformation() and _getDroneStateVector() that can be used to retrieve the current state of the drone after the step.
#     Then to continue the loop of the MPC we extract the observations after updating them using computeObs() method.
#     '''