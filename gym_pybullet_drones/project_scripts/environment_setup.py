import pybullet as p  # Import pybullet here

class Environment:
    def __init__(self, client):
        self.client = client 

    def add_obstacle(self, type, coordinates=[0, 0, 0], orientations=None, verbose=False, return_data=False):
        if orientations is None:
            orientations = p.getQuaternionFromEuler([0, 0, 0])  

        # Load different obstacle types
        if type == "beam":
            obs_id = p.loadURDF("../assets/beam.urdf", coordinates, orientations, physicsClientId=self.client)
        elif type == "beam_horizontal":
            obs_id = p.loadURDF("../assets/beam_horizontal.urdf", coordinates, orientations, physicsClientId=self.client)
        elif type == "cube":
            obs_id = p.loadURDF("../assets/cube.urdf", coordinates, orientations, physicsClientId=self.client)
        elif type == "cylinder":
            obs_id = p.loadURDF("../assets/cylinder.urdf", coordinates, orientations, physicsClientId=self.client)
        elif type == "sphere":
            obs_id = p.loadURDF("../assets/sphere.urdf", coordinates, orientations, physicsClientId=self.client)
        else:
            raise ValueError(f"Unknown obstacle type: {type}")

        # Get obstacle data
        position, orientation = p.getBasePositionAndOrientation(obs_id, physicsClientId=self.client)
        aabb_min, aabb_max = p.getAABB(obs_id, physicsClientId=self.client)

        obstacle_data = {
            "position": position,  # Obstacle's center position
            "aabb_min": aabb_min,  # Bounding box minimum corner
            "aabb_max": aabb_max,  # Bounding box maximum corner
        }

        if verbose:
            for key, value in obstacle_data.items():
                print(f"{key} = {value}")

        if return_data:
            return obstacle_data
