import os
import numpy as np
from gym import spaces
import pkg_resources
import pybullet as p
import open3d as o3d
from gym_pybullet_drones.envs.BaseAviary import BaseAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType
from scipy.spatial.transform import Rotation as R
import cv2

class VisionAviary(BaseAviary):
    """Multi-drone environment class for control applications using vision."""

    ################################################################################
    
    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 freq: int=240,
                 aggregate_phy_steps: int=1,
                 gui=False,
                 record=False,
                 obstacles=False,
                 user_debug_gui=True,
                 output_folder='results'
                 ):
        """Initialization of an aviary environment for control applications using vision.

        Attribute `vision_attributes` is automatically set to True when calling
        the superclass `__init__()` method.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        freq : int, optional
            The frequency (Hz) at which the physics engine steps.
        aggregate_phy_steps : int, optional
            The number of physics steps within one call to `BaseAviary.step()`.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation in folder `files/videos/`.
        obstacles : bool, optional
            Whether to add obstacles to the simulation.
        user_debug_gui : bool, optional
            Whether to draw the drones' axes and the GUI RPMs sliders.

        """
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         record=record,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui,
                         vision_attributes=True,
                         output_folder=output_folder
                         )
        self.VID_WIDTH=int(640)
        self.VID_HEIGHT=int(480)
        
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=90.0,
                                                            aspect=self.VID_WIDTH/self.VID_HEIGHT,
                                                            nearVal=0.1,
                                                            farVal=1000.0
                                                            )
        self.near = 0.1
        self.far = 1000

    
    ################################################################################
    
    def _actionSpace(self):
        """Returns the action space of the environment.

        Returns
        -------
        dict[str, ndarray]
            A Dict of Box(4,) with NUM_DRONES entries,
            indexed by drone Id in string format.

        """
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([0.,           0.,           0.,           0.])
        act_upper_bound = np.array([self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({str(i): spaces.Box(low=act_lower_bound,
                                               high=act_upper_bound,
                                               dtype=np.float32
                                               ) for i in range(self.NUM_DRONES)})
    
    ################################################################################
    
    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------
        dict[str, dict[str, ndarray]]
            A Dict with NUM_DRONES entries indexed by Id in string format,
            each a Dict in the form {Box(20,), MultiBinary(NUM_DRONES), Box(H,W,4), Box(H,W), Box(H,W)}.

        """
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
        obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
        obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        return spaces.Dict({str(i): spaces.Dict({"state": spaces.Box(low=obs_lower_bound,
                                                                     high=obs_upper_bound,
                                                                     dtype=np.float32
                                                                     ),
                                                 "neighbors": spaces.MultiBinary(self.NUM_DRONES),
                                                 "rgb": spaces.Box(low=0,
                                                                   high=255,
                                                                   shape=(self.IMG_RES[1], self.IMG_RES[0], 4),
                                                                   dtype=np.uint8
                                                                   ),
                                                 "dep": spaces.Box(low=.01,
                                                                   high=1000.,
                                                                   shape=(self.IMG_RES[1],
                                                                    self.IMG_RES[0]),
                                                                   dtype=np.float32
                                                                   ),
                                                 "seg": spaces.Box(low=0,
                                                                   high=100,
                                                                   shape=(self.IMG_RES[1],
                                                                   self.IMG_RES[0]),
                                                                   dtype=int
                                                                   )
                                                 }) for i in range(self.NUM_DRONES)})
    
    ################################################################################
    
    def _computeObs(self):
        """Returns the current observation of the environment.

        For the value of key "state", see the implementation of `_getDroneStateVector()`,
        the value of key "neighbors" is the drone's own row of the adjacency matrix,
        "rgb", "dep", and "seg" are matrices containing POV camera captures.

        Returns
        -------
        dict[str, dict[str, ndarray]]
            A Dict with NUM_DRONES entries indexed by Id in string format,
            each a Dict in the form {Box(20,), MultiBinary(NUM_DRONES), Box(H,W,4), Box(H,W), Box(H,W)}.

        """
        adjacency_mat = self._getAdjacencyMatrix()
        obs = {}
        for i in range(self.NUM_DRONES):
            if self.step_counter%self.IMG_CAPTURE_FREQ == 0:
                self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i)
                #### Printing observation to PNG frames example ############
                if self.RECORD:
                    self._exportImage(img_type=ImageType.RGB, # ImageType.BW, ImageType.DEP, ImageType.SEG
                                      img_input=self.rgb[i],
                                      path=self.ONBOARD_IMG_PATH+"drone_"+str(i),
                                      frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ)
                                      )
            obs[str(i)] = {"state": self._getDroneStateVector(i), \
                           "neighbors": adjacency_mat[i,:], \
                           "rgb": self.rgb[i], \
                           "dep": self.dep[i], \
                           "seg": self.seg[i] \
                           }
        return obs

    ################################################################################

    def _pcd_generation(self,depth_image, rgb_image=None):

        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)
        
        nearVal = self.L
        farVal = 1000
        depth_mm = (2 * nearVal * farVal) / (farVal + nearVal - depth_image * (farVal - nearVal))
        
        height, width = depth_image.shape
        #print(width, height)
        aspect = width/height
        fov = 90    
        
        fx = width / (2*np.tan(np.radians(fov / 2)))
        fy = height / (2*np.tan(np.radians(fov / 2)))
        cx = width/2
        cy = height/2
        s = 0
        intrinsic_matrix = np.array([[fx, s, cx],
                             [0, fy, cy],
                             [0, 0, 1]])

        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.intrinsic_matrix = intrinsic_matrix
        # Extract drone state
        drone_state = self._getDroneStateVector(0)
        drone_position = drone_state[:3]  # The first three elements are the position
        drone_orientation_quat = drone_state[3:7]  # The next four elements are the quaternion
        # Create the extrinsic transformation matrix
        depth_o3d = o3d.geometry.Image(depth_mm)
        rot_mat = np.array(p.getMatrixFromQuaternion(drone_orientation_quat)).reshape(3, 3)
        #### Set target point, camera view and projection matrices #
        target = np.dot(rot_mat,np.array([1000, 0, 0])) + np.array(drone_position)
        DRONE_CAM_VIEW = p.computeViewMatrix(cameraEyePosition=drone_position+np.array([0, 0, self.L]),
                                             cameraTargetPosition=target,
                                             cameraUpVector=[0, 0, 1],
                                             physicsClientId=self.CLIENT
                                             )
        DRONE_CAM_VIEW = self.get_extrinsics(np.array(DRONE_CAM_VIEW).reshape(4,4))
        if rgb_image is not None:
            rgb_o3d = o3d.geometry.Image(rgb_image.astype(np.uint8))  # Convert RGB to Open3D Image
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic, DRONE_CAM_VIEW)
        else:
            pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, intrinsic, DRONE_CAM_VIEW)
        
        distances = np.linalg.norm(np.asarray(pcd.points), axis=1)
        indices = np.where((distances >= 0.15) & (distances <= farVal))[0]
        pcd = pcd.select_by_index(indices)
        return pcd
    
    ################################################################################
    
    def get_extrinsics(self, view_matrix):
        Tc = np.array([[1,  0,  0,  0],
                    [0,  -1,  0,  0],
                    [0,  0,  -1,  0],
                    [0,  0,  0,  1]]).reshape(4,4)
        T = np.linalg.inv(view_matrix) @ Tc

        return T

    ################################################################################

    def _occupancy_generation(self,depth_image):
        point_cloud = self._pcd_generation(depth_image)
        voxel_size = 0.1  # Set voxel size as per requirement
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(point_cloud, voxel_size=voxel_size)

        # Step 4: Create the occupancy grid
        min_bound = voxel_grid.get_min_bound()
        max_bound = voxel_grid.get_max_bound()
        grid_size = np.ceil((max_bound - min_bound) / voxel_size).astype(int)
        voxels = voxel_grid.get_voxels()
        voxel_indices = np.array([v.grid_index for v in voxels])
        occupancy_grid = np.zeros(grid_size, dtype=bool)
        for idx in voxel_indices:
            occupancy_grid[idx[0], idx[1], idx[2]] = True

        # Optional: Save the occupancy grid
        #np.save("occupancy_grid.npy", occupancy_grid)
        print("Occupancy grid created and saved successfully.")
        return occupancy_grid

    ################################################################################

    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Clips and converts a dictionary into a 2D array.

        Parameters
        ----------
        action : dict[str, ndarray]
            The (unbounded) input action for each drone, to be translated into feasible RPMs.

        Returns
        -------
        ndarray
            (NUM_DRONES, 4)-shaped array of ints containing to clipped RPMs
            commanded to the 4 motors of each drone.

        """
        clipped_action = np.zeros((self.NUM_DRONES, 4))
        for k, v in action.items():
            clipped_action[int(k), :] = np.clip(np.array(v), 0, self.MAX_RPM)
        return clipped_action

    ################################################################################

    def _computeReward(self):
        """Computes the current reward value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        int
            Dummy value.

        """
        return -1

    ################################################################################
    
    def _computeDone(self):
        """Computes the current done value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        bool
            Dummy value.

        """
        return False

    ################################################################################
    
    def _computeInfo(self):
        """Computes the current info dict(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        dict[str, int]
            Dummy value.

        """
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years

    def _addObstacles(self):
        """Add obstacles to the environment, including multiple cylinders of different colors at fixed positions."""
        super()._addObstacles()
        base_path = pkg_resources.resource_filename('gym_pybullet_drones', 'assets')
        cylinder_colors = ['red', 'orange', 'green']
        cylinders = [os.path.join(base_path, f"{color}_cylinder.urdf") for color in cylinder_colors for _ in range(3)]
        # obstacles = os.path.join(base_path, "gate.urdf")
        # Fixed positions
        self.fixed_positions = [
            (0.5, 0.5, 0.5),
            (0.5, 0, 0.5),
            (0.5, -0.5, 0.5),
            (0, 0.5, 0.5),
            (0, 5, 0.5),
            (0, -0.5, 0.5),
            (-0.5, 0.5, 0.5),
            (-0.5, 0, 0.5),
            (-0.5, -0.5, 0.5)
        ]

        # obstacle_pos = (0.0, 0.0, 0.0)

        for urdf, pos in zip(cylinders, self.fixed_positions):
            if os.path.exists(urdf):
                p.loadURDF(urdf,
                        pos,
                        p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=True,
                        physicsClientId=self.CLIENT
                        )
            else:
                print(f"File not found: {urdf}")

        # if os.path.exists(obstacles):
        #     p.loadURDF(obstacles,
        #                 obstacle_pos,
        #                 p.getQuaternionFromEuler([0, 0, 0]),
        #                 useFixedBase=True,
        #                 physicsClientId=self.CLIENT)
        # else:
        #     print(f"File not found: {obstacles}")
