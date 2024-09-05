#!/home/astik/anaconda3/envs/drones/bin/python
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import csv
import numpy as np
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
import sys, os, getpass
sys.path.append(sys.path[0].replace("ros2/install/clustering_python/lib/clustering_python", ""))
sys.path.append("/home/"+getpass.getuser()+"/anaconda3/envs/drones/lib/python3.10/site-packages")  
import torch
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if torch.cuda.is_available():
    # Set the device to GPU
    device = torch.device("cuda")
    print("Using GPU for computation.")
    print("Device type:", torch.cuda.get_device_name(device))
    print("Number of GPUs:", torch.cuda.device_count())
else:
    # Set the device to CPU
    device = torch.device("cpu")
    print("GPU not available. Using CPU for computation.")

class PointCloudProcessing(Node):

    def __init__(self):
        super().__init__('clustering_python')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcd_gym_pybullet',  # Replace with your actual topic
            self.point_cloud_callback,
            10)
        self.pcd_og = o3d.geometry.PointCloud()
        self.publisher_ = self.create_publisher(BoundingBox3DArray, 'pcd_bounding_boxes', 10)
        timer_period_sec = 0.1
        self.timer = self.create_timer(timer_period_sec, self.step_callback)
        self.marker_publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)  # Marker publisher for RViz

    def point_cloud_callback(self, point_cloud_ros2):
        # Assign the NumPy array to the point cloud
        self.pcd_og.points = o3d.utility.Vector3dVector(self.ros2_to_o3d_pcd(point_cloud_ros2))

    def step_callback(self):

        pcd_downsampled = self.pcd_og.voxel_down_sample(voxel_size=0.05)
        outlier_cloud, inlier_cloud, = self.ransac(pcd_downsampled, distance_threshold=0.1, ransac_n=3, num_iterations=2000)


        ### ------- CLUSTERING
        roi_outliers, labels = self.dbscan(outlier_cloud, eps=0.3, min_points=50)

        ### ------- PCA
        boxes = self.get_bounding_boxes(labels, roi_outliers, MAX_POINTS=1500)
        boxes_ros2 = self.convert_list_of_oriented_bboxes(boxes)
        self.publisher_.publish(boxes_ros2)
        self.publish_bounding_boxes_as_markers(boxes)

    def ros2_to_o3d_pcd(self, point_cloud):
        points_list = []

        for point in pc2.read_points(point_cloud, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])  # X, Y, Z coordinates

        # Convert to numpy array
        points_array = np.array(points_list)
        return points_array
    
    def oriented_bbox_to_ros_bbox3d(self,oriented_bbox: o3d.geometry.OrientedBoundingBox) -> BoundingBox3D:
        bbox3d = BoundingBox3D()
        print(type(oriented_bbox))
        # Extract the center (translation) of the OrientedBoundingBox
        bbox3d.center.position.x = oriented_bbox.center[0]
        bbox3d.center.position.y = oriented_bbox.center[1]
        bbox3d.center.position.z = oriented_bbox.center[2]

        # Extract the size of the OrientedBoundingBox (extent)
        bbox3d.size.x = oriented_bbox.extent[0]
        bbox3d.size.y = oriented_bbox.extent[1]
        bbox3d.size.z = oriented_bbox.extent[2]

        # Note: ROS BoundingBox3D doesn't directly support orientation, so if you need
        # to represent the orientation, you'd have to include that in a Pose message or
        # handle the rotation separately.

        return bbox3d
    def convert_list_of_oriented_bboxes(self, oriented_bboxes: list) -> BoundingBox3DArray:
        # Create an empty BoundingBox3DArray message
        bounding_box_array = BoundingBox3DArray()

        # Convert each OrientedBoundingBox to a BoundingBox3D and append to the array
        for obbox in oriented_bboxes:
            ros_bbox = self.oriented_bbox_to_ros_bbox3d(obbox)
            bounding_box_array.boxes.append(ros_bbox)
        
        return bounding_box_array
    
    def publish_bounding_boxes_as_markers(self, boxes):
        for i, obb in enumerate(boxes):
            marker = Marker()
            marker.header.frame_id = "camera_link"  # Replace with your coordinate frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bounding_boxes"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.01  # Line width

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Get the 8 corners of the bounding box
            corners = np.asarray(obb.get_box_points())

            # Define the edges of the box
            edges = [
                [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
                [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
                [0, 4], [1, 5], [2, 6], [3, 7]   # Side edges
            ]

            for edge in edges:
                start, end = edge
                p1 = Point(x=corners[start][0], y=corners[start][1], z=corners[start][2])
                p2 = Point(x=corners[end][0], y=corners[end][1], z=corners[end][2])
                marker.points.append(p1)
                marker.points.append(p2)

            self.marker_publisher_.publish(marker)
    def ransac(self, point_cloud, distance_threshold=0.33, ransac_n=3, num_iterations=100):
        """
        RANSAC-based plane segmentation for a point cloud.

        Parameters:
            point_cloud (open3d.geometry.PointCloud): The input point cloud.
            distance_threshold (float, optional): The maximum distance a point can be from the plane to be considered an inlier.
                Default is 0.33.
            ransac_n (int, optional): The number of points to randomly sample for each iteration of RANSAC. Default is 3.
            num_iterations (int, optional): The number of RANSAC iterations to perform. Default is 100.

        Returns:
            open3d.geometry.PointCloud, open3d.geometry.PointCloud: Two point clouds representing the inliers and outliers
            of the segmented plane, respectively.
        """
        # Perform plane segmentation using RANSAC
        plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n,
                                                        num_iterations=num_iterations)

        # Extract inlier and outlier point clouds
        inlier_cloud = point_cloud.select_by_index(inliers)
        outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

        # Color the outlier cloud red and the inlier cloud blue
        outlier_cloud.paint_uniform_color([0.8, 0.2, 0.2])  # Red
        inlier_cloud.paint_uniform_color([0.25, 0.5, 0.75])  # Blue

        return outlier_cloud, inlier_cloud




    def dbscan(self, outlier_cloud, eps=1.0, min_points=10):
        """
        Perform Density-Based Spatial Clustering of Applications with Noise (DBSCAN) on the input point cloud.

        Parameters:
            outlier_cloud (open3d.geometry.PointCloud): The input point cloud to be clustered.
            eps (float, optional): The maximum distance between two points for one to be considered as in the neighborhood of the other.
                Default is 1.0.
            min_points (int, optional): The minimum number of points required to form a dense region (core points). Default is 10.

        Returns:
            open3d.geometry.PointCloud: The input point cloud with updated cluster colors.
            numpy.ndarray: Array of cluster labels assigned to each point in the point cloud.
        """

        # Perform DBSCAN clustering on the input point cloud
        labels = np.array(outlier_cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

        # Find the maximum cluster label to get the total number of clusters
        max_label = labels.max()
        if max_label > 0:
            print(f"\nPoint cloud has {max_label + 1} clusters")

        # Generate colors for the clusters using a colormap
        #colors = plt.get_cmap("hsv")(labels / (max_label if max_label > 0 else 1))
        #colors[labels < 0] = 0
        colors = np.zeros((len(labels), 3))
        
                # Generate a random color for each cluster
        for i in range(max_label + 1):
            random_color = np.random.rand(3)  # Generate a random RGB color
            colors[labels == i] = random_color  # Assign the color to the points in the cluster

        # Assign black color to noise points (label == -1)
        colors[labels == -1] = [0, 0, 0]

        # Assign the computed colors to the point cloud
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors)

        # Assign the computed colors to the point cloud
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # Return the point cloud with updated colors and the cluster labels
        return outlier_cloud, labels





    def get_bounding_boxes(self, labels, outlier_cloud, MAX_POINTS=300):
        """
        Get axis-aligned bounding boxes (boxes) for clusters in a point cloud.

        Parameters:
            labels (numpy.ndarray): The cluster labels assigned to each point in the point cloud.
            outlier_cloud (open3d.geometry.PointCloud): The point cloud containing the outlier points.

        Returns:
            list of open3d.geometry.OrientedBoundingBox: A list of axis-aligned bounding boxes (boxes) for each cluster.
        """
        # Extract points for each cluster
        clusters = {}
        for i, label in enumerate(labels):
            if label >= 0:
                if label not in clusters:
                    clusters[label] = []
                clusters[label].append(outlier_cloud.points[i])

        # Filter out clusters with more than 300 points (optional)
        clusters = {label: points for label, points in clusters.items() if len(points) <= MAX_POINTS}

        # Create boxes for each cluster
        boxes = []
        for points in clusters.values():
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(points)
            aabb = cluster_cloud.get_oriented_bounding_box() #PCA
            boxes.append(aabb)
        return boxes    
    

def main(args=None):
    rclpy.init(args=args)
    point_cloud_processing = PointCloudProcessing()
    rclpy.spin(point_cloud_processing)

    # Destroy the node explicitly
    point_cloud_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()