import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from rrt_python.corridor_finder import RRT_start
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

class PointCloudPlanner(Node):
    def __init__(self):
        super().__init__('point_cloud_planner')
        
        # Declare and read parameters
        self.declare_parameter('plan_rate', 10.0)
        self.declare_parameter('safety_margin', 1000000.0)
        self.declare_parameter('search_margin', 0.50)
        self.declare_parameter('max_radius', 1.5)
        self.declare_parameter('sensing_range', 10.0)
        self.declare_parameter('refine_portion', 0.80)
        self.declare_parameter('sample_portion', 0.25)
        self.declare_parameter('goal_portion', 0.05)
        self.declare_parameter('path_find_limit', 50.0)
        self.declare_parameter('max_samples', 3000)
        self.declare_parameter('stop_horizon', 0.5)
        self.declare_parameter('commit_time', 1.0)
        self.declare_parameter('x_l', -105.0)
        self.declare_parameter('x_h', 105.0)
        self.declare_parameter('y_l', -105.0)
        self.declare_parameter('y_h', 105.0)
        self.declare_parameter('z_l', 0.0)
        self.declare_parameter('z_h', 1.0)

        # Read the parameters
        self._planning_rate = self.get_parameter('plan_rate').value
        self._safety_margin = self.get_parameter('safety_margin').value
        self._search_margin = self.get_parameter('search_margin').value
        self._max_radius = self.get_parameter('max_radius').value
        self._sensing_range = self.get_parameter('sensing_range').value
        self._refine_portion = self.get_parameter('refine_portion').value
        self._sample_portion = self.get_parameter('sample_portion').value
        self._goal_portion = self.get_parameter('goal_portion').value
        self._path_find_limit = self.get_parameter('path_find_limit').value
        self._max_samples = self.get_parameter('max_samples').value
        self._stop_time = self.get_parameter('stop_horizon').value
        self._time_commit = self.get_parameter('commit_time').value
        self._x_l = self.get_parameter('x_l').value
        self._x_h = self.get_parameter('x_h').value
        self._y_l = self.get_parameter('y_l').value
        self._y_h = self.get_parameter('y_h').value
        self._z_l = self.get_parameter('z_l').value
        self._z_h = self.get_parameter('z_h').value

        # Initialize RRT planner
        self.rrtPathPlanner = RRT_start(self._safety_margin, self._search_margin, self._max_radius, self._sensing_range)

        # Publishers
        self._vis_corridor_pub = self.create_publisher(MarkerArray, 'flight_corridor', 1)
        self._rrt_waypoints_pub = self.create_publisher(Path, 'rrt_waypoints', 1)
        self._vis_rrt_tree_pub = self.create_publisher(MarkerArray, 'rrt_tree_visualization', 1)
        self._vis_subscribed_pcd = self.create_publisher(PointCloud2,'pcd_rrt_vis',2)

        # Subscribers
        self._odom_sub = self.create_subscription(Odometry, 'odometry', self.rcv_odometry_callback, 1)
        self._obs_sub = self.create_subscription(Float32MultiArray, 'obs', self.rcv_obs_callback, 1)
        self._dest_pts_sub = self.create_subscription(Path, 'waypoints', self.rcv_waypoints_callback, 1)
        self._map_sub = self.create_subscription(PointCloud2, 'pcd_gym_pybullet', self.rcv_pointcloud_callback, 1)

        # Timer for planning
        self._planning_timer = self.create_timer(1.0 / self._planning_rate, self.planning_callback)

        # Path Planning Variables
        self._start_pos = np.zeros(3)
        self._end_pos = np.zeros(3)
        self._is_traj_exist = False
        self._is_target_arrive = False
        self._is_target_receive = False
        self._is_has_map = False

    def rcv_waypoints_callback(self, msg):
        if not msg.poses or msg.poses[0].pose.position.z < 0.0:
            return

        self._end_pos[0] = msg.poses[0].pose.position.x
        self._end_pos[1] = msg.poses[0].pose.position.y
        self._end_pos[2] = msg.poses[0].pose.position.z

        self._is_target_receive = True
        self._is_target_arrive = False
        self._is_traj_exist = False

    def rcv_obs_callback(self, msg):
        self._start_pos[0] = msg.data[0]
        self._start_pos[1] = msg.data[1]
        self._start_pos[2] = msg.data[2]

    def rcv_odometry_callback(self, msg):
        self._start_pos[0] = msg.pose.pose.position.x
        self._start_pos[1] = msg.pose.pose.position.y
        self._start_pos[2] = msg.pose.pose.position.z

        if self._is_target_receive:
            self.rrtPathPlanner.setStartPt(self._start_pos, self._end_pos)

    def rcv_pointcloud_callback(self, msg):
        if not msg.data:
            return

        # Convert PointCloud2 to numpy array
        # point_cloud_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        # point_cloud = np.array([[point[0], point[1], point[2]] for point in point_cloud_list])
        
        points_list = []

        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])  # X, Y, Z coordinates

        # Convert to numpy array
        points_array = np.array(points_list)

        self._is_has_map = True
        self.rrtPathPlanner.setInput(points_array)
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Define fields for PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]   

        # Create PointCloud2 message
        pointcloud_msg = pc2.create_cloud(header, fields, points_array)
        self._vis_subscribed_pcd.publish(pointcloud_msg)


    def publish_rrt_waypoints(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for point in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path_msg.poses.append(pose)

        self._rrt_waypoints_pub.publish(path_msg)

    def publish_corridor_visualization(self, path, radii):
        corridor_markers = MarkerArray()

        for i, point in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "corridor"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]

            diameter = 2.0 * radii[i]  # Radius to diameter
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter

            marker.color.a = 0.5  # Transparency
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            corridor_markers.markers.append(marker)

        self._vis_corridor_pub.publish(corridor_markers)

    def publish_rrt_tree(self):
        """
        Publish the RRT tree as branches (paths) connecting nodes without visualizing the nodes.
        """
        tree_markers = MarkerArray()
        marker_id = 0

        # Loop through all the nodes in the tree
        for node in self.rrtPathPlanner.nodeList:
            # Skip visualizing nodes, only visualize branches (paths)
            if node.preNode is not None:
                branch_marker = Marker()
                branch_marker.header.frame_id = "map"
                branch_marker.header.stamp = self.get_clock().now().to_msg()
                branch_marker.ns = "rrt_branches"
                branch_marker.id = marker_id
                branch_marker.type = Marker.LINE_STRIP
                branch_marker.action = Marker.ADD

                # Define start and end points for the branch (from node to its parent)
                start_point = Pose().position
                start_point.x = node.coordinates[0]
                start_point.y = node.coordinates[1]
                start_point.z = node.coordinates[2]

                end_point = Pose().position
                end_point.x = node.preNode.coordinates[0]
                end_point.y = node.preNode.coordinates[1]
                end_point.z = node.preNode.coordinates[2]

                branch_marker.points.append(start_point)
                branch_marker.points.append(end_point)

                branch_marker.scale.x = 0.01  # Line width
                branch_marker.color.a = 0.8  # Higher transparency for better visibility
                branch_marker.color.r = 0.0
                branch_marker.color.g = 0.0
                branch_marker.color.b = 1.0  # Blue color for branches

                tree_markers.markers.append(branch_marker)
                marker_id += 1

        # Publish only the branches
        self._vis_rrt_tree_pub.publish(tree_markers)

    def plan_initial_traj(self):
        self.rrtPathPlanner.reset()
        self.rrtPathPlanner.setPt(self._start_pos, self._end_pos, self._x_l, self._x_h, self._y_l, self._y_h,
                                  self._z_l, self._z_h, 10.0, self._max_samples, self._sample_portion,
                                  self._goal_portion)
        self.rrtPathPlanner.safeRegionExplansion(self._path_find_limit)

        path, radius = self.rrtPathPlanner.getPath()
        if self.rrtPathPlanner.getPathExistStatus():
            self.publish_rrt_waypoints(path)
            self.publish_corridor_visualization(path, radius)
            self.publish_rrt_tree()  # New function to visualize the RRT tree
            self._is_traj_exist = True
        else:
            self.get_logger().warn("No path found in initial trajectory planning")

    def plan_incremental_traj(self):
        if self._is_target_arrive:
            if not self.rrtPathPlanner.getPathExistStatus():
                self.get_logger().warn("Reached committed target but no feasible path exists")
                self._is_traj_exist = False
            else:
                self.rrtPathPlanner.resetRoot(self._max_radius)
                path, radius = self.rrtPathPlanner.getPath()
                self.publish_rrt_waypoints(path)
                self.publish_corridor_visualization(path, radius)
                self.publish_rrt_tree()  # New function to visualize the RRT tree

        else:
            self.rrtPathPlanner.safeRegionRefine(self._refine_portion)
            path, radius = self.rrtPathPlanner.getPath()
            self.publish_rrt_waypoints(path)
            self.publish_corridor_visualization(path, radius)
            self.publish_rrt_tree()  # New function to visualize the RRT tree


    def planning_callback(self):
        if not self._is_target_receive or not self._is_has_map:
            self.get_logger().debug("No target or map received.")
            return

        self.plan_initial_traj()
        #if not self._is_traj_exist:
            # self.plan_initial_traj()
        # else:
            # self.plan_incremental_traj()

def main(args=None):
    rclpy.init(args=args)
    planner = PointCloudPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
