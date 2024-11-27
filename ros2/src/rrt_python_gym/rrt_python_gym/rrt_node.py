import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from rrt_python_gym.corridor_finder import RRT_start
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from std_msgs.msg import Header
import tf2_ros
import tf2_py
from scipy.spatial.transform import Rotation as R
from rrt_python_gym.trajectory import Trajectory
import time
from custom_interface.msg import TrajMsg
class PointCloudPlanner(Node):
    def __init__(self):
        super().__init__('point_cloud_planner')
        
        # Declare and read parameters
        self.declare_parameter('plan_rate', 10.0)
        self.declare_parameter('safety_margin', 1.5)
        self.declare_parameter('search_margin', 0.5)
        self.declare_parameter('max_radius', 2.5)
        self.declare_parameter('sensing_range', 10.0)
        self.declare_parameter('refine_portion', 0.80)
        self.declare_parameter('sample_portion', 0.25)
        self.declare_parameter('goal_portion', 0.05)
        self.declare_parameter('path_find_limit', 50.0)
        self.declare_parameter('max_samples', 3000)
        self.declare_parameter('stop_horizon', 0.5)
        self.declare_parameter('commit_time', 1.0)
        self.declare_parameter('x_l', 0.0)
        self.declare_parameter('x_h', 200.0)
        self.declare_parameter('y_l', -15.0)
        self.declare_parameter('y_h', 15.0)
        self.declare_parameter('z_l', 0.1)
        self.declare_parameter('z_h', 1.5)

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
        self._vis_subscribed_pcd = self.create_publisher(PointCloud2,'pcd_rrt_vis',1)
        self.path_visualizer = self.create_publisher(MarkerArray,'vis_rrt_wp',1)
        self.traj_publisher = self.create_publisher(TrajMsg, 'Trajectory',1)

        # Subscribers
        self._obs_sub = self.create_subscription(Float32MultiArray, 'obs', self.rcv_obs_callback, 1)
        self._dest_pts_sub = self.create_subscription(Path, 'waypoints', self.rcv_waypoints_callback, 1)
        self._map_sub = self.create_subscription(PointCloud2, 'pcd_gym_pybullet', self.rcv_pointcloud_callback, 1)

        # Timer for planning
        self._planning_timer = self.create_timer(1.0 / self._planning_rate, self.planning_callback)

        self._path = None
        self._radius = None

        # Path Planning Variables
        self._start_pos = np.zeros(3)
        self._end_pos = np.zeros(3)
        self._is_traj_exist = False
        self._is_target_arrive = False
        self._is_target_receive = False
        self._is_has_map = False

        # Transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.commit_target = None
        self.commit_target_reached = False
        self._waypoint_callback_triggered = False

        trajSelect = np.zeros(4)

        # Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel,            3: xyz_pos with geometric)
        ctrlType = "xyz_pos"   
        # Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,    
        #                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
        #                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
        #                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
        #                                 12: pos_waypoint_arrived     13: pos_waypoint_arrived_wait
        trajSelect[0] = 3       
        # Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
        trajSelect[1] = 3           
        # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
        trajSelect[2] = 1
        v_avg = 0.3 # m/s
        self.traj = Trajectory(ctrlType, trajSelect, v_avg)
        self._psi = 0
        traj_msg = TrajMsg()

    def rcv_waypoints_callback(self, msg):
        if not msg.poses or msg.poses[0].pose.position.z < 0.0:
            return
        if self._waypoint_callback_triggered:
            return
        self._end_pos[0] = msg.poses[0].pose.position.x
        self._end_pos[1] = msg.poses[0].pose.position.y
        self._end_pos[2] = msg.poses[0].pose.position.z

        self.commit_target = self._end_pos

        self._is_target_receive = True
        self.commit_target_reached = False
        self._is_traj_exist = False
        print("[waypoint cb] self._is_traj_exist", self._is_traj_exist)

        print("waypoint callback called, is_traj_exist set to False")
        self._waypoint_callback_triggered = True

    def rcv_obs_callback(self, msg):
        self._start_pos[0] = msg.data[0]
        self._start_pos[1] = msg.data[1]
        self._start_pos[2] = msg.data[2]
        self._psi = msg.data[9]
        if self._is_target_receive:
            self.rrtPathPlanner.setStartPt(self._start_pos, self._end_pos)
        
        if self.commit_target is not None:
            dist_commit = self.rrtPathPlanner.getDis(self._start_pos, self.commit_target)
            print("distance between current position and commit target: ",dist_commit)
            if self._is_traj_exist and  dist_commit< 0.5:
                self.commit_target_reached = True
        

    def rcv_pointcloud_callback(self, msg):
        try:
            # Get the transform between 'map' and the point cloud's frame
            transform = self.tf_buffer.lookup_transform('ground_link', msg.header.frame_id, rclpy.time.Time())

            # Extract translation and rotation (quaternion)
            translation = (transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z)
            quaternion = (transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w)

            # Convert quaternion to rotation matrix using scipy
            rotation_matrix = R.from_quat(quaternion).as_matrix()

            # Transform points
            points_list = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # Apply rotation and translation
                rotated_point = np.dot(rotation_matrix, np.array([point[0], point[1], point[2]]))
                transformed_point = rotated_point + np.array(translation)
                points_list.append(transformed_point)

            # Convert list to numpy array
            points_array = np.array(points_list, dtype=np.float32)

            # Set the transformed point cloud as input to your planner
            self.rrtPathPlanner.setInput(points_array)
            self._is_has_map = True
            # if not self.rrtPathPlanner.getPathExistStatus():
            #     print("Traj not existing, reverting to expansion")
            #     self.is_traj_exist = False

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "ground_link"

            # Define fields for PointCloud2
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]   

            # Create PointCloud2 message
            pointcloud_msg = pc2.create_cloud(header, fields, points_array)

            # Publish the PointCloud2 message
            self._vis_subscribed_pcd.publish(pointcloud_msg)


        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        # Compute the rotation matrix from the quaternion
        rotation_matrix = np.array([
            [1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
            [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw],
            [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2]
        ])
        return rotation_matrix


    def publish_rrt_waypoints(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "ground_link"
        commit_wp_list = []
        
        if len(path) > 2:
            for i in range(0,3):
                point = path[i]
                commit_wp_list.append(point)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "ground_link"
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                path_msg.poses.append(pose)    
        else:
            for i in range(0,len(path)):
                point = path[i]
                commit_wp_list.append(point)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "ground_link"
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                path_msg.poses.append(pose)

        self._rrt_waypoints_pub.publish(path_msg)

    def publish_rrt_traj(self, sDes):
        traj_msg = TrajMsg()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "ground_link"
        pos_sp    = sDes[0:3]
        vel_sp    = sDes[3:6]
        acc_sp    = sDes[6:9]
        eul_sp    = sDes[12:15]
        jerk_sp   = sDes[19:22]
        snap_sp   = sDes[22:25]
        
        traj_msg.position = Point(x=pos_sp[0], y=pos_sp[1], z=pos_sp[2])
        traj_msg.velocity = Vector3(x=vel_sp[0], y=vel_sp[1], z=vel_sp[2])
        traj_msg.acceleration = Vector3(x=acc_sp[0], y=acc_sp[1], z=acc_sp[2])
        traj_msg.jerk = Vector3(x=jerk_sp[0], y=jerk_sp[1], z=jerk_sp[2])
        traj_msg.snap = Vector3(x=snap_sp[0], y=snap_sp[1], z=snap_sp[2])
        traj_msg.yaw = 0.0
        
        if self.rrtPathPlanner.getDis(self._start_pos, self._end_pos) < 0.5:
            traj_msg.position = Point(x = self._end_pos[0], y = self._end_pos[1], z = self._end_pos[2])
            traj_msg.velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish the message
        self.traj_publisher.publish(traj_msg)


    def publish_corridor_visualization(self, path, radii):
        corridor_markers = MarkerArray()

        for i, point in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "ground_link"
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

    def complete_path_visualizer(self, path):
        print("path visualizer called")
        complete_path_marker = MarkerArray()
        marker_id = 0
        for i in range(1, len(path)):
            point = Marker()
            point.header.frame_id = "ground_link"
            point.header.stamp = self.get_clock().now().to_msg()
            point.ns = "rrt_complete_path"
            point.id = marker_id
            point.type = Marker.LINE_STRIP
            point.action = Marker.ADD

            start_pt = Pose().position
            start_pt.x = path[i-1][0]
            start_pt.y = path[i-1][1]
            start_pt.z = path[i-1][2]

            end_pt = Pose().position
            end_pt.x = path[i][0]
            end_pt.y = path[i][1]
            end_pt.z = path[i][2]
            point.points.append(start_pt)
            point.points.append(end_pt)

            point.scale.x = 0.01  # Line width
            point.color.a = 0.8  # Higher transparency for better visibility
            point.color.r = 1.0
            point.color.g = 0.584
            point.color.b = 0.0  # Blue color for branches
            complete_path_marker.markers.append(point)
            marker_id+=1
        
        self.path_visualizer.publish(complete_path_marker)


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
                branch_marker.header.frame_id = "ground_link"
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
        print("in init planner")
        self.rrtPathPlanner.reset()
        self.rrtPathPlanner.setPt(self._start_pos, self._end_pos, self._x_l, self._x_h, self._y_l, self._y_h,
                                  self._z_l, self._z_h, 10.0, self._max_samples, self._sample_portion,
                                  self._goal_portion)
        self.rrtPathPlanner.safeRegionExpansion(0.5)
        print("path get status: ",self.rrtPathPlanner.getPathExistStatus())
        if self.rrtPathPlanner.getPathExistStatus():
            self._path, self._radius = self.rrtPathPlanner.getPath()
            self.commit_target, self.wps = self.getcommittedtarget()
            self.t_assign = time.time()
            self.traj.set_coefficients(self.wps)
            self.commit_target_assigned = True
            self.rrtPathPlanner.resetRoot(self.commit_target)
            sdes =  self.traj.desiredState(0, 0.01, self._start_pos, self._psi)
            self.publish_rrt_traj(sdes)
            self.publish_rrt_waypoints(self._path)
            self.complete_path_visualizer(self._path)
            self.publish_corridor_visualization(self._path, self._radius)
            self.publish_rrt_tree()  # New function to visualize the RRT tree

            self._is_traj_exist = True
            print("[initial planner] self._is_traj_exist", self._is_traj_exist)
        else:
            self.get_logger().warn("No path found in initial trajectory planning: giving hover command")
            path_hover = []
            self.publish_rrt_tree()
            path_hover.append(self._start_pos)
            self.publish_rrt_waypoints(path_hover)
            self.is_traj_exist = False
    def plan_incremental_traj(self):
        if self.checkEndOfCommitedTraj():
            if not self.rrtPathPlanner.getPathExistStatus():
                self.get_logger().warn("Reached committed target but no feasible path exists")
                self._is_traj_exist = False
                print("[incremental cb if 2] self._is_traj_exist", self._is_traj_exist)

            else:
                self.commit_target, self.wps = self.getcommittedtarget()
                self.traj.set_coefficients(self.wps)
                self.t_assign = time.time()
                sdes =  self.traj.desiredState(0, 0.01, self._start_pos, self._psi)
                self.publish_rrt_traj(sdes)
                self.rrtPathPlanner.resetRoot(self.commit_target)
                self._path, self._radius = self.rrtPathPlanner.getPath()
                self.publish_rrt_waypoints(self._path)
                self.complete_path_visualizer(self._path)
                self.publish_corridor_visualization(self._path, self._radius)
                self.complete_path_visualizer(self._path)
                self.publish_rrt_tree()  # New function to visualize the RRT tree
        else:
            print("in evaluate/refine loop")
            print('Path exist status: ',self.rrtPathPlanner.getPathExistStatus())
            self.rrtPathPlanner.safeRegionRefine(0.05)
            self.rrtPathPlanner.safeRegionEvaluate(0.05)
            del_t = time.time()-self.t_assign
            sdes = self.traj.desiredState(del_t, 0.01, self._start_pos, self._psi)
            self.publish_rrt_traj(sdes)
            if self.rrtPathPlanner.getPathExistStatus():
                self._path, self._radius = self.rrtPathPlanner.getPath()
            
            self.publish_corridor_visualization(self._path, self._radius)
            self.complete_path_visualizer(self._path)
            self.publish_rrt_tree()  # New function to visualize the RRT tree


    def planning_callback(self):
        if not self._is_target_receive or not self._is_has_map:
            self.get_logger().debug("No target or map received.")
            return
        
        if not self._is_traj_exist:
            self.plan_initial_traj()
        else:
            self.plan_incremental_traj()
    
    def checkEndOfCommitedTraj(self):
        if self.commit_target_reached:
            self.commit_target_reached = False
            return True
        else:
            return False
    def getcommittedtarget(self):
        # for i in range(0,len(self._path)):
        #     if i < len(self._path)-1 and self.rrtPathPlanner.getDis(self._path[i],self._start_pos) > 5 and self.rrtPathPlanner.getDis(self._path[i+1], self._start_pos) < 7:
        #         print("commit_target_set: ", self._path[i],", distance: ", self.rrtPathPlanner.getDis(self._start_pos, self._path[i]))
        #         return self._path[i]
        k = len(self._path)-1
        wps = []
        if k+1 > 2:
            print("commit target set as: ",self._path[2][0],self._path[2][1],self._path[2][2])
            for i in range(0,3):
                wps.append(self._path[i])
            return self._path[2], wps
        else:
            print("commit target set as: ",self._path[k][0],self._path[k][1],self._path[k][2])
            for i in range(0,k+1):
                wps.append(self._path[i])
            return self._path[k], wps        

def main(args=None):
    rclpy.init(args=args)
    planner = PointCloudPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
