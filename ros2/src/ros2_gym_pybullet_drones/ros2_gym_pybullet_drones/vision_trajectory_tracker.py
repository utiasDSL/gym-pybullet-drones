"""ROS2 Python wrapper node for class VisionAviary.

It creates an environment VisionAviary and continually calls CtrlAviary.step().
It publishes on topic 'obs' and reads from topic 'action'.
"""
import sys, os  # See: https://github.com/utiasDSL/gym-pybullet-drones/issues/89
import getpass
sys.path.append(sys.path[0].replace("ros2/install/ros2_gym_pybullet_drones/lib/ros2_gym_pybullet_drones", ""))
if sys.platform == 'darwin': # macOS
    sys.path.append("/Users/"+os.getlogin()+"/opt/anaconda3/envs/drones/lib/python3.8/site-packages")  
elif sys.platform == 'linux': # Ubuntu
    sys.path.append("/home/"+getpass.getuser()+"/anaconda3/envs/drones/lib/python3.10/site-packages")  

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped


from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from custom_interface.msg import TrajMsg
class AviaryWrapper(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('aviary_wrapper')
        self.step_cb_count = 0
        self.get_action_cb_count = 0
        timer_freq_hz = 400
        timer_period_sec = 1/timer_freq_hz
        self.R = 0.3
        self.H = 0.5
        self.INIT_XYZS = np.array([[-2, -self.R, self.H]])
        self.INIT_RPYS = np.array([[0, 0, 0]])


        self.env = VisionAviary(drone_model=DroneModel.CF2X,
                           num_drones=1,
                           initial_xyzs=self.INIT_XYZS,
                           initial_rpys=self.INIT_RPYS,
                           physics=Physics.PYB,
                           neighbourhood_radius=np.inf,
                           freq=timer_freq_hz,
                           aggregate_phy_steps=1,
                           gui=True,
                           record=False,
                           obstacles=True,
                           user_debug_gui=False
                           )
        #### Initialize an action with the RPMs at hover ###########
        self.action = np.ones(4)*self.env.HOVER_RPM
        self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

        #### Declare publishing on 'obs' and create a timer to call 
        #### action_callback every timer_period_sec ################
        self.publisher_ = self.create_publisher(Float32MultiArray, 'obs', 1)
        self.dep_pub = self.create_publisher(Image,'depth_image',1)
        self.pcd_pub = self.create_publisher(PointCloud2,'pcd_gym_pybullet',2)
        self.rgb_pub = self.create_publisher(Image,'rgb_image',1)
        self.goal_pub = self.create_publisher(Path,'waypoints',1)
    #    self.seg_pub = self.create_publisher(Image,'segmentation_image',1)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(timer_period_sec, self.step_callback)
        #### Subscribe to topic 'waypoints' ###########################
        # self.wp_subs = self.create_subscription(Path, 'rrt_waypoints', self.get_waypoint_callback, 1)
        self.traj_subs = self.create_subscription(TrajMsg,'Trajectory', self.get_trajectory_callback, 1)
        self.pos = self.INIT_XYZS.flatten()
        self.waypoints = []
        self.prev_wp = []
        self.current_wp = None
        self.wp_idx = 0
        self.p_z = 0.1
        self.x_hist = []
        self.y_hist = []
        self.z_hist = []

        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_marker_array', 10)

        # Call a timer to publish the marker array at a lower rate than the main loop
        self.trajectory_timer = self.create_timer(1.0, self.publish_trajectory)
        self.is_goal_sent = False
        self.des_pos = self.INIT_XYZS.flatten()
        self.des_vel = np.zeros(3)

    
    def publish_trajectory(self):
        if len(self.x_hist) == 0:
            return  # No trajectory points to publish

        # Create a MarkerArray
        marker_array = MarkerArray()

        # Create a line strip marker to visualize the trajectory
        line_marker = Marker()
        line_marker.header.frame_id = "ground_link"  # Change this to the appropriate frame if needed
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        # Set marker properties
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.a = 1.0  # Alpha
        line_marker.color.r = 1.0  # Red
        line_marker.color.g = 0.0  # Green
        line_marker.color.b = 0.0  # Blue

        # Create trajectory points from x_hist, y_hist, z_hist
        for x, y, z in zip(self.x_hist, self.y_hist, self.z_hist):
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            line_marker.points.append(point)

        # Add the line marker to the marker array
        marker_array.markers.append(line_marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def broadcast_transform(self, x,y,z,qx,qy,qz,qw, frame_name, parent_name):
        # Assuming you have the drone's position and orientation as (x, y, z) and (qx, qy, qz, qw)
        drone_position = [x, y, z]  # Replace with actual drone position
        drone_orientation = [qx, qy, qz, qw]  # Replace with actual drone orientation

        # Create the transform message
        t = TransformStamped()

        # Set the timestamp and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_name  # Global reference frame
        t.child_frame_id = frame_name  # Frame representing the drone body

        # Set the translation and rotation
        t.transform.translation.x = drone_position[0]
        t.transform.translation.y = drone_position[1]
        t.transform.translation.z = drone_position[2]
        t.transform.rotation.x = drone_orientation[0]
        t.transform.rotation.y = drone_orientation[1]
        t.transform.rotation.z = drone_orientation[2]
        t.transform.rotation.w = drone_orientation[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    #### Step the env and publish drone0's state on topic 'obs'
    def step_callback(self):
        self.step_cb_count += 1

        obs, reward, done, info = self.env.step({"0": self.action})
        msg = Float32MultiArray()
        msg.data = obs["0"]["state"].tolist()
        self.pos = np.array(obs["0"]["state"].tolist()[0:3])
        
        self.x_hist.append(self.pos[0])
        self.y_hist.append(self.pos[1])
        self.z_hist.append(self.pos[2])
        
        self.broadcast_transform(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, "ground_link", "map")
        self.broadcast_transform(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], "base_link", "ground_link")

        self.publisher_.publish(msg)
        depth_image = obs["0"]["dep"]
        rgb_image = obs["0"]["rgb"]

        pcd = self.env._pcd_generation(depth_image)
        points = np.asarray(pcd.points)
        # points = self.env._pcd_generation_opencv(depth_image)
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Define fields for PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]   

        # Create PointCloud2 message
        pointcloud_msg = pc2.create_cloud(header, fields, points)

        # Publish the PointCloud2 message
        self.pcd_pub.publish(pointcloud_msg)


    #    seg_image = obs["0"]["seg"]
        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)

        if rgb_image.dtype != np.float32:
            rgb_image = rgb_image.astype(np.float32)
        
        rgb_image_3ch = rgb_image[:, :, :3]
        rgb_image_8uc3 = cv2.convertScaleAbs(rgb_image_3ch, alpha=(255.0/rgb_image_3ch.max()))
        gray_image = cv2.cvtColor(rgb_image_8uc3, cv2.COLOR_RGB2GRAY)
        # Convert the depth image to a ROS Image message
        ros_dep_image = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
        ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image_8uc3, encoding='rgb8')
        #ros_gray_image = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')

    #    ros_seg_image = self.bridge.cv2_to_imgmsg(seg_image, encoding='32FC1')
        self.dep_pub.publish(ros_dep_image)
        self.rgb_pub.publish(ros_rgb_image)

        action_em = {}
        action_em['0'], _, _ = self.ctrl.computeControlFromState(
                control_timestep=1/200,
                state=obs["0"]["state"],
                target_pos=self.des_pos,
                target_vel=self.des_vel,
                target_rpy=np.array([0, 0, 0]).reshape(3,1)
            )
        self.action = action_em['0'].tolist()
        
        goal = Path()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "ground_link"
        goal_pose.pose.position.x = 110.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.5
        goal.poses.append(goal_pose)
        self.goal_pub.publish(goal)


    #### Read the Trajectory (pos, vel, acc, jerk, snap)
    def get_trajectory_callback(self, msg):
        self.des_pos = np.array([[msg.position.x, msg.position.y, msg.position.z]]).flatten()
        print('des pos: ',self.des_pos)
        self.des_vel = np.array([[msg.velocity.x, msg.velocity.y, msg.velocity.z]]).flatten()
        print('des vel',self.des_vel)
############################################################
def main(args=None):
    rclpy.init(args=args)
    aviary_wrapper = AviaryWrapper()
    rclpy.spin(aviary_wrapper)

if __name__ == '__main__':
    main()
