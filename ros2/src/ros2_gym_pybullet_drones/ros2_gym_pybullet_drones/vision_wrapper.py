"""ROS2 Python wrapper node for class CtrlAviary.

It creates an environment CtrlAviary and continually calls CtrlAviary.step().
It publishes on topic 'obs' and reads from topic 'action'.
"""
import sys, os  # See: https://github.com/utiasDSL/gym-pybullet-drones/issues/89
import getpass
sys.path.append(sys.path[0].replace("ros2/install/ros2_gym_pybullet_drones/lib/ros2_gym_pybullet_drones", ""))
if sys.platform == 'darwin': # macOS
    sys.path.append("/Users/"+os.getlogin()+"/opt/anaconda3/envs/drones/lib/python3.8/site-packages")  
elif sys.platform == 'linux': # Ubuntu
    sys.path.append("/home/"+getpass.getuser()+"/anaconda3/envs/drones/lib/python3.10/site-packages")  

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.VisionAviary import VisionAviary

class AviaryWrapper(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('aviary_wrapper')
        self.step_cb_count = 0
        self.get_action_cb_count = 0
        timer_freq_hz = 240
        timer_period_sec = 1/timer_freq_hz
        self.env = VisionAviary(drone_model=DroneModel.CF2X,
                           num_drones=1,
                           initial_xyzs=None,
                           initial_rpys=None,
                           physics=Physics.PYB,
                           neighbourhood_radius=np.inf,
                           freq=timer_freq_hz,
                           aggregate_phy_steps=1,
                           gui=False,
                           record=False,
                           obstacles=True,
                           user_debug_gui=False
                           )
        #### Initialize an action with the RPMs at hover ###########
        self.action = np.ones(4)*self.env.HOVER_RPM
        #### Declare publishing on 'obs' and create a timer to call 
        #### action_callback every timer_period_sec ################
        self.publisher_ = self.create_publisher(Float32MultiArray, 'obs', 1)
        self.dep_pub = self.create_publisher(Image,'depth_image',1)
        self.pcd_pub = self.create_publisher(PointCloud2,'pcd_gym_pybullet',1)
    #    self.rgb_pub = self.create_publisher(Image,'rgb_image',1)
    #    self.seg_pub = self.create_publisher(Image,'segmentation_image',1)

        self.bridge = CvBridge()

        self.timer = self.create_timer(timer_period_sec, self.step_callback)
        #### Subscribe to topic 'action' ###########################
        self.action_subscription = self.create_subscription(Float32MultiArray, 'action', self.get_action_callback, 1)
        self.action_subscription  # prevent unused variable warning

    #### Step the env and publish drone0's state on topic 'obs'
    def step_callback(self):
        self.step_cb_count += 1
        obs, reward, done, info = self.env.step({"0": self.action})
        msg = Float32MultiArray()
        msg.data = obs["0"]["state"].tolist()
        self.publisher_.publish(msg)
        depth_image = obs["0"]["dep"]
        pcd = self.env._occupancy_generation(depth_image)
        points = np.asarray(pcd.points)

        # Create header
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
        pointcloud_msg = pc2.create_cloud(header, fields, points)

        # Publish the PointCloud2 message
        self.pcd_pub.publish(pointcloud_msg)

    #    rgb_image = obs["0"]["rgb"]
    #    seg_image = obs["0"]["seg"]
        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)

        # Convert the depth image to a ROS Image message
        ros_dep_image = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
    #    ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding='32FC1')
    #    ros_seg_image = self.bridge.cv2_to_imgmsg(seg_image, encoding='32FC1')
        self.dep_pub.publish(ros_dep_image)
    #    self.rgb_pub.publish(ros_rgb_image)
    #    self.seg_pub.publish(ros_seg_image)
        if self.step_cb_count%10 == 0:
            self.get_logger().info('Publishing obs: "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f"' \
                                   % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                                      msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9],
                                      msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14],
                                      msg.data[15], msg.data[16], msg.data[17], msg.data[18], msg.data[19]
                                      )
                                   )

    #### Read the action to apply to the env from topic 'action'
    def get_action_callback(self, msg):
        self.get_action_cb_count += 1
        self.action = np.array([msg.data[0], msg.data[1], msg.data[2], msg.data[3]])
        if self.get_action_cb_count%10 == 0:
            self.get_logger().info('I received action: "%f" "%f" "%f" "%f"' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))

############################################################
def main(args=None):
    rclpy.init(args=args)
    aviary_wrapper = AviaryWrapper()
    rclpy.spin(aviary_wrapper)

if __name__ == '__main__':
    main()
