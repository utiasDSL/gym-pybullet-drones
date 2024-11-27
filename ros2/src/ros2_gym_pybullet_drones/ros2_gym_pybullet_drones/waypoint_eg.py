import sys, os  # See: https://github.com/utiasDSL/gym-pybullet-drones/issues/89
import getpass
sys.path.append(sys.path[0].replace("ros2/install/ros2_gym_pybullet_drones/lib/ros2_gym_pybullet_drones", ""))
if sys.platform == 'darwin': # macOS
    sys.path.append("/Users/"+os.getlogin()+"/opt/anaconda3/envs/drones/lib/python3.8/site-packages")  
elif sys.platform == 'linux': # Ubuntu
    sys.path.append("/home/"+getpass.getuser()+"/anaconda3/envs/drones/lib/python3.10/site-packages")  

import rclpy
import random
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

class waypoint_pub(Node):
    def __init__(self):
        super().__init__('waypoint_pub')  # Initialize the ROS 2 node
        self.wp1 = np.array([
            [-2.0, -0.3, 1.5],
            [2.0, 0.0, 0.5],
            [3.0, 0.0, 0.5],
            [4.0, 0.0, 0.5],
            [5.0, 0.0, 0.5]
        ])

        self.wp2 = np.array([
            [3.5, 0.0, 0.5],
            [3.5, 1.0, 0.5],
            [4.5, 2.0, 0.5],
            [5.4, 3.0, 0.5],
            [7.0, 4.0, 0.5],
            [8.5, 5.0, 0.5]
        ])

        self.wp3 = np.array([
            [7.2, 4.0, 0.5],
            [6.2, 4.0, 0.5],
            [5.2, 4.0, 0.5],
            [4.2, 3.0, 0.5],
            [3.2, 2.0, 0.5],
            [2.2, 1.0, 0.5],
            [0.0, 0.0, 0.5]
        ])

        self.wp4 = np.array([
            [0.0, 0.0, 0.5]
        ])
        FREQ = 100
        self.timer_period_sec = 1/FREQ
        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)
        self.path_publisher = self.create_publisher(Path, 'rrt_waypoints', 1)
        self.timer = self.create_timer(self.timer_period_sec, self.action_callback)
        self.obs = None
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.wp1_finish = False
        self.wp2_finish = False
        self.wp3_finish = False

    def action_callback(self):
        if self.obs is None:
            self.get_logger().info('waiting for observation')
            return
        
        self.current_pos = self.obs[0:3]
        self.current_pos[0] = self.current_pos[0]
        self.current_pos[1] = self.current_pos[1]

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "ground_link"

        if not self.wp1_finish:
            for point in self.wp1:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.path_publisher.publish(path_msg)
            dist_from_anchor1 = np.linalg.norm(self.wp1[2] - self.current_pos)

            self.get_logger().info('Publishing wp1, dist_from_anchor1 = "%f"'%(dist_from_anchor1))
            if dist_from_anchor1 < 0.1:
                self.wp1_finish = True
        
        else:
            if not self.wp2_finish:
                for point in self.wp2:
                    pose = PoseStamped()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = point[2]
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
                
                self.path_publisher.publish(path_msg)
                self.get_logger().info('Publishing wp2')
                dist_from_anchor2 = np.linalg.norm(self.wp2[4] - self.current_pos)
                if dist_from_anchor2 < 0.1:
                    self.wp2_finish = True
            
            elif not self.wp3_finish:
                for point in self.wp3:
                    pose = PoseStamped()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = point[2]
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
    
                self.get_logger().info('Publishing wp3')
                self.path_publisher.publish(path_msg)
                dist_from_anchor3 = np.linalg.norm(self.wp3[6] - self.current_pos)
                if dist_from_anchor3 < 0.1:
                    self.wp3_finish = True
            else:
                for point in self.wp4:
                    pose = PoseStamped()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = point[2]
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
                
                self.get_logger().info('Publishing wp4')
                self.path_publisher.publish(path_msg)



    def get_obs_callback(self, msg):
        self.obs = msg.data
        self.get_logger().info('observation received')

def main(args=None):
    rclpy.init(args=args)
    waypoint = waypoint_pub()
    rclpy.spin(waypoint)

if __name__ == '__main__':
    main()


        