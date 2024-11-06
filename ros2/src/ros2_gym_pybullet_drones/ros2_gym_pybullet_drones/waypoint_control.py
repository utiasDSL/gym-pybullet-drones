"""ROS2 Python node for random control.

It subscribes to aviary_wrapper's 'obs' topic (but ignores it).
It publishes random RPMs on topic 'action'.
"""
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
import math
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class waypointControl(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('waypoint_control')
        self.action_cb_count = 0
        self.get_obs_cb_count = 0
        #### Set the frequency used to publish actions #############
        timer_freq_hz = 400
        self.timer_period_sec = 1/timer_freq_hz
        #### Dummy CtrlAviary to obtain the waypoint_RPM constant #####
        self.env = CtrlAviary()
        #### Declare publishing on 'action' and create a timer to ##
        #### Subscribe to topic 'obs' ##############################
        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)
        self.waypoint_subscription = self.create_subscription(Path, 'rrt_waypoints', self.get_waypoint_callback, 1)
        #### call action_callback every timer_period_sec ###########
        self.publisher_ = self.create_publisher(Float32MultiArray, 'action', 1)
        self.timer = self.create_timer(self.timer_period_sec, self.action_callback)
        self.obs_subscription  # prevent unused variable warning
        self.waypoint_subscription
        self.R = 0.3
        self.H = 0.1
        self.INIT_XYZS = np.array([[0, -self.R, self.H]])
        self.current_wp = self.INIT_XYZS.flatten()
        self.INIT_RPYS = np.array([[0, 0, 0]])
        self.waypoints = []
        self.wp_idx = 0
        PERIOD = 10
        self.control_freq_hz = 50
        self.NUM_WP = self.control_freq_hz * PERIOD
        self.TARGET_POS = np.zeros((self.NUM_WP, 3))
        for i in range(self.NUM_WP):
            self.TARGET_POS[i, :] = self.R * np.cos((i / self.NUM_WP) * (2 * np.pi) + np.pi / 2) + self.INIT_XYZS[0, 0], \
                            self.R * np.sin((i / self.NUM_WP) * (2 * np.pi) + np.pi / 2) - self.R + self.INIT_XYZS[0, 1], \
                            0
        self.wp_counters = 0
        self.current_obs = None
        SIM_FREQ = 240
        self.CTRL_EVERY_N_STEPS = int(np.floor(SIM_FREQ/self.control_freq_hz))
        self.TIMESTEP = 1./SIM_FREQ
        self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.prev_wp = []
        self.p_z = 0.5





    #### Publish random RPMs on topic 'action' #################
    def action_callback(self):
        self.action_cb_count += 1

        if self.current_obs is None:
            msg = Float32MultiArray()
            msg.data = [self.env.HOVER_RPM, self.env.HOVER_RPM, self.env.HOVER_RPM, self.env.HOVER_RPM]
            print("no obs received")
            self.publisher_.publish(msg)
            return
        
        if self.waypoints == [] or self.waypoints is None:
            print("no waypoint received")
            action_em = {}
            action_em['0'], _, _ = self.ctrl.computeControlFromState(
                control_timestep=self.timer_period_sec,
                state=self.current_obs,
                target_pos=np.array(self.current_obs[0:3]).flatten(),
                target_rpy=np.array([0, 0, 0]).reshape(3,1)
            )
            msg = Float32MultiArray()
            msg.data = action_em['0'].tolist()
            self.publisher_.publish(msg)
            return


        
        current_pos = self.current_obs[0:3]
        print(np.array(self.current_wp).shape)
        distance_to_wp = np.linalg.norm(np.array(current_pos) - np.array(self.current_wp))

        self.get_logger().info('current waypoint: "%f", "%f", "%f", distance = "%f"' % (self.current_wp[0], self.current_wp[1], self.current_wp[2], distance_to_wp))
        target_diff = np.array(self.current_wp) - np.array(current_pos)
        target_yaw = math.atan2(target_diff[1], target_diff[0])
        norm_xy = np.linalg.norm(target_diff[0:2])
        target_velocity = np.array([target_diff[0]/norm_xy, target_diff[1]/norm_xy, target_diff[2]*self.p_z]).flatten()
        # Compute the control action
        action = {}
        print('################## wp index ################', self.wp_idx)
        action['0'], _, _ = self.ctrl.computeControlFromState(
            control_timestep = self.timer_period_sec,
            state=self.current_obs,  # Assuming msg.data contains the state of the drone
            target_pos = self.current_wp,
            target_vel=target_velocity,
            target_rpy=np.array([[0, 0, np.pi]]).reshape(3, 1)
        )
        
        if distance_to_wp < 0.3:  # Threshold to consider waypoint reached
            print("waypoint change triggered")
            if self.wp_idx < len(self.waypoints) - 1:
                self.wp_idx += 1
                print('waypoint index: ', self.wp_idx, len(self.waypoints))
                self.current_wp = self.waypoints[self.wp_idx]
            else:
                self.current_wp = np.array(self.current_obs[0:3])

        # Publish the control action
        msg = Float32MultiArray()
        msg.data = action['0'].tolist()
        self.publisher_.publish(msg)
       
    #### Read the state of drone 0 from topic 'obs' ############
    def get_obs_callback(self, msg):
        self.get_obs_cb_count += 1
        self.current_obs = msg.data  # Store the observation
        
    def get_waypoint_callback(self, msg):
        """Callback to handle updates to the path."""
        # Update waypoints based on the incoming path message
        self.waypoints = [
            np.asarray([-1*pose.pose.position.x, -1*pose.pose.position.y, pose.pose.position.z])
            for pose in msg.poses
        ]
        # Reset waypoint index if new path received
        if self.waypoints:
            if not np.array_equal(self.waypoints, self.prev_wp):
                self.prev_wp = self.waypoints
                mind = 1000000000
                for i in range(0, len(self.waypoints)):
                    dist = np.linalg.norm(np.array(self.current_obs[0:3]) - np.array(self.waypoints[i]))
                    if dist < mind:
                        mind = dist
                        self.current_wp = self.waypoints[i]
                        self.wp_idx = i

        else:
            self.current_wp = np.array([0.0, 10.0, 0.5]).flatten()
            

############################################################
def main(args=None):
    rclpy.init(args=args)
    waypoint_control = waypointControl()
    rclpy.spin(waypoint_control)

if __name__ == '__main__':
    main()
