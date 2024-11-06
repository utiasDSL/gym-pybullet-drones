import sys, os  # See: https://github.com/utiasDSL/gym-pybullet_drones/issues/89
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
from gym_pybullet_drones.utils.enums import DroneModel
from ros2_gym_pybullet_drones.srv import UavControl  # Import your service

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class waypointControl(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('waypoint_control')

        #### Set the frequency used to publish actions #############
        timer_freq_hz = 400
        self.timer_period_sec = 1 / timer_freq_hz

        #### Dummy CtrlAviary to obtain the waypoint_RPM constant #####
        self.env = CtrlAviary()

        #### Declare the service ###################################
        self.srv = self.create_service(UavControl, 'uav_control', self.uav_control_callback)

        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)
        self.waypoint_subscription = self.create_subscription(Path, 'rrt_waypoints', self.get_waypoint_callback, 1)

        #### Initialize control variables ####
        self.R = 0.3
        self.H = 0.1
        self.INIT_XYZS = np.array([[0, -self.R, self.H]])
        self.current_wp = self.INIT_XYZS.flatten()
        self.waypoints = []
        self.wp_idx = 0
        self.current_obs = None
        self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
        self.prev_wp = []
        self.p_z = 0.5

        SIM_FREQ = 240
        self.CTRL_EVERY_N_STEPS = int(np.floor(SIM_FREQ / 50))  # 50 Hz control
        self.TIMESTEP = 1. / SIM_FREQ
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    #### Service callback to handle control requests #############
    def uav_control_callback(self, request, response):
        """Handle incoming service requests for UAV control."""
        
        # Get the control inputs: [roll, pitch, yaw, throttle] from the request
        roll, pitch, yaw, throttle = request.request
        
        # Assuming current_obs holds the state of the drone
        if self.current_obs is None:
            # No observation yet, respond with default state
            self.get_logger().info("No observation received yet.")
            response.response = [0.0] * 7  # Return default zero state
            return response

        # Process control action based on current waypoints and state
        current_pos = self.current_obs[0:3]
        distance_to_wp = np.linalg.norm(np.array(current_pos) - np.array(self.current_wp))

        # Compute control from the current state and target waypoint
        action, _, _ = self.ctrl.computeControlFromState(
            control_timestep=self.timer_period_sec,
            state=self.current_obs,
            target_pos=self.current_wp,
            target_rpy=np.array([[roll, pitch, yaw]]).reshape(3, 1)
        )

        if distance_to_wp < 0.3:  # Threshold to consider waypoint reached
            self.get_logger().info("Reached waypoint, updating...")
            if self.wp_idx < len(self.waypoints) - 1:
                self.wp_idx += 1
                self.current_wp = self.waypoints[self.wp_idx]
            else:
                self.current_wp = np.array(self.current_obs[0:3])

        # Prepare the response with the current UAV state
        response.response = list(self.current_obs[0:3]) + list(self.current_obs[3:7])
        
        return response

    #### Read the state of drone 0 from topic 'obs' ############
    def get_obs_callback(self, msg):
        self.current_obs = msg.data  # Store the observation

    #### Update the waypoint list based on new path ###########
    def get_waypoint_callback(self, msg):
        self.waypoints = [
            np.asarray([-pose.pose.position.x, -pose.pose.position.y, pose.pose.position.z])
            for pose in msg.poses
        ]
        if self.waypoints:
            self.current_wp = self.waypoints[0]  # Set the first waypoint as the target
            self.wp_idx = 0

############################################################
def main(args=None):
    rclpy.init(args=args)
    waypoint_control = waypointControl()
    rclpy.spin(waypoint_control)

if __name__ == '__main__':
    main()
