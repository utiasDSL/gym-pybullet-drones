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

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

class RandomControl(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('random_control')
        self.action_cb_count = 0
        self.get_obs_cb_count = 0
        #### Set the frequency used to publish actions #############
        timer_freq_hz = 200
        self.timer_period_sec = 1/timer_freq_hz
        #### Dummy CtrlAviary to obtain the HOVER_RPM constant #####
        self.env = CtrlAviary()
        #### Declare publishing on 'action' and create a timer to ##
        #### call action_callback every timer_period_sec ###########
        self.publisher_ = self.create_publisher(Float32MultiArray, 'action', 1)
        self.timer = self.create_timer(self.timer_period_sec, self.action_callback)
        #### Subscribe to topic 'obs' ##############################
        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)
        self.obs_subscription  # prevent unused variable warning
        self.current_obs = None
        self.R = 0.3
        self.H = 0.1
        self.INIT_XYZS = np.array([[0, -self.R, self.H]])
        self.INIT_RPYS = np.array([[0, 0, np.pi/2]])
        self.waypoints = np.array([
                [0, 0, 0.1],   # Waypoint 1
                [1, 1, 0.1],   # Waypoint 2
                [2, 1, 0.1],   # Waypoint 3
                [3, 1, 0.1],  # Waypoint 4
                [4, 1, 0.1],   # Return to Waypoint 1
                [4, 2, 0.1],
                [4, 3, 0.1],
                [4, 4, 0.1],
                [4, 5, 0.1],
            ])
        
        self.NUM_WP = 9
        self.wp_counters = 0  # Start from the first waypoint

        self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)


    #### Publish random RPMs on topic 'action' #################
    def action_callback(self):
        self.action_cb_count += 1
        if self.current_obs is None:
            msg = Float32MultiArray()
            msg.data = [self.env.HOVER_RPM, self.env.HOVER_RPM, self.env.HOVER_RPM, self.env.HOVER_RPM]
            self.publisher_.publish(msg)
            return
        #random_rpm13 = self.env.HOVER_RPM
        #random_rpm24 = self.env.HOVER_RPM
        msg = Float32MultiArray()
        action = {}
        action['0'], _, _ = self.ctrl.computeControlFromState(
            control_timestep = self.timer_period_sec,
            state=self.current_obs,  # Assuming msg.data contains the state of the drone
            target_pos=self.current_wp,
            #target_pos=np.array([1.0,0.0,1.0]).flatten(),
            target_rpy=self.INIT_RPYS[0, :]
        )
        msg.data = action['0'].tolist()
        self.publisher_.publish(msg)
        if self.action_cb_count%10 == 0:
            self.get_logger().info('Publishing action: "%f" "%f" "%f" "%f"' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))

    #### Read the state of drone 0 from topic 'obs' ############
    def get_obs_callback(self, msg):
        self.get_obs_cb_count += 1
        self.current_obs = msg.data
        
        # if self.get_obs_cb_count%10 == 0:
        #     self.get_logger().info('I received obs: "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f"' \
        #                            % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
        #                               msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9],
        #                               msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14],
        #                               msg.data[15], msg.data[16], msg.data[17], msg.data[18], msg.data[19]
        #                               )
        #                            )

############################################################
def main(args=None):
    rclpy.init(args=args)
    random_control = RandomControl()
    rclpy.spin(random_control)

if __name__ == '__main__':
    main()
