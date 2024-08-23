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
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics


class HoverControl(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('hover_control')
        self.action_cb_count = 0
        self.get_obs_cb_count = 0
        #### Set the frequency used to publish actions #############
        timer_freq_hz = 50
        timer_period_sec = 1/timer_freq_hz
        #### Dummy CtrlAviary to obtain the HOVER_RPM constant #####
        self.env = CtrlAviary()
        #### Declare publishing on 'action' and create a timer to ##
        #### call action_callback every timer_period_sec ###########
        self.publisher_ = self.create_publisher(Float32MultiArray, 'action', 1)
        self.timer = self.create_timer(timer_period_sec, self.action_callback)
        #### Subscribe to topic 'obs' ##############################
        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)
        self.obs_subscription  # prevent unused variable warning
        self.R = 0.3
        self.H = 0.1
        self.INIT_XYZS = np.array([[self.R * np.cos(np.pi / 2), self.R * np.sin(np.pi / 2) - self.R, self.H]])
        self.INIT_RPYS = np.array([[0, 0, 0]])
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




    #### Publish random RPMs on topic 'action' #################
    def action_callback(self):
        self.action_cb_count += 1

        if self.current_obs is None:
            self.get_logger().info('no data received')
            return

        
        # Compute the control action
        #a = np.hstack([self.TARGET_POS[self.wp_counters, 0:2], self.INIT_XYZS[0, 2]])
        action = {}
        action['0'], _, _ = self.ctrl.computeControlFromState(
            control_timestep = 0.02,
            state=self.current_obs,  # Assuming msg.data contains the state of the drone
            target_pos=np.hstack([self.TARGET_POS[self.wp_counters, 0:2], self.INIT_XYZS[0, 2]]),
            #target_pos=np.array([[1],[0],[4]]),
            target_rpy=self.INIT_RPYS[0, :]
        )
        # Update waypoint counter
        self.wp_counters = self.wp_counters + 1 if self.wp_counters < (self.NUM_WP - 1) else 0
        
        # Publish the control action
        msg = Float32MultiArray()
        msg.data = action['0'].tolist()
        self.publisher_.publish(msg)
        if self.action_cb_count%10 == 0:
            self.get_logger().info('Publishing action 2: "%f" "%f" "%f" "%f"' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))

    #### Read the state of drone 0 from topic 'obs' ############
    def get_obs_callback(self, msg):
        self.get_obs_cb_count += 1
        self.current_obs = msg.data  # Store the observation
        if self.get_obs_cb_count%10 == 0:
            self.get_logger().info('I received obs: "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f"' \
                                   % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                                      msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9],
                                      msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14],
                                      msg.data[15], msg.data[16], msg.data[17], msg.data[18], msg.data[19]
                                      )
                                   )

############################################################
def main(args=None):
    rclpy.init(args=args)
    hover_control = HoverControl()
    rclpy.spin(hover_control)

if __name__ == '__main__':
    main()
