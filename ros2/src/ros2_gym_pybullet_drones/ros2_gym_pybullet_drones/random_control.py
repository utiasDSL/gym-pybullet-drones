"""ROS2 Python node for random control.

It subscribes to aviary_wrapper's 'obs' topic (but ignores it).
It publishes random RPMs on topic 'action'.
"""
import rclpy
import random
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

class RandomControl(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('random_control')
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

    #### Publish random RPMs on topic 'action' #################
    def action_callback(self):
        self.action_cb_count += 1
        random_rpm13 = random.uniform(.9, 1.1)*self.env.HOVER_RPM
        random_rpm24 = random.uniform(.9, 1.1)*self.env.HOVER_RPM
        msg = Float32MultiArray()
        msg.data = [random_rpm13,random_rpm24,random_rpm13,random_rpm24]
        self.publisher_.publish(msg)
        if self.action_cb_count%10 == 0:
            self.get_logger().info('Publishing action: "%f" "%f" "%f" "%f"' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))

    #### Read the state of drone 0 from topic 'obs' ############
    def get_obs_callback(self, msg):
        self.get_obs_cb_count += 1
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
    random_control = RandomControl()
    rclpy.spin(random_control)

if __name__ == '__main__':
    main()
