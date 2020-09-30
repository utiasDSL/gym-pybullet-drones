import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary


######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
class AviaryWrapper(Node):

    #### WIP ###########################################################################################
    def __init__(self):
        super().__init__('aviary_wrapper')
        #
        self.env = CtrlAviary()
        #
        #### write observations 
        self.publisher_ = self.create_publisher(String, 'obs', 10)
        timer_freq_hz = 2; timer_period_sec = 1/timer_freq_hz
        self.timer = self.create_timer(timer_period_sec, self.step_callback)
        self.i = 0
        #
        #### read actions
        self.action_subscription = self.create_subscription(String, 'action', self.get_action_callback, 10)
        self.action_subscription  # prevent unused variable warning

    #### WIP ###########################################################################################
    def step_callback(self):
        #
        self.env.step({"0": np.array([0,0,0,0])})
        #
        msg = String(); msg.data = '(A ROS2 Python wrapper for gym-pybullet-drones: %d)' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing obs: "%s"' % msg.data)
        self.i += 1

    #### WIP ###########################################################################################
    def get_action_callback(self, msg):
        self.get_logger().info('I receive action: "%s"' % msg.data)


######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    aviary_wrapper = AviaryWrapper()
    rclpy.spin(aviary_wrapper)

if __name__ == '__main__':
    main()
