"""ROS2 Python wrapper node for class CtrlAviary.

It creates an environment CtrlAviary and continually calls CtrlAviary.step().
It publishes on topic 'obs' and reads from topic 'action'.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

class AviaryWrapper(Node):

    #### Initialize the node ###################################
    def __init__(self):
        super().__init__('aviary_wrapper')
        self.step_cb_count = 0
        self.get_action_cb_count = 0
        timer_freq_hz = 240
        timer_period_sec = 1/timer_freq_hz
        #### Create the CtrlAviary environment wrapped by the node #
        self.env = CtrlAviary(drone_model=DroneModel.CF2X,
                              num_drones=1,
                              neighbourhood_radius=np.inf,
                              initial_xyzs=None,
                              initial_rpys=None,
                              physics=Physics.PYB,
                              freq=timer_freq_hz,
                              aggregate_phy_steps=1,
                              gui=True,
                              record=False,
                              obstacles=False,
                              user_debug_gui=True
                              )
        #### Initialize an action with the RPMs at hover ###########
        self.action = np.ones(4)*self.env.HOVER_RPM
        #### Declare publishing on 'obs' and create a timer to call 
        #### action_callback every timer_period_sec ################
        self.publisher_ = self.create_publisher(Float32MultiArray, 'obs', 1)
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
