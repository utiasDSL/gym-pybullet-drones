import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary


######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
class AviaryWrapper(Node):

    #### WIP ###########################################################################################
    def __init__(self):
        super().__init__('aviary_wrapper')
        self.i = 0
        timer_freq_hz = 240
        self.env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, neighbourhood_radius=np.inf, initial_xyzs=None, initial_rpys=None,
                                physics=Physics.PYB, freq=timer_freq_hz, aggregate_phy_steps=1, gui=True, record=False, obstacles=False, user_debug_gui=True)
        self.action = np.ones(4)*self.env.HOVER_RPM
        #
        #### step and write observations 
        timer_period_sec = 1/timer_freq_hz
        self.publisher_ = self.create_publisher(Float32MultiArray, 'obs', 10)
        self.timer = self.create_timer(timer_period_sec, self.step_callback)
        #
        #### read actions
        self.action_subscription = self.create_subscription(Float32MultiArray, 'action', self.get_action_callback, 10)
        self.action_subscription  # prevent unused variable warning

    #### WIP ###########################################################################################
    def step_callback(self):
        self.i += 1
        obs, reward, done, info = self.env.step({"0": self.action})
        #
        msg = Float32MultiArray(); msg.data = obs["0"]["state"].tolist()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing obs: "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f" "%f"' \
                                                            % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                                                            msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9],
                                                            msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14],
                                                            msg.data[15], msg.data[16], msg.data[17], msg.data[18], msg.data[19]))
        

    #### WIP ###########################################################################################
    def get_action_callback(self, msg):
        self.get_logger().info('I received action: "%f" "%f" "%f" "%f"' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))
        self.action = np.array([msg.data[0], msg.data[1], msg.data[2], msg.data[3]])


######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    aviary_wrapper = AviaryWrapper()
    rclpy.spin(aviary_wrapper)

if __name__ == '__main__':
    main()
