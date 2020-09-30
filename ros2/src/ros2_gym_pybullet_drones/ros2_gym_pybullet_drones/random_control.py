import rclpy
from rclpy.node import Node
from std_msgs.msg import String

######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
class RandomControl(Node):

    #### WIP ###########################################################################################
    def __init__(self):
        super().__init__('random_control')
        #
        #### read observations
        self.obs_subscription = self.create_subscription(String, 'obs', self.get_obs_callback, 10)
        self.obs_subscription  # prevent unused variable warning
        #
        #### write action 
        self.publisher_ = self.create_publisher(String, 'action', 10)
        timer_freq_hz = 2; timer_period_sec = 1/timer_freq_hz
        self.timer = self.create_timer(timer_period_sec, self.action_callback)
        self.i = 0

    #### WIP ###########################################################################################
    def action_callback(self):
        msg = String(); msg.data = 'Random action: %d)' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing action: "%s"' % msg.data)
        self.i += 1

    #### WIP ###########################################################################################
    def get_obs_callback(self, msg):
        self.get_logger().info('I observe: "%s"' % msg.data)


######################################################################################################################################################
#### WIP #############################################################################################################################################
######################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    random_control = RandomControl()
    rclpy.spin(random_control)


if __name__ == '__main__':
    main()
