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
from geometry_msgs.msg import Point, Vector3
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from custom_interface_gym.msg import TrajMsg
from ros2_gym_pybullet_drones.trajectory import Trajectory
import time

class trajectory_pub(Node):
    def __init__(self):
        super().__init__('trajectory_pub')

        self._planning_rate = 10

        self.wp_subs = self.create_subscription(Path, 'rrt_waypoints', self.get_waypoint_callback, 1)
        self.obs_subscription = self.create_subscription(Float32MultiArray, 'obs', self.get_obs_callback, 1)

        self.traj_publisher = self.create_publisher(TrajMsg, 'rrt_command',1)
        self._planning_timer = self.create_timer(1.0 / self._planning_rate, self.step)
        trajSelect = np.zeros(4)

        # Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel,            3: xyz_pos with geometric)
        ctrlType = "xyz_pos"   
        # Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,    
        #                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
        #                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
        #                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
        #                                 12: pos_waypoint_arrived     13: pos_waypoint_arrived_wait
        trajSelect[0] = 3       
        # Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
        trajSelect[1] = 3           
        # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
        trajSelect[2] = 1
        v_avg = 0.3 # m/s
        self.traj = Trajectory(ctrlType, trajSelect, v_avg)
        self._psi = 0
        traj_msg = TrajMsg()

        self.wp_list = []
        self._start_pos = np.zeros(3)
        self.psi = 0
        self.replan = True
        self.t_assign = time.time()


    def get_waypoint_callback(self, msg):
        wp_list = [
            np.asarray([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            for pose in msg.poses
        ]
        wp_list = [np.resize(wp, (3,)) for wp in wp_list]
        if len(wp_list) != len(self.wp_list) or not all(np.array_equal(wp, self.wp_list[i]) for i, wp in enumerate(wp_list)):
            self.wp_list = wp_list
            self.traj.set_coefficients(wp_list)
            self.t_assign = time.time()
        

        
    def get_obs_callback(self, msg):
        self._start_pos[0] = msg.data[0]
        self._start_pos[1] = msg.data[1]
        self._start_pos[2] = msg.data[2]
        self._psi = msg.data[9]


    def step(self):
        
        del_t = time.time()-self.t_assign
        sDes =  self.traj.desiredState(del_t, 0.01, self._start_pos, self._psi)
        print("step function called, del_t = ",del_t)
        traj_msg = TrajMsg()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "ground_link"
        pos_sp    = sDes[0:3]
        vel_sp    = sDes[3:6]
        acc_sp    = sDes[6:9]
        eul_sp    = sDes[12:15]
        jerk_sp   = sDes[19:22]
        snap_sp   = sDes[22:25]
        
        traj_msg.position = Point(x=pos_sp[0], y=pos_sp[1], z=pos_sp[2])
        traj_msg.velocity = Vector3(x=vel_sp[0], y=vel_sp[1], z=vel_sp[2])
        traj_msg.acceleration = Vector3(x=acc_sp[0], y=acc_sp[1], z=acc_sp[2])
        traj_msg.jerk = Vector3(x=jerk_sp[0], y=jerk_sp[1], z=jerk_sp[2])
        traj_msg.snap = Vector3(x=snap_sp[0], y=snap_sp[1], z=snap_sp[2])
        traj_msg.yaw = eul_sp[2]
        print("trajectory published: des_pos: ", pos_sp[0], pos_sp[1], pos_sp[2])
        self.traj_publisher.publish(traj_msg)
    
def main(args=None):
    rclpy.init(args=args)
    trajectory = trajectory_pub()
    rclpy.spin(trajectory)


if __name__ == '__main__':
    main()

