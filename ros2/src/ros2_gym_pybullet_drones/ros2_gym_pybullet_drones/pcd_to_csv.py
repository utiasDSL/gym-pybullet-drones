#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import csv
import numpy as np
from sensor_msgs.msg import PointCloud2
import time
import os
from datetime import datetime

import tf2_ros

class PointCloudToCSV(Node):

    def __init__(self):
        super().__init__('point_cloud_to_csv')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcd_gym_pybullet',  # Replace with your actual topic
            self.point_cloud_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.counter = 0  # Counter to track the number of point clouds saved
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base_path = '/home/astik/pcd_outputs/'
        folder_name = current_time
        self.full_path = os.path.join(base_path, folder_name)

        # Check whether the specified path exists or not
        isExist = os.path.exists(self.full_path)
        if not isExist:
            os.makedirs(self.full_path)

    def point_cloud_callback(self, point_cloud):
        # Convert PointCloud2 to an array
        points_list = []

        for point in pc2.read_points(point_cloud, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])  # X, Y, Z coordinates

        # Convert to numpy array
        points_array = np.array(points_list)

        # Save to CSV with a unique name
        file_name = os.path.join(self.full_path, f'pcd_{self.counter:04d}.csv')
        with open(file_name, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y", "Z"])  # Write the header
            writer.writerows(points_array)

        self.get_logger().info(f'Point cloud saved to {file_name}')

        # Increment the counter for the next file
        self.counter += 1

        # Ensure the callback is invoked at 2 Hz
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_to_csv_node = PointCloudToCSV()
    rclpy.spin(point_cloud_to_csv_node)

    # Destroy the node explicitly
    point_cloud_to_csv_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
