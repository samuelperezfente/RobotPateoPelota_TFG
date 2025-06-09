#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class BallFollowerWithDepth(Node):
    def __init__(self):
        super().__init__('ball_follower_with_depth')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_detection = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self.sub_lidar = self.create_subscription(PointCloud2, '/point_cloud2', self.lidar_callback, 10)

        self.image_width = 1280
        self.tolerance = 5
        self.kp = 0.002

        self.target_found = False
        self.target_centered = False
        self.latest_point_cloud = None

        self.timer = self.create_timer(0.1, self.search_behavior)

    def lidar_callback(self, msg):
        self.latest_point_cloud = msg

    def detection_callback(self, msg: Yolov8Inference):
        if self.target_centered or not msg.yolov8_inference:
            return

        for det in msg.yolov8_inference:
            if det.class_name.lower() == "ball":
                x_center = (det.left + det.right) / 2
                error = x_center - (self.image_width / 2)

                if abs(error) < self.tolerance:
                    self.get_logger().info("Ball centered.")
                    self.target_centered = True
                    self.get_depth_from_lidar()
                else:
                    angular_z = -self.kp * error
                    twist = Twist()
                    twist.angular.z = angular_z
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"Tracking ball: error={error}, angular_z={angular_z:.3f}")
                    self.target_found = True
                return

    def get_depth_from_lidar(self):
        if self.latest_point_cloud is None:
            self.get_logger().warn("No point cloud data available.")
            return

        gen = pc2.read_points(self.latest_point_cloud, skip_nans=True, field_names=("x", "y", "z"))
        min_distance = float('inf')
        closest_point = None

        for pt in gen:
            distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_point = pt

        if closest_point:
            x, y, z = closest_point
            self.get_logger().info(f"Estimated object position from LiDAR: x={x:.2f}, y={y:.2f}, z={z:.2f} (distance: {min_distance:.2f} m)")
        else:
            self.get_logger().warn("No matching LiDAR point found.")

    def search_behavior(self):
        if not self.target_found:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)
            self.get_logger().info("Searching for ball...")

def main(args=None):
    rclpy.init(args=args)
    node = BallFollowerWithDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
