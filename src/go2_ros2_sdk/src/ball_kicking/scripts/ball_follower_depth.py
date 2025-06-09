#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from object_msgs.msg import ObjectsPosition

import numpy as np
import tf2_ros

# ... [imports sin cambios]

class BallFollowerWithFilteredLidar(Node):
    def __init__(self):
        super().__init__('ball_follower_with_filtered_lidar')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_array_pub = self.create_publisher(ObjectsPosition, '/ball_and_goal_positions', 10)
        self.sub_detection = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self.sub_lidar = self.create_subscription(PointCloud2, '/filtered_lidar', self.lidar_callback, qos)

        self.goal_pose = Pose()
        self.goal_pose.position.x = 6.045
        self.goal_pose.position.y = 5.851
        self.goal_pose.position.z = 0.394
        self.goal_pose.orientation.w = 1.0
        
        self.image_width = 1280
        self.tolerance = 5
        self.kp = 0.002

        self.target_found = False
        self.target_centered = False
        self.latest_filtered_lidar = None
        self.last_bbox = None
        self.last_ball_pose = None 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.loop_behavior)  

    def lidar_callback(self, msg):
        self.latest_filtered_lidar = msg

    def detection_callback(self, msg: Yolov8Inference):
        if self.target_centered or not msg.yolov8_inference:
            return

        for det in msg.yolov8_inference:
            if det.class_name.lower() == "ball":
                x_center = (det.left + det.right) / 2
                error = x_center - (self.image_width / 2)
                self.last_bbox = (det.left, det.right, det.top, det.bottom)

                if abs(error) < self.tolerance:
                    self.get_logger().info("Ball centered.")
                    self.target_centered = True
                    self.estimate_distance()

                    subprocess.Popen(["ros2", "run", "ball_kicking", "calculate_and_move_to_ball.py"])
                else:
                    angular_z = -self.kp * error
                    twist = Twist()
                    twist.angular.z = angular_z
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"Tracking ball: error={error}, angular_z={angular_z:.3f}")
                    self.target_found = True
                return

    def estimate_distance(self):
        if self.latest_filtered_lidar is None:
            self.get_logger().warn("No filtered LiDAR data.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            robot_position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        min_distance = float('inf')
        closest_point = None

        for pt in pc2.read_points(self.latest_filtered_lidar, skip_nans=True, field_names=("x", "y", "z")):
            point = np.array([pt[0], pt[1], pt[2]])
            dist = np.linalg.norm(point - robot_position)
            if dist < min_distance:
                min_distance = dist
                closest_point = point

        if closest_point is not None:
            x, y, z = closest_point
            self.get_logger().info(f"Ball position x={x:.2f}, y={y:.2f}, z={z:.2f}, distance={min_distance:.2f} m")
            self.last_ball_pose = Pose() 
            self.last_ball_pose.position.x = float(x)
            self.last_ball_pose.position.y = float(y)
            self.last_ball_pose.position.z = float(z)
            self.last_ball_pose.orientation.w = 1.0
        else:
            self.get_logger().warn("No valid points in filtered LiDAR.")

    def publish_positions(self):
        if self.last_ball_pose is None:
            return

        msg = ObjectsPosition()
        msg.name = ['soccer_ball', 'goal']
        msg.pose = [self.last_ball_pose, self.goal_pose]
        self.pose_array_pub.publish(msg)

    def loop_behavior(self):
        if not self.target_found:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)
            self.get_logger().info("Searching for ball...")

        # Publicar la posición constantemente si ya fue detectado
        if self.last_ball_pose is not None:
            self.publish_positions()

def main(args=None):
    rclpy.init(args=args)
    node = BallFollowerWithFilteredLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
