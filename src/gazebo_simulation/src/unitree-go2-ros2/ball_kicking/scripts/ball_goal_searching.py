#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference

class BallGoalFinder(Node):
    def __init__(self):
        super().__init__('ball_goal_finder')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)

        self.image_width = 640
        self.kp = 0.002

        self.ball_found = False
        self.goal_found = False
        self.finished = False  # Evita seguir girando después de encontrar ambos

        self.timer = self.create_timer(0.1, self.search_behavior)

    def detection_callback(self, msg: Yolov8Inference):
        if not msg.yolov8_inference or self.finished:
            return

        for det in msg.yolov8_inference:
            name = det.class_name.lower()
            if name == "ball":
                self.ball_found = True
                self.get_logger().info("⚽ Pelota detectada")
            elif name == "goalpost":
                self.goal_found = True
                self.get_logger().info("🥅 Portería detectada")

        if self.ball_found and self.goal_found and not self.finished:
            self.get_logger().info("✅ Pelota y portería detectadas. ¡Listo para jugar!")
            self.stop_movement()
            self.finished = True

            subprocess.Popen(["ros2", "run", "ball_kicking", "calculate_and_move_to_ball.py"])

    def search_behavior(self):
        if not self.ball_found or not self.goal_found:
            twist = Twist()
            twist.linear.x = 0.05
            twist.angular.z = 0.4
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"🔍 Buscando pelota {self.ball_found}  y portería {self.goal_found}...")

    def stop_movement(self):
        twist = Twist()  # Todos los valores por defecto en 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallGoalFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
