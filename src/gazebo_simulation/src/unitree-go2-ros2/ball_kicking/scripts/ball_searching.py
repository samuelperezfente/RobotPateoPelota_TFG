#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)

        self.image_width = 1280   # Ancho de la imagen (ajustar a tu cámara)
        self.tolerance = 5      # Tolerancia en píxeles para considerar centrado
        self.kp = 0.002          # Ganancia proporcional para el giro

        self.target_found = False
        self.target_centered = False

        self.timer = self.create_timer(0.1, self.search_behavior)

    def detection_callback(self, msg: Yolov8Inference):
        # self.target_found = False

        if not msg.yolov8_inference:
            if self.target_found:
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                self.cmd_pub.publish(twist)
            return
        
        

        for det in msg.yolov8_inference:
            if det.class_name.lower() == "ball":
                angular_z = 0.0
                linear_x = 0.0
                
                x_center = (det.left + det.right) / 2
                error = x_center - (self.image_width / 2)

                if abs(error) < self.tolerance:
                    self.get_logger().info("Ball centered.")
                    self.target_centered = True
                else:
                    self.tolerance = 5
                    angular_z = -self.kp * error
                    linear_x = 0.05

                twist = Twist()
                twist.linear.x = linear_x
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.y = 0.0
                twist.angular.z = angular_z

                self.cmd_pub.publish(twist)
                self.get_logger().info(f"Tracking ball: top={det.top}, bottom={det.bottom}, left={det.left}, right={det.right}, x_center={x_center}, error={error}, z={angular_z:.3f}")
                self.target_found = True
                return
        

    def search_behavior(self):
        # Si no se ha encontrado la pelota en la última detección, girar buscando
        if not self.target_found:
            twist = Twist()
            twist.linear.x = 0.05
            twist.angular.z = 0.3  # Giro constante para búsqueda
            self.cmd_pub.publish(twist)
            self.get_logger().info("Searching for ball...")

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
