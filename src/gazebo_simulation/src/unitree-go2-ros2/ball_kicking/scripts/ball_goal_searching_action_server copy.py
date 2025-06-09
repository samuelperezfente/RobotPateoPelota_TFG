#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference
from ball_kicking_interfaces.action import DetectBallAndGoal

class BallGoalFinder(Node):
    def __init__(self):
        super().__init__('ball_goal_finder')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self._action_server = ActionServer(
            self,
            DetectBallAndGoal,
            'detect_ball_and_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.image_width = 640
        self.kp = 0.002

        self.ball_found = False
        self.goal_found = False
        self.goal_handle = None
        self.feedback_msg = DetectBallAndGoal.Feedback()

    def goal_callback(self, goal_request):
        self.get_logger().info('🎯 Nueva meta recibida para búsqueda')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('❌ Solicitud de cancelación recibida')
        return CancelResponse.ACCEPT

    

    async def execute_callback(self, goal_handle):
        self.get_logger().info('🚀 Acción iniciada: buscando pelota y portería')
        self.ball_found = False
        self.goal_found = False
        self.goal_handle = goal_handle

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_movement()
                goal_handle.canceled()
                self.get_logger().info('⚠️ Acción cancelada')
                return DetectBallAndGoal.Result(success=False, message="Acción cancelada")

            if self.ball_found and self.goal_found:
                self.stop_movement()
                goal_handle.succeed()
                self.get_logger().info('✅ Pelota y portería detectadas')
                self.goal_handle = None
                return DetectBallAndGoal.Result(success=True, message="Detección completa")

            self.search_behavior()
            self.feedback_msg.status = '🔍 Buscando...'
            goal_handle.publish_feedback(self.feedback_msg)
            

        return DetectBallAndGoal.Result(success=False, message="Nodo finalizado inesperadamente")


    def detection_callback(self, msg):
        if self.goal_handle is None:
            return

        for det in msg.yolov8_inference or []:
            name = det.class_name.lower()
            if name == 'ball':
                self.ball_found = True
                self.get_logger().info('⚽ Pelota detectada')
            elif name in ['goalpost', 'goal']:
                self.goal_found = True
                self.get_logger().info('🥅 Portería detectada')

    def search_behavior(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.4
        self.cmd_pub.publish(twist)

    def stop_movement(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    

    rclpy.init(args=args)
    node = BallGoalFinder()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
