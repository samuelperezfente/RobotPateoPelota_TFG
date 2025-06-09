#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from ball_kicking_interfaces.action import DetectBallAndGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference

import math

class CalculateAndMoveToBallServer(Node):
    def __init__(self):
        super().__init__('calculate_and_move_to_ball_action_server')

        self._action_server = ActionServer(
            self,
            DetectBallAndGoal,
            'calculate_and_move_to_ball',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)
        self.yolo_sub = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)

        self.ball_pose = None
        self.goal_pose = None
        self.robot_pose = None
        self.positions_initialized = False
        self.phase = 0

        self.image_width = 1280
        self.tolerance = 5
        self.kp = 0.002

        self.last_yolo_detections = []
        self.visual_detection_ready = False
        self.visual_centered = False

        self._goal_handle = None
        self.feedback_msg = DetectBallAndGoal.Feedback()

        self.timer = self.create_timer(0.1, self.movement_loop)

    def goal_callback(self, goal_request):
        self.get_logger().info('🎯 Nueva meta recibida para mover al balón')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('❌ Solicitud de cancelación recibida')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('🚀 Acción iniciada: alineando con balón')
        self._goal_handle = goal_handle
        self.phase = 0
        self.ball_pose = None
        self.goal_pose = None
        self.robot_pose = None
        self.positions_initialized = False
        self.visual_detection_ready = False
        self.visual_centered = False
        self.last_yolo_detections = []

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_movement()
                goal_handle.canceled()
                self.get_logger().info("⚠️ Acción cancelada")
                return DetectBallAndGoal.Result(success=False, message="Acción cancelada")

            if self.phase >= 4:
                self.stop_movement()
                goal_handle.succeed()
                self.get_logger().info("✅ Alineación completada")
                return DetectBallAndGoal.Result(success=True, message="Alineación finalizada")

            self.feedback_msg.status = f"Fase actual: {self.phase}"
            goal_handle.publish_feedback(self.feedback_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        return DetectBallAndGoal.Result(success=False, message="Nodo finalizado inesperadamente")

    def model_callback(self, msg):
        if self._goal_handle is None:
            return
        try:
            robot_index = msg.name.index("go2")
            self.robot_pose = msg.pose[robot_index]

            if not self.positions_initialized:
                ball_index = msg.name.index("soccer_ball")
                goal_index = msg.name.index("robocup_3d_goal")
                self.ball_pose = msg.pose[ball_index]
                self.goal_pose = msg.pose[goal_index]
                self.positions_initialized = True
                self.get_logger().info("✅ Posiciones obtenidas")
        except ValueError:
            self.get_logger().warn("Esperando modelos...")

    def yolo_callback(self, msg):
        if self._goal_handle is None:
            return
        self.last_yolo_detections = msg.yolov8_inference
        for det in msg.yolov8_inference:
            if det.class_name.lower() == "ball":
                x_center = (det.left + det.right) / 2
                error = x_center - (self.image_width / 2)
                self.visual_centered = abs(error) < self.tolerance
        self.visual_detection_ready = True

    def movement_loop(self):
        if not self._goal_handle or not (self.ball_pose and self.goal_pose and self.robot_pose):
            return

        if self.phase == 0:
            self.go_to_alignment_point()
        elif self.phase == 1:
            self.rotate_towards_ball()
        elif self.phase == 1.5:
            self.visual_check_before_shift()
        elif self.phase == 2:
            self.shift_left()
        elif self.phase == 3:
            self.advance_forward()

    def go_to_alignment_point(self):
        x1, y1 = self.ball_pose.position.x, self.ball_pose.position.y
        x2, y2 = self.goal_pose.position.x, self.goal_pose.position.y
        xr, yr = self.robot_pose.position.x, self.robot_pose.position.y

        dx, dy = x2 - x1, y2 - y1
        norm = math.sqrt(dx**2 + dy**2)
        dx /= norm
        dy /= norm

        offset = 0.6
        target_x = x1 - offset * dx
        target_y = y1 - offset * dy

        to_target_x = target_x - xr
        to_target_y = target_y - yr
        distance = math.sqrt(to_target_x**2 + to_target_y**2)
        angle_to_target = math.atan2(to_target_y, to_target_x)

        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        angle_diff = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))

        twist = Twist()
        if abs(angle_diff) > 0.05:
            twist.angular.z = 0.5 * angle_diff
            twist.linear.x = 0.05
        elif distance > 0.05:
            twist.linear.x = 0.8 * distance
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.phase = 1
        self.cmd_pub.publish(twist)

    def rotate_towards_ball(self):
        x1, y1 = self.ball_pose.position.x, self.ball_pose.position.y
        xr, yr = self.robot_pose.position.x, self.robot_pose.position.y

        dx = x1 - xr
        dy = y1 - yr
        angle_to_ball = math.atan2(dy, dx)

        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        angle_diff = math.atan2(math.sin(angle_to_ball - yaw), math.cos(angle_to_ball - yaw))

        twist = Twist()
        if abs(angle_diff) > 0.01:
            twist.angular.z = 0.5 * angle_diff
            twist.linear.x = 0.055
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.phase = 1.5
        self.cmd_pub.publish(twist)

    def visual_check_before_shift(self):
        twist = Twist()
        if self.visual_detection_ready:
            if self.visual_centered:
                self.phase = 2
            else:
                for det in self.last_yolo_detections:
                    if det.class_name.lower() == "ball":
                        x_center = (det.left + det.right) / 2
                        error = x_center - (self.image_width / 2)
                        self.visual_centered = abs(error) < self.tolerance
                        twist.angular.z = -self.kp * error
                        twist.linear.x = 0.055
                        break
        self.cmd_pub.publish(twist)

    def shift_left(self):
        if not hasattr(self, 'shift_start_pose'):
            self.shift_start_pose = self.robot_pose.position
            return

        xr, yr = self.robot_pose.position.x, self.robot_pose.position.y
        x0, y0 = self.shift_start_pose.x, self.shift_start_pose.y

        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        dx_lateral = -math.sin(yaw)
        dy_lateral = math.cos(yaw)

        delta_x = xr - x0
        delta_y = yr - y0
        projection = delta_x * dx_lateral + delta_y * dy_lateral

        twist = Twist()
        if abs(projection) < 0.115:
            twist.linear.x = 0.055
            twist.linear.y = 0.1
        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.phase = 3
        self.cmd_pub.publish(twist)

    def advance_forward(self):
        if not hasattr(self, 'advance_start_pose'):
            self.advance_start_pose = self.robot_pose.position
            return

        xr, yr = self.robot_pose.position.x, self.robot_pose.position.y
        x0, y0 = self.advance_start_pose.x, self.advance_start_pose.y

        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        dx_forward = math.cos(yaw)
        dy_forward = math.sin(yaw)

        delta_x = xr - x0
        delta_y = yr - y0
        projection = delta_x * dx_forward + delta_y * dy_forward

        twist = Twist()
        if abs(projection) < 1.0:
            twist.linear.x = 1.0
        else:
            twist.linear.x = 0.0
            self.phase = 4
        self.cmd_pub.publish(twist)

    def stop_movement(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info("🛑 Movimiento detenido")

    def get_yaw_from_quaternion(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


def main(args=None):
    rclpy.init(args=args)
    node = CalculateAndMoveToBallServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
