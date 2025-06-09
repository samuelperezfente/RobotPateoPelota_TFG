#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ball_kicking_interfaces.action import DetectBallAndGoal


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.det_client = ActionClient(self, DetectBallAndGoal, 'detect_ball_and_goal')
        self.kick_client = ActionClient(self, DetectBallAndGoal, 'calculate_and_move_to_ball')
        self.send_detection_goal()

    def send_detection_goal(self):
        self.get_logger().info("🛰️ Esperando a servidor de detección...")
        self.det_client.wait_for_server()

        goal_msg = DetectBallAndGoal.Goal()
        goal_msg.start = True

        self.get_logger().info("📤 Enviando goal de detección")
        self._det_goal_future = self.det_client.send_goal_async(goal_msg, feedback_callback=self.detection_feedback_callback)
        self._det_goal_future.add_done_callback(self.detection_goal_response_callback)

    def detection_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"📡 Feedback detección: {feedback.status}")

    def detection_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal de detección rechazado")
            return

        self.get_logger().info("✅ Goal de detección aceptado")
        self._det_result_future = goal_handle.get_result_async()
        self._det_result_future.add_done_callback(self.detection_result_callback)

    def detection_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"🏁 Detección exitosa: {result.message}")
            self.send_kick_goal()
        else:
            self.get_logger().error(f"❌ Falló la detección: {result.message}")

    def send_kick_goal(self):
        self.get_logger().info("🛰️ Esperando a servidor de pateo...")
        self.kick_client.wait_for_server()

        goal_msg = DetectBallAndGoal.Goal()
        goal_msg.start = True

        self.get_logger().info("📤 Enviando goal de pateo")
        self._kick_goal_future = self.kick_client.send_goal_async(goal_msg, feedback_callback=self.kick_feedback_callback)
        self._kick_goal_future.add_done_callback(self.kick_goal_response_callback)

    def kick_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"📡 Feedback pateo: {feedback.status}")

    def kick_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal de pateo rechazado")
            return

        self.get_logger().info("✅ Goal de pateo aceptado")
        self._kick_result_future = goal_handle.get_result_async()
        self._kick_result_future.add_done_callback(self.kick_result_callback)

    def kick_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"🥅 Pateo completado con éxito: {result.message}")
        else:
            self.get_logger().error(f"⚠️ Pateo fallido: {result.message}")

        self.get_logger().info("🧼 Finalizando nodo orquestador")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
