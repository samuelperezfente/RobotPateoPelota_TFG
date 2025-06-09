#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_go.msg import WebRtcReq
import time
import json

class ObstaclesAvoidMover(Node):
    def __init__(self):
        super().__init__('obstacles_avoid_mover')
        self.publisher = self.create_publisher(WebRtcReq, '/webrtc_req', 10)

        self.get_logger().info("🚀 Iniciando movimiento con control remoto de obstacles_avoid...")

        # 1. Enviar SwitchSet(True) -> api_id 1001
        self.send_api_command(1001, {"enable": True}, "Activando switch")

        # 2. Esperar un poco
        time.sleep(0.2)

        # 3. UseRemoteCommandFromApi(True) -> api_id 1004
        self.send_api_command(1004, {"is_remote_commands_from_api": True}, "Activando control remoto desde API")

        # 4. Movimiento hacia adelante -> api_id 1003
        move_params = {"x": 0.5, "y": 0.0, "yaw": 0.0, "mode": 0}
        start_time = time.time()
        while time.time() - start_time < 1.0:
            print("Moviendo")
            self.send_api_command(1003, move_params)
            time.sleep(0.1)

        # 5. Parar el movimiento
        self.send_api_command(1003, {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}, "Parando movimiento")

        # 6. UseRemoteCommandFromApi(False)
        self.send_api_command(1004, {"is_remote_commands_from_api": False}, "Desactivando control remoto desde API")

        self.get_logger().info("✅ Movimiento completado.")

    def send_api_command(self, api_id, params, log=None):
        msg = WebRtcReq()
        msg.api_id = api_id
        msg.parameter = json.dumps(params)
        msg.topic = "rt/api/obstacles_avoid/request"
        self.publisher.publish(msg)
        if log:
            self.get_logger().info(f"📡 {log}: api_id={api_id}, params={params}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesAvoidMover()
    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
