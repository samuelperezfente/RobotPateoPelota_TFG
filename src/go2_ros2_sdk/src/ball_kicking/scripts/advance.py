#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import WebRtcReq
import time
import json

class WebRTCForwardMover(Node):
    def __init__(self):
        super().__init__('webrtc_forward_mover')
        self.publisher = self.create_publisher(WebRtcReq, '/webrtc_req', 10)

        self.get_logger().info("📡 Enviando comandos WebRTC para moverse hacia adelante 1 segundo...")

        # Movimiento hacia adelante
        x, y, z = 0.3, 0.0, 0.0  # Ajusta los valores si tu robot requiere otro paso

        parameters = {
            "x": x,
            "y": y,
            "z": z
        }

        msg = WebRtcReq()
        msg.api_id = 1008
        msg.parameter = json.dumps(parameters)
        msg.topic = "rt/api/sport/request"

        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.publisher.publish(msg)
            time.sleep(0.1)

        self.get_logger().info("✅ Comando enviado durante 1 segundo. Movimiento completo.")

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCForwardMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()