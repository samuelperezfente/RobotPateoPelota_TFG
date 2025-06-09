#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_go.msg import WebRtcReq
import time
import json

class AIForwardMover(Node):
    def __init__(self):
        super().__init__('ai_forward_mover')
        self.publisher = self.create_publisher(WebRtcReq, '/webrtc_req', 10)
        time.sleep(0.5)  # Aseguramos conexión con bridge

        # Cambiar al modo "ai"
        self.send_mode_switch("general")
        time.sleep(5.0)

        # # Activar freeAvoid
        # self.send_free_avoid(True)
        # time.sleep(0.3)

        # # Mover hacia adelante por 1 segundo
        # self.send_move(0.3, 0.0, 0.0, duration=1.0)

        # # Detener movimiento
        # self.send_move(0.0, 0.0, 0.0, duration=0.2)

        # # Regresar a modo "normal"
        # self.send_mode_switch("normal")
        # self.get_logger().info("✅ Movimiento finalizado y modo restaurado.")

    def send_mode_switch(self, mode: str):
        msg = WebRtcReq()
        msg.api_id = 1002  # MOTION_SWITCHER_API_ID_SELECT_MODE
        msg.topic = "rt/api/motion_switcher/request"
        msg.parameter = json.dumps({ "name": mode })
        self.publisher.publish(msg)
        self.get_logger().info(f"🔄 Cambiando al modo: {mode}")

    def send_free_avoid(self, enable: bool):
        msg = WebRtcReq()
        msg.api_id = 1048  # ROBOT_SPORT_API_ID_FREEAVOID
        msg.topic = "rt/api/sport/request"
        msg.parameter = json.dumps({ "flag": True })
        self.publisher.publish(msg)
        self.get_logger().info(f"🚧 FreeAvoid {'activado' if enable else 'desactivado'}")

    def send_move(self, x, y, yaw, duration=1.0):
        msg = WebRtcReq()
        msg.api_id = 1008  # SPORT_API_ID_MOVE
        msg.topic = "rt/api/sport/request"
        msg.parameter = json.dumps({ "x": x, "y": y, "z": yaw })
        start = time.time()
        while time.time() - start < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.get_logger().info(f"🏃 Movimiento enviado: x={x}, y={y}, yaw={yaw} por {duration:.1f}s")

def main(args=None):
    rclpy.init(args=args)
    node = AIForwardMover()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
