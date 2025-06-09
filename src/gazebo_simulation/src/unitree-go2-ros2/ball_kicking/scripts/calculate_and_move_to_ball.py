#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference
import math

class TeleopAligner(Node):
    def __init__(self):
        super().__init__('teleop_aligner')
        
        super().__init__('teleop_aligner')

        self.model_sub = self.create_subscription(ModelStates, '/model_states', self.callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yolo_sub = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)

        self.ball_pose = None
        self.goal_pose = None
        self.robot_pose = None

        self.positions_initialized = False
        self.phase = 0  # 0: alineación, 1: giro, 1.5: chequeo visual, 2: desplazamiento lateral, 3: avance
        self.timer = self.create_timer(0.1, self.move_towards_alignment)

        self.image_width = 1280
        self.tolerance = 5
        self.kp = 0.002
        self.last_yolo_detections = []
        self.visual_detection_ready = False
        self.visual_centered = False

    def callback(self, msg: ModelStates):
        try:
            robot_index = msg.name.index("go2")
            self.robot_pose = msg.pose[robot_index]

            if not self.positions_initialized:
                ball_index = msg.name.index("soccer_ball")
                goal_index = msg.name.index("robocup_3d_goal")

                self.ball_pose = msg.pose[ball_index]
                self.goal_pose = msg.pose[goal_index]

                self.positions_initialized = True
                self.get_logger().info("✅ Posición de pelota y portería adquiridas.")

        except ValueError:
            self.get_logger().warn("Esperando modelos en Gazebo...")

    def yolo_callback(self, msg: Yolov8Inference):
        self.last_yolo_detections = msg.yolov8_inference

        ball_detected = False
        goal_detected = False
        ball_centered = False

        for det in msg.yolov8_inference:
            class_name = det.class_name.lower()
            if class_name == "ball":
                ball_detected = True
                x_center = (det.left + det.right) / 2
                error = x_center - (self.image_width / 2)
                ball_centered = abs(error) < self.tolerance
            elif class_name == "goal":
                goal_detected = True

        self.visual_detection_ready = True
        # self.visual_centered = ball_centered

    def move_towards_alignment(self):
        if not (self.ball_pose and self.goal_pose and self.robot_pose):
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

        self.cmd_pub.publish(twist)
        if abs(angle_diff) > 0.05:
            twist.angular.z = 0.5 * angle_diff
            twist.linear.x = 0.05
        elif distance > 0.05:
            twist.linear.x = 0.8 * distance
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.phase = 1
            self.get_logger().info("🟢 Alineado detrás de la pelota. Girando hacia ella...")
            self.wait_for_user("✅ Fase 0 completada. Pulsa ENTER para girar hacia la pelota (fase 1)...")
            self.cmd_pub.publish(twist)
            return

        # linear_x = 0.0
        # angular_z = 0.0
        

        # if distance > 0.1:
        #     linear_x = 0.5 * distance

        # # Gira si no está bien orientado
        # if abs(angle_diff) > 0.1:
        #     angular_z = 0.5 * angle_diff
        #     linear_x = 0.05 

        # twist.linear.x = linear_x
        # twist.angular.z = angular_z

        # if distance > 0.1:
        #     twist.linear.x = 0.5 * distance
        # elif abs(angle_diff) > 0.1:
        #     twist.angular.z = 0.5 * angle_diff
        #     twist.linear.x = 0.05
        # else:
        #     self.reached_alignment = True
        #     self.get_logger().info("🟢 Alineado detrás de la pelota. Girando hacia ella...")
        #     return

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"🎯 Moviendo hacia ({target_x:.2f}, {target_y:.2f}), distancia: {distance:.2f}")


    def rotate_towards_ball(self):
        x1, y1 = self.ball_pose.position.x, self.ball_pose.position.y
        xr, yr = self.robot_pose.position.x, self.robot_pose.position.y

        dx = x1 - xr
        dy = y1 - yr
        angle_to_ball = math.atan2(dy, dx)

        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        angle_diff = math.atan2(math.sin(angle_to_ball - yaw), math.cos(angle_to_ball - yaw))

        twist = Twist()

        if abs(angle_diff) > 0.001:
            twist.angular.z = 0.5 * angle_diff
            twist.linear.x = 0.055
        else:
            self.phase = 1.5
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("🔁 Giro completado. Reposicionando lateralmente.")
            self.wait_for_user("✅ Fase 1 completada. Pulsa ENTER para girar hacia la pelota (fase 2)...")
            return

        self.cmd_pub.publish(twist)

    def visual_check_before_shift(self):
        twist = Twist()

        if self.visual_detection_ready:
            if self.visual_centered:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info("✅ Pelota y portería detectadas. Pelota centrada.")
                self.wait_for_user("Pulsa ENTER para iniciar el desplazamiento lateral (fase 2)...")
                self.phase = 2
            else:
                for det in self.last_yolo_detections:
                    if det.class_name.lower() == "ball":
                        x_center = (det.left + det.right) / 2
                        error = x_center - (self.image_width / 2)
                        self.visual_centered = abs(error) < self.tolerance
                        twist.angular.z = -self.kp * error
                        twist.linear.x = 0.055
                        self.cmd_pub.publish(twist)
                        self.get_logger().info(f"🎯 Centrando pelota... error={error:.2f}, z={twist.angular.z:.3f}")
                        break
        else:
            self.cmd_pub.publish(twist)
            self.get_logger().info("🔍 Esperando detección de pelota y portería...")

    def shift_left_camera_ball_position(self):
        twist = Twist()

        if not self.visual_detection_ready:
            self.get_logger().info("⏳ Esperando detección visual para desplazar lateralmente...")
            self.cmd_pub.publish(twist)
            return

        desired_x_position = 0.585 * self.image_width  # Objetivo: 75% del ancho de la imagen
        error_margin = 10  # píxeles de margen de error
        max_speed = 0.1     # velocidad máxima lateral
        kp = 0.0015         # ganancia proporcional

        ball_detected = False

        for det in self.last_yolo_detections:
            if det.class_name.lower() == "ball":
                ball_detected = True
                x_center = (det.left + det.right) / 2
                error = x_center - desired_x_position

                self.get_logger().info(f"🎯 Pelota x = {x_center:.1f}, error = {error:.1f}")

                if abs(error) < error_margin:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    self.cmd_pub.publish(twist)
                    self.get_logger().info("✅ Pelota correctamente posicionada en la zona deseada.")
                    self.wait_for_user("✅ Fase 2 completada. Pulsa ENTER para avanzar hacia la pelota (fase 3)...")
                    self.phase = 3
                    return
                else:
                    # Calcula velocidad lateral proporcional al error
                    velocity_y = -kp * error  # negativo porque mover a la izquierda desplaza la pelota a la derecha
                    velocity_y = max(-max_speed, min(max_speed, velocity_y))  # limitador de velocidad
                    twist.linear.y = velocity_y
                    twist.linear.x = 0.055
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"↔️ Corrigiendo posición lateral: vel_y = {velocity_y:.3f}")
                    return

        if not ball_detected:
            self.get_logger().warn("⚠️ Pelota no detectada visualmente durante desplazamiento lateral.")
            self.cmd_pub.publish(twist)



    def shift_left(self):
        if not hasattr(self, 'shift_start_pose'):
            self.shift_start_pose = self.robot_pose.position
            self.get_logger().info("🔄 Iniciando desplazamiento lateral...")
            return  # Espera al siguiente ciclo para tener datos estables

        # Coordenadas actuales del robot
        xr = self.robot_pose.position.x
        yr = self.robot_pose.position.y

        # Coordenadas del inicio del desplazamiento
        x0 = self.shift_start_pose.x
        y0 = self.shift_start_pose.y

        # Dirección lateral izquierda respecto a orientación actual
        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        dx_lateral = -math.sin(yaw)
        dy_lateral = math.cos(yaw)

        # Vector de desplazamiento desde inicio
        delta_x = xr - x0
        delta_y = yr - y0
        projection = delta_x * dx_lateral + delta_y * dy_lateral  # distancia lateral proyectada

        distance_goal = 0.115  # metros que quieres moverte hacia la izquierda

        if abs(projection) < distance_goal:
            twist = Twist()
            twist.linear.x = 0.055
            twist.linear.y = 0.1  # mueve lateralmente hacia la izquierda
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"↔️ Desplazando... {projection:.3f} m")
        else:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().info("✅ Desplazamiento lateral completado.")
            self.wait_for_user("✅ Fase 2 completada. Pulsa ENTER para girar hacia la pelota (fase 3)...")
            self.side_shift_done = True
            self.phase = 3  # O siguiente fase si tienes más

    def advance_forward(self):
        if not hasattr(self, 'advance_start_pose'):
            self.advance_start_pose = self.robot_pose.position
            self.get_logger().info("🏃 Iniciando avance hacia adelante...")
            return  # espera un ciclo para registrar la pose inicial

        # Posición actual
        xr = self.robot_pose.position.x
        yr = self.robot_pose.position.y

        # Posición inicial del avance
        x0 = self.advance_start_pose.x
        y0 = self.advance_start_pose.y

        # Dirección hacia adelante basada en el yaw actual
        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        dx_forward = math.cos(yaw)
        dy_forward = math.sin(yaw)

        # Vector desplazamiento desde inicio
        delta_x = xr - x0
        delta_y = yr - y0
        projection = delta_x * dx_forward + delta_y * dy_forward  # distancia frontal proyectada

        distance_goal = 1.0  # metros que quieres avanzar

        if abs(projection) < distance_goal:
            twist = Twist()
            twist.linear.x = 1.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"⬆️ Avanzando... {projection:.3f} m")
        else:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().info("✅ Avance completado. Secuencia finalizada.")
            self.wait_for_user("✅ Fase 3 completada. Pulsa ENTER para girar hacia la pelota (fase 4)...")
            self.phase = 4  # Puedes usar esta fase para detener todo o hacer algo más



    def stop_movement(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().info("✅ Posicionado. Secuencia completa.")
        self.timer.cancel()  # Detiene el movimiento continuo

    def get_yaw_from_quaternion(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y**2 + q.z**2))
    
    def wait_for_user(self, mensaje="Presiona ENTER para continuar a la siguiente fase..."):
        input(mensaje)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
