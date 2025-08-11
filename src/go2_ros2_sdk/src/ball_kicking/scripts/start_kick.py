#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from yolov8_msgs.msg import Yolov8InferencePosition, InferenceResultPosition
from object_msgs.msg import FrontDistance, FrontDetection
from transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped
from unitree_go.msg import WebRtcReq
from unitree_api.msg import Request
import tf2_ros
import math
import json
import numpy as np
import time
import random


class VUI_COLOR:
    WHITE: str = 'white'
    RED: str = 'red'
    YELLOW: str = 'yellow'
    BLUE: str = 'blue'
    GREEN: str = 'green'
    CYAN: str = 'cyan'
    PURPLE: str = 'purple'

class ObjectAligner(Node):
    def __init__(self):
        super().__init__('object_aligner')

        self.Led_Color = VUI_COLOR()
        self.led_color_time = 1

        self.image_width = 1280
        self.image_height = 720

        self.center_exit = 125
        self.center_enter = 75
        self.lateral_exit = 120
        self.lateral_enter = 80

        self.too_far_exit = 1.0
        self.too_far_enter = 1.2
        self.too_close_exit = 2.5
        self.too_close_enter = 1.8

        self.angle_threshold = 0.07  # radianes
        self.close_threshold = 0.37
        self.far_threshold = 0.6
        self.too_close_threshold = 0.34
        self.too_far_lateral_threshold = 0.08
        self.lateral_distance_position = -0.037
        self.lateral_distance_threshold = 0.017

        self.cooldown = 1.75 # segundos

        self.last_lateral_move_time = self.get_clock().now()
        self.last_forward_move_time = self.get_clock().now()

        self.last_move_time = self.get_clock().now()
        self.last_move_type = "none"
        self.switch_cooldown = 1.75

        self.alignment_completed = False
        self.approach_completed = False
        self.completed_stable_time = 2.5 
        self.alignment_stable_since = None

        self.current_phase = 0  # 1: alineación, 2: acercamiento, 3: pateo

        self.time_since_lost_started = None
        self.timeout_to_phase_0 = 3.0

        self.min_speed = 0.12
        self.max_speed = 0.2

        # self.color_dict = {
        #     "rojo": (0, 200, 200),
        #     "naranja": (15, 200, 200),
        #     "amarillo": (30, 200, 200),
        #     "verde": (60, 200, 200),
        #     "azul": (110, 200, 200),
        #     "rosado": (140, 150, 200),
        #     "morado": (160, 150, 150),
        #     "blanco": (0, 20, 255),
        #     "gris": (0, 20, 128),
        #     "negro": (0, 0, 30)
        # }

        # self.color_dict = {
        #     "rojo": (0, 200, 200),
        #     "naranja": (15, 200, 200),
        #     "verde": (60, 200, 200),
        #     "azul": (110, 200, 200),
        #     "blanco": (0, 20, 255),
        # }

        self.target_kick_name = "kick_target"
        # self.target_kick = {"name": "Ball", "color_name": "azul"}
        self.target_goal_name = "goal_target"
        # self.target_goal = {"name": "Ball", "color_name": "verde"}

        self.kick_bbox = None
        self.goal_bbox = None

        self.kick_centered = False
        self.goal_centered = False
        self.advance_active = False
        self.retreat_active = False
        self.goal_aligned_lateral = False

        self.kick_completed = False

        self.front_object_position = None
        self.goal_marker_position = None

        self.robot_pose = None
        self.front_distances = None

        self.kick_offset_ok = True
        self.distance_ok = True
        self.lateral_ok = True
        self.goal_rotation_ok = True


        self.inference_sub = self.create_subscription(
            Yolov8InferencePosition,
            '/Yolov8_Inference_Position',
            self.inference_callback,
            10
        )

        # self.detections_marker_sub = self.create_subscription(
        #     MarkerArray,
        #     '/detections_marker_array',
        #     self.marker_callback,
        #     10
        # )

        # self.front_object_marker_sub = self.create_subscription(
        #     Marker,
        #     '/front_object_marker',
        #     self.front_marker_callback,
        #     10
        # )

        # self.front_object_distance_sub = self.create_subscription(
        #     FrontDistance,
        #     '/front_object_distance',
        #     self.front_distances_callback,
        #     10
        # )

        self.front_object_detection_sub = self.create_subscription(
            FrontDetection,
            '/front_object_detection',
            self.front_detection_callback,
            10
        )

        self.pose_robot = self.create_subscription(
            PoseStamped,
            '/utlidar/robot_pose',
            self.pose_robot_callback,
            10
        )

        

        self.publisher_webrtc = self.create_publisher(WebRtcReq, '/webrtc_req', 10)
        self.publisher_sport_api = self.create_publisher(Request, '/api/sport/request', 10)
        self.publisher_vui_api = self.create_publisher(Request, '/api/vui/request', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.timer = self.create_timer(0.1, self.control_loop)
        # self.time_led = self.create_timer(0.1, self.led_loop)

    def led_loop(self):
        if self.kick_completed:
            return
        
        self.change_led_color()

    def control_loop(self):
        if self.kick_completed:
            return
        
        self.evaluate_phase()
        # self.get_logger().info(f"Fase actual: {self.current_phase}")
        self.change_led_color_api()
        
        if self.current_phase == 1:
            self.execute_alignment()
        elif self.current_phase == 2:
            self.execute_approach()
        elif self.current_phase == 3:
            self.execute_kick()

    def change_led_color(self):
        if self.current_phase == 0:
            color = self.Led_Color.RED
        elif self.current_phase == 1:
            color = self.Led_Color.YELLOW
        elif self.current_phase == 2:
            color = self.Led_Color.CYAN
        elif self.current_phase == 3:
            color = self.Led_Color.PURPLE
            self.led_color_time = 4

        msg = WebRtcReq()
        msg.api_id = 1007  # SPORT_API_ID_HELLO
        msg.parameter = json.dumps({"color": color, "time": self.led_color_time})
        msg.topic = "rt/api/vui/request"

        self.publisher_webrtc.publish(msg)

    def change_led_color_api(self):
        if self.current_phase == 0:
            color = self.Led_Color.RED
        elif self.current_phase == 1:
            color = self.Led_Color.YELLOW
        elif self.current_phase == 2:
            color = self.Led_Color.CYAN
        elif self.current_phase == 3:
            color = self.Led_Color.PURPLE
            self.led_color_time = 4

        request = Request()
        request.header.identity.id = int(time.time() * 1e6) + random.randint(0, 1000)
        request.header.identity.api_id = 1007  # SPORT_API_ID_HELLO
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = True
        request.parameter = json.dumps({"color": color, "time": self.led_color_time})

        self.publisher_vui_api.publish(request)
        
    def evaluate_phase(self):
        # Sería ideal que los cambios a fase 0 debido a perder de vista los balones tuviesen un contador de espera, 
        #   ya que es normal que en un par de mensajes seguidos se pierda la posicion del balon debido a la posibilidad de que camara 
        #   y lidar no estan del todo juntas debido al delay
        if self.current_phase == 3:
            return
        # === Fase 0: No se detecta nada o se pierde visión ===
        if self.kick_bbox is None and self.goal_bbox is None:
            self.get_logger().info("🔍 No se visualiza ningún objetivo de interés.")
            self.request_phase_change(0)
            return

        if self.kick_bbox is not None and self.goal_bbox is None:
            self.get_logger().info("🎯 No se visualiza el objetivo destino del pateo.")
            if self.current_phase == 1:
                self.request_phase_change(1)
                return
            self.request_phase_change(0)
            return

        # A partir de aquí, siempre se visualiza el objetivo de destino (goal_bbox)

        # === Fase 1 en curso ===
        if self.current_phase == 1:
            if self.kick_bbox is None:
                # Puede ser que se perdió por acercamiento; mantener fase para retroceder
                self.get_logger().info("👀 Objetivo de pateo no visible, pero mantenemos fase 1 en caso de que sea por cercanía.")
                return
            # Aquí podrías evaluar si se completó la alineación en fase 1 (opcional)
            if self.alignment_completed:
                self.request_phase_change(2)
                self.alignment_completed = False
                return

        # === Caso especial: solo se ve la portería (y no el balón) ===
        if self.kick_bbox is None and self.goal_bbox is not None:
            if self.front_distances is None:
                self.get_logger().info("📏 Distancias no disponibles para comprobar posible cambio de fase.")
                self.request_phase_change(0)
                return

            front_x = self.front_distances.distance_x

            if front_x > self.far_threshold + 0.25:
                self.get_logger().info(
                    f"⚠️ Solo se ve la portería y el objeto frontal está a {front_x:.2f} m (> {self.far_threshold + 0.25:.2f} m). "
                    "No hay balón cerca, salimos de fase 2."
                )
                self.request_phase_change(0)     
                return

            self.get_logger().info(
                f"✅ Solo se ve la portería, pero el objeto frontal está a {front_x:.2f} m (<= {self.far_threshold + 0.05:.2f} m). "
                "El balón puede estar demasiado cerca para ser visto. Mantenemos fase 2."
            )

            # Esta todo listo para pasar a la fase 3 pateo?
            now = self.get_clock().now()

            if self.alignment_stable_since is None:
                self.get_logger().info("🕒 Esperando estabilidad...")
            else:
                stable_duration = (now - self.alignment_stable_since).nanoseconds / 1e9
                self.get_logger().info(f"🕒 Esperando estabilidad... ({stable_duration})")
                if stable_duration >= self.completed_stable_time:
                    self.get_logger().info("🎯 Alineación y posicionamiento completados de forma estable.")
                    self.request_phase_change(3)
                    self.alignment_stable_since = None
                    return
                else:
                    self.get_logger().info(f"⌛ Manteniendo posición... {stable_duration:.2f}s / {self.completed_stable_time}s")

            # No esta todo listo estonces nos quedamos en la fase dos
            
            self.request_phase_change(2)
            

        # === Ambos objetos visibles (balón y portería) ===

        # si estamos en fase 2 nos mantendremos en esta salvo que se pierda la distancia frontal (un caso para saberlo pordía ser que este mas lejos que el objeto destino)
        if self.kick_bbox is not None and self.goal_bbox is not None:
            if self.current_phase == 2:
                if self.front_object_position is None or self.goal_marker_position is None:
                    self.get_logger().info("❌ Posiciones no disponibles para análisis de distancia.")
                    self.request_phase_change(0)
                    return
            
            
            # # Verificamos si el balón está más lejos que la portería → caso extraño
            # dx_ball = self.front_object_position.x
            # dx_goal = self.goal_marker_position.x

            # if dx_ball - dx_goal > 0.15:
            #     self.get_logger().info(f"⚠️ El balón detectado está más lejos que la portería. Fase 0 por detección incorrecta.")
            #     self.current_phase = 0
            # else:
            #     self.get_logger().info("✅ Objetivos válidos detectados. Iniciando fase de alineación.")
            #     self.current_phase = 1
 
            if self.current_phase != 2:
                self.request_phase_change(1)

    
    def request_phase_change(self, desired_phase: int):
        
        # Intenta cambiar de fase. Si se quiere cambiar a la fase 0,
        # se introduce un retardo para evitar transiciones por pérdidas puntuales.
        # Cualquier otra transición reinicia el temporizador.

        # Args:
        #     desired_phase (int): Fase a la que se desea cambiar.
        
        now = self.get_clock().now()

        if desired_phase == 0:
            # Si ya estamos esperando, comprobamos el tiempo transcurrido
            if self.time_since_lost_started is None:
                self.time_since_lost_started = now
                self.get_logger().info("⏳ Posible pérdida de objetivos. Iniciando temporizador de cambio a fase 0.")
                return  # Aún no cambiamos
            else:
                elapsed = (now - self.time_since_lost_started).nanoseconds / 1e9
                if elapsed >= self.timeout_to_phase_0:
                    self.get_logger().info(f"❌ Objetivos perdidos durante {elapsed:.2f}s. Cambio a fase 0 confirmado.")
                    self.current_phase = 0
                    self.time_since_lost_started = None
                else:
                    self.get_logger().info(f"⌛ Esperando confirmación para cambio a fase 0... ({elapsed:.2f}s / {self.timeout_to_phase_0}s)")
                return  # Solo cambiamos si se cumple el tiempo
        else:
            # Cambio a cualquier otra fase → se cancela cualquier temporizador
            if self.current_phase != desired_phase:
                self.get_logger().info(f"🔄 Cambio de fase: {self.current_phase} → {desired_phase}")
            self.current_phase = desired_phase
            if self.time_since_lost_started is not None:
                self.get_logger().info("✅ Objetivos recuperados. Cancelando temporizador de pérdida.")
                self.time_since_lost_started = None


        

    def execute_alignment(self):
        if self.kick_bbox is None:
            return
              
        self.alignment_completed = False

        self.kick_offset_ok = True
        self.distance_ok = True
        self.lateral_ok = True

        vx, vy, vz = 0.0, 0.0, 0.0

        image_center = self.image_width // 2
        kick_center = self.get_center_x(self.kick_bbox)
        # goal_center = self.get_center_x(self.goal_bbox)
        kick_offset = kick_center - image_center
        # goal_offset = goal_center - image_center

        # Paso 1: Histéresis rotacional balón
        if not self.kick_centered:
            if abs(kick_offset) >= self.center_enter:
                self.get_logger().info(f"Rotando: {-0.001 * kick_offset}.")
                # self.move_axis('z', -0.001 * kick_offset)
                vz = -0.0015 * kick_offset
                self.kick_offset_ok = False
                
            else:
                self.kick_centered = True
        else:
            if abs(kick_offset) >= self.center_exit:
                self.kick_centered = False
                

        # Paso 2: Distancia balón (avanzar/retroceder)
        ball_width = self.kick_bbox.right - self.kick_bbox.left
        ball_height = self.kick_bbox.bottom - self.kick_bbox.top
        ball_ratio = ball_width / float(ball_height + 1e-6)

        # === Retroceso ===
        if not self.retreat_active:
            if ball_ratio > self.too_close_exit:
                self.get_logger().info("🔙 Pelota demasiado cerca. Iniciando retroceso.")
                # self.move_axis('x', -0.12)
                vx = -0.12
                self.retreat_active = True
                self.advance_active = False
                self.distance_ok = False
                
        else:
            if ball_ratio > self.too_close_enter:
                self.get_logger().info("⬅️ Retrocediendo aún, ratio alto.")
                vx = -0.12
                self.distance_ok = False
                # self.move_axis('x', -0.12)
                
            else:
                self.get_logger().info("✅ Se detiene retroceso.")
                self.retreat_active = False

        # === Avance ===
        if not self.advance_active:
            if self.kick_bbox.bottom < (self.image_height - 80) or ball_ratio < self.too_far_exit:
                self.get_logger().info("⚠️ Pelota lejos. Iniciando avance.")
                # self.move_axis('x', 0.12)
                vx = 0.12
                self.advance_active = True
                self.retreat_active = False
                self.distance_ok = False
                
        else:
            if self.kick_bbox.bottom < (self.image_height - 80) or ball_ratio < self.too_far_enter:
                self.get_logger().info("➡️ Avanzando hacia la pelota.")
                # self.move_axis('x', 0.12)
                vx = 0.12
                self.distance_ok = False
                
            else:
                self.get_logger().info("✅ Se detiene avance.")
                self.advance_active = False

        if self.goal_bbox is None:
            self.log_alignment_status(kick_offset, ball_ratio)
            self.move(vx, vy, vz)
            return
        
        # Paso 3: Alineación lateral con objetivo (con umbrales específicos)
        kick_center = self.get_center_x(self.kick_bbox)
        goal_center = self.get_center_x(self.goal_bbox)
        offset_lateral = goal_center - kick_center
        if not self.goal_aligned_lateral:
            if abs(offset_lateral) >= self.lateral_enter:
                lateral_direction = -1.0 if goal_center < kick_center else 1.0
                self.get_logger().info(f"Desplazamiento lateral: {lateral_direction * 0.16}.")
                # self.move_axis('y', lateral_direction * 0.12)
                # vy = lateral_direction * 16
                vy = offset_lateral * 0.001
                self.lateral_ok = False
                
            else:
                print("No estabamos alineados pero pasamos a estarlo")
                self.goal_aligned_lateral = True
        else:
            if abs(offset_lateral) >= self.lateral_exit:
                print("Estabamos alineados pero pasamos a no estarlo")
                self.goal_aligned_lateral = False
                

        # Paso 4: Centrado final de portería (rotación)
        goal_center = self.get_center_x(self.goal_bbox)
        goal_offset = goal_center - image_center
        if not self.goal_centered:
            if abs(goal_offset) >= self.center_enter:
                self.get_logger().info(f"Ultima rotacion: {-0.001 * goal_offset}.")
                # self.move_axis('z', -0.001 * goal_offset)
                # vz = -0.001 * goal_offset
                
            else:
                self.goal_centered = True
        else:
            if abs(goal_offset) >= self.center_exit:
                self.goal_centered = False

        # print(vy)
        if vx != 0.0 or vy != 0.0 or vz != 0.0:
            self.log_alignment_status(kick_offset, ball_ratio, offset_lateral, goal_offset)
            self.move(vx, vy, vz)
            return

        self.send_api_move(0.0, 0.0, 0.0)
        self.get_logger().info("✅ Alineación completa.")
        self.alignment_completed = True


    def log_alignment_status(self, kick_offset, ball_ratio, offset_lateral=None, goal_offset=None
    ):
        log_msg = (
            f"Fase actual: Aliñamento | "
            f"offset balón; {kick_offset:.1f}; {'OK' if self.kick_offset_ok else 'CORREGIR'} | "
            f"ratio balón; {ball_ratio:.2f}; {'OK' if self.distance_ok else 'CORREGIR'}"
        )

        if offset_lateral is not None:
            log_msg += f" | offset_lateral; {offset_lateral:.1f}; {'OK' if self.lateral_ok else 'CORREGIR'}"
        else:
            log_msg += f" | offset_lateral; desconocido; desconocido"

        if goal_offset is not None:
            log_msg += f" | offset_goal; {goal_offset:.1f}; {'OK' if self.goal_rotation_ok else 'CORREGIR'}"
        else:
            log_msg += f" | offset_goal; desconocido; desconocido"

        self.get_logger().info(log_msg)

        

    def execute_approach(self):
        if self.approach_completed:
            return

        if self.front_object_position is None or self.goal_marker_position is None:
            return

        ball = self.front_object_position
        goal = self.goal_marker_position

        try:
            quat = self.robot_pose.pose.orientation
            current_yaw, _, _ = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la orientación del robot: {e}")
            return

        dx = goal.x - ball.x
        dy = goal.y - ball.y
        desired_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_angle - current_yaw)

        now = self.get_clock().now()
        elapsed_since_switch = (now - self.last_move_time).nanoseconds / 1e9

        if self.front_distances is None:
            return

        front_distance = self.front_distances.distance_x
        lateral_distance = self.front_distances.distance_y
        lateral_error = lateral_distance - self.lateral_distance_position

        self.log_approach_status(angle_error, lateral_error, front_distance)

        # === Paso 1: RETROCESO si estamos demasiado cerca ===
        if front_distance < self.too_close_threshold:
            backward_elapsed = (now - self.last_forward_move_time).nanoseconds / 1e9

            # if self.last_move_type != "backward" and elapsed_since_switch < self.switch_cooldown:
            #     self.get_logger().info(f"⏱ Esperando para cambiar a retroceso ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
            #     return

            if backward_elapsed < self.cooldown:
                self.get_logger().info(f"⌛ Esperando para nuevo retroceso ({backward_elapsed:.2f}s < {self.cooldown}s)")
                return

            self.get_logger().info("⛔ Muy cerca del balón, iniciando retroceso.")

            self.get_logger().info("Subfase: Retroceso por proximidade excesiva.")
            # self.move_axis('x', -0.12)  # retroceso constante
            self.move(-0.12, 0.0, 0.0)
            self.last_forward_move_time = now
            self.last_move_time = now
            self.last_move_type = "backward"
            self.alignment_stable_since = None
            return
        
        # === Paso 2: CORRECCION ALINEAMIENTO LATERAL MUY LEJANO ===
        if abs(lateral_distance) > self.too_far_lateral_threshold:
            if self.last_move_type != "lateral" and elapsed_since_switch < self.switch_cooldown:
                self.get_logger().info(f"⏱ Esperando para cambiar a movimiento lateral por lejanía ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
                return
            
            lateral_direction = -1.0 if lateral_distance < 0 else 1.0
            lateral_speed = lateral_direction * 0.16
            self.get_logger().info(f"🔄 Corrigiendo erro lateral por lejanía: {lateral_speed:.2f}")
            # self.move_axis('y', lateral_speed)
            self.get_logger().info("Subfase: Corrección lateral.")
            self.move(0.0, lateral_speed, 0.0)
            self.last_move_time = now
            self.last_move_type = "lateral"
            self.alignment_stable_since = None
            return

        # === Paso 3: ROTACIÓN ===
        if abs(angle_error) > self.angle_threshold:
            if self.last_move_type != "rotation" and elapsed_since_switch < self.switch_cooldown:
                self.get_logger().info(f"⏱ Esperando para cambiar a rotación ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
                return
            rotation_speed = 0.6 * angle_error
            self.get_logger().info(f"🔄 Corrigiendo orientación: {rotation_speed:.2f}")
            # self.move_axis('z', rotation_speed)
            self.get_logger().info("Subfase: Corrección da orientación.")
            self.move(0.0, 0.0, rotation_speed)
            self.last_move_time = now
            self.last_move_type = "rotation"
            self.alignment_stable_since = None
            return
        
        self.get_logger().info("✅ Orientación alineada.")

        # === Paso 4: AVANCE LEJANO ===
        if abs(front_distance) > self.far_threshold:
            forward_elapsed = (now - self.last_forward_move_time).nanoseconds / 1e9

            if self.last_move_type != "forward" and elapsed_since_switch < self.switch_cooldown:
                self.get_logger().info(f"⏱ Esperando para cambiar a avance ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
                return

            front_speed = 0.4 * front_distance
            front_speed = 0.12
            self.get_logger().info(f"⬆️ Corrigiendo avance: {front_speed:.2f}")
            # self.move_axis('x', front_speed)
            self.get_logger().info("Subfase: Avance a longa distancia.")
            self.move(front_speed, 0.0, 0.0)
            self.last_forward_move_time = now
            self.last_move_time = now
            self.last_move_type = "forward"
            self.alignment_stable_since = None
            return
        
        now = self.get_clock().now()
        elapsed_since_switch = (now - self.last_move_time).nanoseconds / 1e9

        # === Paso 5: AVANCE CERCANO ===
        if abs(front_distance) > self.close_threshold:
            forward_elapsed = (now - self.last_forward_move_time).nanoseconds / 1e9

            if self.last_move_type != "forward" and elapsed_since_switch < self.switch_cooldown:
                self.get_logger().info(f"⏱ Esperando para cambiar a avance ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
                return
            
            if forward_elapsed < self.cooldown:
                self.get_logger().info(f"⌛ Esperando para nuevo avance cercano ({forward_elapsed:.2f}s < {self.cooldown}s)")
                return

            front_speed = 0.4 * front_distance
            self.get_logger().info(f"⬆️ Corrigiendo avance: {front_speed:.2f}")
            # self.move_axis('x', front_speed)
            self.get_logger().info("Subfase: Avance a curta distancia.")
            self.move(front_speed, 0.0, 0.0)
            self.last_forward_move_time = now
            self.last_move_time = now
            self.last_move_type = "forward"
            self.alignment_stable_since = None
            return
        
        self.get_logger().info("✅ Posición frontal correcta.")

        now = self.get_clock().now()
        elapsed_since_switch = (now - self.last_move_time).nanoseconds / 1e9

        # === Paso 6: LATERAL ===
        if abs(lateral_error) > self.lateral_distance_threshold:
            lateral_elapsed = (now - self.last_lateral_move_time).nanoseconds / 1e9

            if self.last_move_type != "lateral" and elapsed_since_switch < self.switch_cooldown:
                self.get_logger().info(f"⏱ Esperando para cambiar a movimiento lateral ({elapsed_since_switch:.2f}s < {self.switch_cooldown}s)")
                return

            if lateral_elapsed < self.cooldown:
                self.get_logger().info(f"⌛ Esperando para nuevo movimiento lateral ({lateral_elapsed:.2f}s < {self.cooldown}s)")
                return

            lateral_speed = 0.4 * lateral_error
            self.get_logger().info(f"🔁 Corrigiendo lateral: {lateral_speed:.2f}")
            # self.move_axis('y', lateral_speed)
            self.get_logger().info("Subfase: Aliñamento lateral.")
            self.move(0.0, lateral_speed, 0.0)
            self.last_lateral_move_time = now
            self.last_move_time = now
            self.last_move_type = "lateral"
            self.alignment_stable_since = None
            return
        
        self.get_logger().info("✅ Posición lateral correcta.")

        # === Todo está alineado ===
        # === Verificación de estabilidad total ===
        now = self.get_clock().now()

        if self.alignment_stable_since is None:
            self.alignment_stable_since = now
            self.get_logger().info("🕒 Esperando estabilidad...")

        # else:
        #     stable_duration = (now - self.alignment_stable_since).nanoseconds / 1e9
        #     self.get_logger().info(f"🕒 Esperando estabilidad... ({stable_duration})")
        #     if stable_duration >= self.completed_stable_time:
        #         self.get_logger().info("🎯 Alineación y posicionamiento completados de forma estable.")
        #         self.send_webrtc_command(0.0, 0.0, 0.0)
        #         self.approach_completed = True
        #         self.last_move_type = "none"
        #         self.send_webrtc_kick()
        #     else:
        #         self.get_logger().info(f"⌛ Manteniendo posición... {stable_duration:.2f}s / {self.completed_stable_time}s")


    def log_approach_status(self, angle_error, lateral_error, front_distance):
        # Cálculo de estados
        angle_deg = np.degrees(angle_error)
        angle_ok = abs(angle_error) <= self.angle_threshold
        too_far = front_distance > self.far_threshold
        far = front_distance > self.close_threshold 
        too_close = front_distance < self.too_close_threshold
        lateral_ok = abs(lateral_error) <= self.lateral_distance_threshold
        lateral_too_far = abs(lateral_error) > self.too_far_lateral_threshold

        # Estado de cada componente
        estado_front = (
            "DEMASIADO CERCA" if too_close else
            "DEMASIADO LEJOS" if too_far else
            "LEJOS" if far else
            "OK"
        )

        estado_lateral = (
            "DEMASIADO LEJOS" if lateral_too_far else
            "CORREGIR" if not lateral_ok else
            "OK"
        )

        estado_angulo = "OK" if angle_ok else "CORREGIR"

        # Logger formateado
        self.get_logger().info(
            f"Fase actual: Acercamento | front_distance; {front_distance:.2f}; {estado_front} | "
            f"lateral_distance; {lateral_error:.2f}; {estado_lateral} | "
            f"angle_error; {angle_deg:.2f}°; {estado_angulo}"
        )
 

    def execute_kick(self):
        
        self.send_api_kick()
        self.get_logger().info(f"Pateo completado: distancia frontal actual {self.front_distances.distance_x}.")
        self.get_logger().info("Fase actual: Baixo nivel")
        self.kick_completed = True


    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_robot_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def inference_callback(self, msg: Yolov8InferencePosition):
        self.kick_bbox = None
        self.goal_bbox = None
        for det in msg.yolov8_inference:
            # h, s, v = int(det.h), int(det.s), int(det.v)
            # color_name = self.closest_color_name((h, s, v))
            # print(color_name)
            if det.class_name == self.target_kick_name:
                self.kick_bbox = det
            elif det.class_name == self.target_goal_name:
                self.goal_bbox = det
                self.goal_marker_position = det.pose.position

            # if self.match_object(det, color_name, self.target_kick):
            #     self.kick_bbox = det
            # elif self.match_object(det, color_name, self.target_goal):
            #     self.goal_bbox = det
                # self.goal_marker_position = det.pose.position

    # def front_marker_callback(self, msg: Marker):
    #     self.front_object_position = msg.pose.position

    # def front_distances_callback(self, msg: FrontDistance):
    #     self.front_distances = msg

    def front_detection_callback(self, msg: FrontDetection):
        self.front_object_position = msg.marker.pose.position
        self.front_distances = msg.distance

    # def marker_callback(self, msg: MarkerArray):
    #     for marker in msg.markers:
    #         r = int(marker.color.r * 255)
    #         g = int(marker.color.g * 255)
    #         b = int(marker.color.b * 255)
    #         h, s, v = self.rgb_to_hsv_opencv_style(r, g, b)
    #         color_name = self.closest_color_name((h, s, v))
            
    #         if color_name == self.target_goal["color_name"]:
    #             # print("Color")
    #             # print(color_name)
    #             self.goal_marker_position = marker.pose.position
    #             break

    # def rgb_to_hsv_opencv_style(self, r, g, b):
    #     r_n, g_n, b_n = r / 255.0, g / 255.0, b / 255.0
    #     h, s, v = colorsys.rgb_to_hsv(r_n, g_n, b_n)
    #     return int(h * 180), int(s * 255), int(v * 255)

    # def closest_color_name(self, hsv_tuple):
    #     h, s, v = hsv_tuple
    #     min_dist = float('inf')
    #     closest_name = None
    #     for name, (ch, cs, cv) in self.color_dict.items():
    #         dh = min(abs(h - ch), 180 - abs(h - ch)) / 180.0
    #         ds = abs(s - cs) / 255.0
    #         dv = abs(v - cv) / 255.0
    #         dist = np.sqrt(dh * dh + ds * ds + dv * dv)
    #         if dist < min_dist:
    #             min_dist = dist
    #             closest_name = name
    #     return closest_name

    # def match_object(self, det: InferenceResultPosition, color_name: str, target: dict):
    #     return det.class_name == target["name"] and color_name == target["color_name"]

    def get_center_x(self, det):
        return (det.left + det.right) // 2



    def move(self, x, y, z):
        x = self.limit_speed(x)
        y = self.limit_speed(y)
        z = self.limit_speed(z)

        self.get_logger().info(f"Velocidades: vx {x} | vy {y}  | vz {z}.")

        self.send_api_move(x, y, z)

    def limit_speed(self, speed):
        if speed == 0:
            return speed
        
        speed = max(min(speed, self.max_speed), -self.max_speed)
        if 0 < abs(speed) < self.min_speed:
            speed = self.min_speed if speed > 0 else -self.min_speed

        return speed

    def move_axis(self, axis, value):
        # value = max(min(value, self.max_speed), -self.max_speed)
        # if 0 < abs(value) < self.min_speed:
        #     value = self.min_speed if value > 0 else -self.min_speed

        value = self.limit_speed(value)

        if axis == 'x':
            self.send_api_move(value, 0.0, 0.0)
        elif axis == 'y':
            self.send_api_move(0.0, value, 0.0)
        elif axis == 'z':
            self.send_api_move(0.0, 0.0, value)

    def send_webrtc_move(self, x=0.0, y=0.0, z=0.0):
        msg = WebRtcReq()
        msg.api_id = 1008
        msg.parameter = json.dumps({"x": x, "y": y, "z": z})
        msg.topic = "rt/api/sport/request"
        self.publisher_webrtc.publish(msg)

    
    def send_api_move(self, x=0.0, y=0.0, z=0.0):
        request = Request()
        request.header.identity.id = int(time.time() * 1e6) + random.randint(0, 1000)
        request.header.identity.api_id = 1008  # SPORT_API_ID_MOVE
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = True
        request.parameter = json.dumps({"x": x, "y": y, "z": z})

        self.publisher_sport_api.publish(request)

    def send_webrtc_kick(self):
        msg = WebRtcReq()
        msg.api_id = 1016
        msg.parameter = ""  
        msg.topic = "rt/api/sport/request"
        self.publisher_webrtc.publish(msg)

    def send_api_kick(self):
        request = Request()
        request.header.identity.id = int(time.time() * 1e6) + random.randint(0, 1000)
        request.header.identity.api_id = 1016  # SPORT_API_ID_KICK
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = True
        request.parameter = ""

        self.publisher_sport_api.publish(request)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
