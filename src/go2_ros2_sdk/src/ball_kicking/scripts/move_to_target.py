# Este script asume que está en un paquete ejecutable
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist
import math

class PositionPlanner(Node):
    def __init__(self):
        super().__init__('position_planner')
        self.model_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)  # Dependiendo de tu stack de navegación
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ball_pose = None
        self.goal_pose = None
        self.robot_pose = None

    def callback(self, msg: ModelStates):
        try:
            ball_index = msg.name.index("ball")
            goal_index = msg.name.index("goal")
            robot_index = msg.name.index("robot")  # Cambia según el nombre en Gazebo

            self.ball_pose = msg.pose[ball_index]
            self.goal_pose = msg.pose[goal_index]
            self.robot_pose = msg.pose[robot_index]

            self.plan_and_move()

        except ValueError:
            self.get_logger().info("Esperando modelos en Gazebo...")

    def plan_and_move(self):
        if not self.ball_pose or not self.goal_pose or not self.robot_pose:
            return

        # Cálculo del punto de alineación
        x1, y1 = self.ball_pose.position.x, self.ball_pose.position.y
        x2, y2 = self.goal_pose.position.x, self.goal_pose.position.y

        # Vector entre balón y portería
        dx, dy = x2 - x1, y2 - y1
        norm = math.sqrt(dx**2 + dy**2)

        # Punto detrás de la pelota (hacia el robot)
        offset = 0.5  # distancia detrás del balón
        px = x1 - offset * dx / norm
        py = y1 - offset * dy / norm

        # Publicar objetivo
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = px
        goal_msg.pose.position.y = py
        goal_msg.pose.orientation.w = 1.0  # orientación inicial

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"🔁 Objetivo de alineación publicado: ({px:.2f}, {py:.2f})")

        # Aquí puedes esperar a que el robot llegue y luego girar hacia el balón si lo deseas

def main(args=None):
    rclpy.init(args=args)
    node = PositionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
