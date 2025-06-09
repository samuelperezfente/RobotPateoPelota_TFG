#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')

        # Define articulaciones que va a controlar (ajusta si es necesario)
        self.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        # Acción hacia el controlador
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_group_effort_controller/follow_joint_trajectory'
        )

        # Esperar a que el servidor esté disponible
        self._action_client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            0.0, 0.5, -1.0,
            0.0, 0.5, -1.0,
            0.0, 0.5, -1.0,
            0.0, 0.5, -1.0
        ]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Enviando trayectoria...')
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
