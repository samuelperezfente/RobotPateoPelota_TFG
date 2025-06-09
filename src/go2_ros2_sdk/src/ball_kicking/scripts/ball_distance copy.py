#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from transformations import quaternion_matrix


class LidarVisiblePointsNode(Node):
    def __init__(self):
        super().__init__('lidar_visible_points_node')

        self.camera_frame = 'front_camera'
        self.image_width = 1280
        self.image_height = 720
        self.fov_deg = 120.0  # horizontal FOV en grados

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/point_cloud2',
            self.pointcloud_callback,
            qos
        )

        self.filtered_pub = self.create_publisher(PointCloud2, '/lidar/points_visible_by_camera', 10)

        self.get_logger().info('LidarVisiblePointsNode initialized')

    def pointcloud_callback(self, msg):
        try:
            # Obtenemos la pose de la cámara en el frame del LIDAR (odom)
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id,     # target: frame de los puntos (odom)
                self.camera_frame,       # source: frame de la cámara
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # Pose de la cámara en el frame odom
        camera_position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        camera_quat = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])

        # Matriz de rotación (frame cámara → odom)
        R = quaternion_matrix(camera_quat)[:3, :3]

        # Vector forward de la cámara en su frame local es (0, 0, 1)
        camera_forward = R @ np.array([0, 0, 1])

        # Ángulo de visión permitido (en radianes)
        half_fov_rad = np.deg2rad(self.fov_deg / 2)

        visible_points = []

        for pt in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_pos = np.array([pt[0], pt[1], pt[2]], dtype=np.float64)
            vec_to_point = point_pos - camera_position
            vec_to_point_norm = np.linalg.norm(vec_to_point)
            if vec_to_point_norm == 0:
                continue

            vec_to_point_unit = vec_to_point / vec_to_point_norm

            # Ángulo entre eje óptico y vector al punto
            cos_angle = np.dot(camera_forward, vec_to_point_unit)
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

            if angle < half_fov_rad:
                visible_points.append(pt)

        if visible_points:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id  # sigue siendo odom
            cloud_msg = point_cloud2.create_cloud_xyz32(header, visible_points)
            self.filtered_pub.publish(cloud_msg)
            self.get_logger().info(f'Publicados {len(visible_points)} puntos visibles')
        else:
            self.get_logger().info('No se encontraron puntos visibles')

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisiblePointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
