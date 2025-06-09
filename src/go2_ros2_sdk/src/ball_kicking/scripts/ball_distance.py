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

        # Región de interés (ROI) en coordenadas de imagen (u, v)
        self.roi_xmin = 600
        self.roi_xmax = 800
        self.roi_ymin = 300
        self.roi_ymax = 500

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
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                self.camera_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

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

        R = quaternion_matrix(camera_quat)[:3, :3]
        fwd_raw = R @ np.array([0, 0, 1])
        camera_forward = np.array([fwd_raw[0], fwd_raw[2], fwd_raw[1]])

        half_fov_rad = np.deg2rad(self.fov_deg / 2)

        # Matriz intrínseca aproximada
        f_x = f_y = (self.image_width / 2) / np.tan(np.deg2rad(self.fov_deg / 2))
        c_x = self.image_width / 2
        c_y = self.image_height / 2
        K = np.array([
            [f_x, 0, c_x],
            [0, f_y, c_y],
            [0,  0,  1]
        ])

        visible_points = []

        for pt in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_pos = np.array([pt[0], pt[1], pt[2]], dtype=np.float64)
            # vec_to_point = point_pos - camera_position
            # vec_to_point_norm = np.linalg.norm(vec_to_point)
            # if vec_to_point_norm == 0:
            #     continue

            # vec_to_point_unit = vec_to_point / vec_to_point_norm
            # cos_angle = np.dot(camera_forward, vec_to_point_unit)
            # angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

            # if angle < half_fov_rad:
            #     continue

            # Transformar punto al frame de la cámara
            point_cam = R.T @ (point_pos - camera_position)
            if point_cam[2] <= 0:
                continue  # detrás de la cámara
            #Con esto aquí se podrá comprobar, ya que deberían salir solo los puntos por delante de la cámara, si no es que la rotación está mal
            visible_points.append(pt)
            # Proyección a coordenadas de imagen
            uv_h = K @ point_cam
            u = uv_h[0] / uv_h[2]
            v = uv_h[1] / uv_h[2]

            # Filtro por ROI
            if self.roi_xmin <= u <= self.roi_xmax and self.roi_ymin <= v <= self.roi_ymax:
                visible_points.append(pt)

        if visible_points:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            cloud_msg = point_cloud2.create_cloud_xyz32(header, visible_points)
            self.filtered_pub.publish(cloud_msg)
            self.get_logger().info(f'Publicados {len(visible_points)} puntos visibles en ROI')
        else:
            self.get_logger().info('No se encontraron puntos visibles en ROI')

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
