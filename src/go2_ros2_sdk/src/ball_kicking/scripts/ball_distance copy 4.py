#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from transformations import quaternion_matrix

from yolov8_msgs.msg import Yolov8Inference  # Asegúrate de que este mensaje está bien definido

class LidarFilterWithBoundingBox(Node):
    def __init__(self):
        super().__init__('lidar_filter_with_bbox')

        self.camera_frame = 'front_camera'
        self.image_width = 1280
        self.image_height = 720
        self.fov_deg = 120.0  # Horizontal FOV de la cámara

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_detection = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.detection_callback,
            10
        )
        self.sub_pointcloud = self.create_subscription(
            PointCloud2,
            '/filtered_lidar',
            self.pointcloud_callback,
            qos
        )
        self.filtered_pub = self.create_publisher(PointCloud2, '/lidar/points_visible_by_camera', 10)

        self.last_bbox = None
        self.camera_position_odom = None
        self.camera_rotation_matrix = None

        self.get_logger().info('LidarFilterWithBoundingBox node initialized')

    def detection_callback(self, msg: Yolov8Inference):
        for det in msg.yolov8_inference:
            if det.class_name.lower() == 'ball':
                self.last_bbox = (det.left, det.top, det.right, det.bottom)
                # self.get_logger().info(f'Bounding box de la pelota guardado: {self.last_bbox}')
                break

    def pointcloud_callback(self, msg):
        # if self.last_bbox is None:
        #     return

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', self.camera_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # Posición y orientación de la cámara en el frame odom
        self.camera_position_odom = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        quat = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        self.camera_rotation_matrix = quaternion_matrix(quat)[:3, :3]  # 3x3

        fwd_raw = self.camera_rotation_matrix @ np.array([0, 0, 1])  # vector forward según quaternion

        camera_rotation_matrix_x = self.camera_rotation_matrix @ np.array([1, 0, 0])
        camera_rotation_matrix_y = self.camera_rotation_matrix @ np.array([0, 1, 0])
        camera_rotation_matrix_z = self.camera_rotation_matrix @ np.array([0, 0, 1])
        camera_rotation_matrix_x = np.array([camera_rotation_matrix_x[0], camera_rotation_matrix_x[1], camera_rotation_matrix_x[2]])
        camera_rotation_matrix_y = np.array([camera_rotation_matrix_y[0], camera_rotation_matrix_y[1], camera_rotation_matrix_y[2]])
        camera_rotation_matrix_z = np.array([camera_rotation_matrix_z[0], camera_rotation_matrix_z[2], camera_rotation_matrix_z[1]])

        # Corrige permutando el eje Y y Z
        camera_forward = np.array([fwd_raw[0], fwd_raw[2], fwd_raw[1]])

        

        reconstructed_matrix = np.column_stack([camera_rotation_matrix_x,
                                        camera_rotation_matrix_y,
                                        camera_rotation_matrix_z])
        
        R = reconstructed_matrix.T.copy()

        # R[[1, 2], :] = R[[2, 1], :]

        # print(self.camera_rotation_matrix.T)
        # print("-----------------------------")

        # R = quaternion_matrix(quat)[:3, :3]

        # # Intercambio de columnas Y y Z (como en el vector fwd_raw corregido)
        # R[:, [1, 2]] = R[:, [2, 1]]
        # self.camera_rotation_matrix = R

        # # Vector forward de la cámara
        # fwd_raw = self.camera_rotation_matrix @ np.array([0, 1, 0])

        # camera_forward = np.array([fwd_raw[0], fwd_raw[2], fwd_raw[1]])

        # # Forward vector de la cámara en odom
        # camera_forward = self.camera_rotation_matrix @ np.array([0, 0, 1])

        # Intrínsecos aproximados (solo para proyección)
        f_x = f_y = (self.image_width / 2) / np.tan(np.deg2rad(self.fov_deg / 2))
        c_x = self.image_width / 2
        c_y = self.image_height / 2
        K = np.array([
            [f_x, 0, c_x],
            [0, f_y, c_y],
            [0,  0,  1]
        ])

        # xmin, ymin, xmax, ymax = self.last_bbox

        xmin = 600
        ymin = 0
        xmax = 700
        ymax = 720

        # xmin = 689
        # ymin = 479
        # xmax = 836
        # ymax = 612
        visible_points = []

        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_odom = np.array([pt[0], pt[1], pt[2]])
            vec_to_point = point_odom - self.camera_position_odom

            # Ángulo con respecto a la dirección de la cámara
            norm = np.linalg.norm(vec_to_point)
            if norm == 0:
                continue
            unit_vec = vec_to_point / norm
            cos_angle = np.dot(camera_forward, unit_vec)
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

            if angle > np.deg2rad(self.fov_deg / 2):
                continue

            # visible_points.append(pt)

            # print(K)
            # # Proyectar punto a coordenadas de imagen (usando frame odom y la vista de la cámara)
            # vec_cam = self.camera_rotation_matrix.T @ (point_odom - self.camera_position_odom)
            # if vec_cam[2] <= 0:
            #     continue
            
            # visible_points.append(pt)

            # img_point = K @ (vec_cam / vec_cam[2])
            # u, v = img_point[0], img_point[1]

            # Construye la matriz de proyección desde odom a imagen


            Rt = np.hstack((R, -R @ self.camera_position_odom.reshape(3, 1)))
            P = K @ Rt

            # Punto homogéneo en odom
            point_odom_h = np.append(point_odom, 1.0)
            uv_h = P @ point_odom_h
            u, v = uv_h[0] / uv_h[2], uv_h[1] / uv_h[2]


            if xmin <= u <= xmax and ymin <= v <= ymax:
                visible_points.append(pt)

        if visible_points:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id  # Sigue en 'odom'
            cloud_msg = pc2.create_cloud_xyz32(header, visible_points)
            self.filtered_pub.publish(cloud_msg)
            # self.get_logger().info(f'Publicado {len(visible_points)} puntos dentro del bounding box')
        else:
            self.get_logger().info('No se encontraron puntos dentro del bounding box')

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterWithBoundingBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
