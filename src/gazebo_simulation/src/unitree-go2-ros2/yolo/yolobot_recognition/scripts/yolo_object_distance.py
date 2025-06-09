#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from yolov8_msgs.msg import Yolov8Inference
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

class YoloLidarFusion(Node):

    def __init__(self):
        super().__init__('yolo_lidar_fusion')

        self.bridge = CvBridge()

        self.get_logger().info("Punto 0")
        # velodyne_base_link ← velodyne
        T_velodyne_base_scan_joint = self.transform_matrix([0, 0, 0.0377], [0, 0, 0])

        self.get_logger().info("Punto 1")
        # base_link ← velodyne_base_link
        T_base_velodyne = self.transform_matrix([0.2, 0, 0.08], [0, 0, 0])

        T_base_velodyne_combined = T_base_velodyne @ T_velodyne_base_scan_joint

        # camera_link ← base_link
        T_base_camera_link = self.transform_matrix([0.32715, -0.00003, 0.04297], [0, 0, 0])

        # camera_link_optical ← camera_link (rotación óptica)
        T_camera_optical = self.transform_matrix([0, 0, 0], [-np.pi/2, 0, -np.pi/2])

        T_base_camera_optical = T_base_camera_link @ T_camera_optical

        # T_cam_lidar = inv(T_base_camera_optical) @ T_base_velodyne_combined
        self.T_cam_lidar = np.linalg.inv(T_base_camera_optical) @ T_base_velodyne_combined

        # # --- Cámaras ---
        # self.camera_info = {
        #     "K": np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]]),  # Reemplaza con tu K real
        # }

        # # --- Transformación LIDAR -> Cámara ---
        # self.T_cam_lidar = np.array([
        #     [1, 0, 0, 0.05],  # Reemplaza con tu extrínseca real
        #     [0, 1, 0, 0.00],
        #     [0, 0, 1, -0.10],
        #     [0, 0, 0, 1]
        # ])
        self.K = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)


        # Subscripción sincronizada
        sub_dets = message_filters.Subscriber(self, Yolov8Inference, "/Yolov8_Inference")
        sub_pc = message_filters.Subscriber(self, PointCloud2, "/velodyne_points")

        self.get_logger().info("Punto 2")

        ts = message_filters.ApproximateTimeSynchronizer([sub_dets, sub_pc], 10, 0.1)
        ts.registerCallback(self.callback)



    def transform_matrix(self, xyz, rpy):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        return T

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)


    def callback(self, dets_msg, pc_msg):
        # Extraer bounding boxes
        self.get_logger().info("realizacion del callback")
        detections = []
        for det in dets_msg.yolov8_inference:
            bbox = {
                "class": det.class_name,
                "top": det.top,
                "left": det.left,
                "bottom": det.bottom,
                "right": det.right
            }
            detections.append(bbox)

        # Convertir PointCloud2 a array
        lidar_points = []
        for point in pc2.read_points(pc_msg, skip_nans=True, field_names=("x", "y", "z")):
            lidar_points.append([point[0], point[1], point[2], 1.0])
        lidar_points = np.array(lidar_points).T  # shape: (4, N)

        # Transformar a cámara
        cam_points = self.T_cam_lidar @ lidar_points
        cam_points = cam_points[:3, :]  # (3, N)

        # Filtrar puntos en frente de la cámara
        valid = cam_points[2, :] > 0
        cam_points = cam_points[:, valid]

        # Proyectar a 2D
        if self.K is None:
            self.get_logger().warn("Esperando parámetros de la cámara...")
            return
        
        K = self.K
        img_points = K @ cam_points
        img_points = img_points[:2, :] / cam_points[2, :]  # Normalización

        # Para cada detección, buscar puntos dentro del bbox
        for det in detections:
            x1, y1, x2, y2 = det["left"], det["top"], det["right"], det["bottom"]

            # Filtrar puntos proyectados dentro del bounding box
            inside_mask = (
                (img_points[0, :] >= x1) & (img_points[0, :] <= x2) &
                (img_points[1, :] >= y1) & (img_points[1, :] <= y2)
            )

            points_in_box = cam_points[:, inside_mask]

            if points_in_box.shape[1] == 0:
                self.get_logger().info(f"[{det['class']}] Sin puntos LIDAR en el bounding box")
                continue

            # Calcular distancias
            dists = np.linalg.norm(points_in_box, axis=0)
            initial_avg = np.mean(dists)
            std_dev = np.std(dists)

            # Filtrar outliers: puntos más allá de X desviaciones estándar
            threshold = 1.0  # Puedes ajustar a 1.5 si prefieres más tolerancia
            inlier_mask = np.abs(dists - initial_avg) < threshold * std_dev
            filtered_dists = dists[inlier_mask]

            if filtered_dists.size == 0:
                self.get_logger().info(f"[{det['class']}] Todos los puntos eran outliers")
                continue

            # Calcular nueva distancia promedio
            avg_dist = np.mean(filtered_dists)

            self.get_logger().info(
                f"[{det['class']}] Distancia promedio filtrada: {avg_dist:.2f} m, "
                f"Puntos válidos: {filtered_dists.size} / {dists.size}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloLidarFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
