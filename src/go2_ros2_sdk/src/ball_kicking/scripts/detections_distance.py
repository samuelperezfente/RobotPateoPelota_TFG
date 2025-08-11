#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
import numpy as np
import colorsys
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from yolov8_msgs.msg import Yolov8Inference, Yolov8InferencePosition, InferenceResultPosition
import cv2

class LidarVisiblePointsNode(Node):
    def __init__(self):
        super().__init__('lidar_visible_points_node')

        # self.camera_frame = 'front_camera'
        self.camera_frame = 'camera_color_optical_frame'
        self.image_width = 1280
        self.image_height = 720
        self.K = None
        self.D = None
        self.detections = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # self.sub_camera_info = self.create_subscription(
        #     CameraInfo, '/camera/camera_info', self.camera_info_callback, qos)

        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, qos)

        self.sub_detection = self.create_subscription(
            Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)

        self.sub_pointcloud = self.create_subscription(
            PointCloud2, '/lidar/filtered_with_decay_frontcamera', self.pointcloud_callback, 10)

        self.pub_filtered = self.create_publisher(PointCloud2, '/lidar/points_visible_by_camera', 10)
        self.pub_positions = self.create_publisher(Yolov8InferencePosition, '/Yolov8_Inference_Position', 10)
        self.pub_markers = self.create_publisher(Marker, '/detections_marker', 10)
        self.pub_marker_array = self.create_publisher(MarkerArray, '/detections_marker_array', 10)

        self.get_logger().info('LidarVisiblePointsNode (fisheye projection) initialized.')

    def camera_info_callback(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)

    def detection_callback(self, msg):
        self.detections = [det for det in msg.yolov8_inference]

    def pointcloud_callback(self, msg):
        if self.K is None or self.D is None or not self.detections:
            return

        detections = self.detections

        try:
            tf_cam_to_odom = self.tf_buffer.lookup_transform(
                'odom', self.camera_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(points_list) == 0:
            return
        cloud = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float64)

        # Filtrar puntos delante cámara
        in_front = cloud[:, 2] > 0
        cloud = cloud[in_front]
        if len(cloud) == 0:
            return

        points_3d = cloud.reshape(-1, 1, 3).astype(np.float64)
        rvec = np.zeros((3, 1))
        tvec = np.zeros((3, 1))

        try:
            # projected_points, _ = cv2.fisheye.projectPoints(points_3d, rvec, tvec, self.K, self.D)
            projected_points, _ = cv2.projectPoints(points_3d, rvec, tvec, self.K, self.D)
            projected_points = projected_points.reshape(-1, 2)
        except cv2.error as e:
            self.get_logger().warn(f'Point projection failed: {e}')
            return

        u = projected_points[:, 0]
        v = projected_points[:, 1]
        inside_image = (u >= 0) & (u < self.image_width) & (v >= 0) & (v < self.image_height)

        cloud = cloud[inside_image]
        u = u[inside_image]
        v = v[inside_image]

        detection_points = {i: [] for i in range(len(detections))}
        visible_points = []

        for i, det in enumerate(detections):
            in_bbox = (u >= det.left) & (u <= det.right) & (v >= det.top) & (v <= det.bottom)
            if not np.any(in_bbox):
                continue
            pts_in_bbox = cloud[in_bbox]
            dists = np.linalg.norm(pts_in_bbox, axis=1)
            for pt, dist in zip(pts_in_bbox, dists):
                detection_points[i].append((tuple(pt), dist))
            visible_points.extend([tuple(pt) for pt in pts_in_bbox])

        result_msg = Yolov8InferencePosition()
        result_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')

        marker_array = MarkerArray()

        for i, det in enumerate(detections):
            if not detection_points[i]:
                continue

            points_array = np.array([pt for pt, _ in detection_points[i]], dtype=np.float32)
            median_point = np.median(points_array, axis=0)
            distances = np.linalg.norm(points_array - median_point, axis=1)
            distance_threshold = 0.2
            filtered_points = points_array[distances <= distance_threshold]
            centroid = np.mean(filtered_points, axis=0) if len(filtered_points) > 0 else median_point

            point_stamped = PointStamped()
            point_stamped.header = Header(stamp=msg.header.stamp, frame_id=self.camera_frame)
            point_stamped.point = Point(
                x=float(centroid[0]),
                y=float(centroid[1]),
                z=float(centroid[2])
            )

            try:
                transformed_point = do_transform_point(point_stamped, tf_cam_to_odom)
            except Exception as e:
                self.get_logger().warn(f'Point transformation failed: {e}')
                continue

            pose = Pose()
            pose.position = transformed_point.point
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            enriched = InferenceResultPosition(
                class_name=det.class_name,
                top=det.top,
                left=det.left,
                bottom=det.bottom,
                right=det.right,
                confidence=det.confidence,
                h=det.h,
                s=det.s,
                v=det.v,
                pose=pose
            )
            result_msg.yolov8_inference.append(enriched)

            h_norm = det.h / 179.0
            s_norm = det.s / 255.0
            v_norm = det.v / 255.0
            r_rgb, g_rgb, b_rgb = colorsys.hsv_to_rgb(h_norm, s_norm, v_norm)

            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = r_rgb
            marker.color.g = g_rgb
            marker.color.b = b_rgb
            marker.color.a = 1.0
            marker.lifetime.sec = 4

            self.pub_markers.publish(marker)
            marker_array.markers.append(marker)

        # Publicar MarkerArray completo
        if marker_array.markers:
            self.pub_marker_array.publish(marker_array)

        result_msg.markers = marker_array.markers

        header = Header(stamp=msg.header.stamp, frame_id=self.camera_frame)
        self.pub_filtered.publish(point_cloud2.create_cloud_xyz32(header, visible_points))
        self.pub_positions.publish(result_msg)

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
