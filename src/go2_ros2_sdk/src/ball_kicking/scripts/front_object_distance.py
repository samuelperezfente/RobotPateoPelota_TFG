#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from object_msgs.msg import FrontDistance, FrontDetection

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class FilterFrontRegionDynamicZNode(Node):
    def __init__(self):
        super().__init__('filter_front_dynamic_z_node')

        self.input_topic = '/lidar/filtered_with_decay_baselink'
        self.output_topic = '/lidar/filtered_front_dynamic_z'
        self.distance_topic = '/front_object_distance'
        self.marker_topic = '/front_object_marker'

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.pointcloud_callback, 10)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.distance_pub = self.create_publisher(FrontDistance, self.distance_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.front_detection_topic = '/front_object_detection'
        self.front_detection_pub = self.create_publisher(FrontDetection, self.front_detection_topic, 10)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.min_x = 0.2
        self.max_x = 5.0
        self.min_y = -0.15
        self.max_y = 0.15
        self.max_z = 1.5

        self.near_x = 0.5
        self.far_x = 3.0
        self.min_z_near = -0.33
        self.min_z_far = -0.5

        self.get_logger().info('Filtro con altura dinámica activado.')

    def compute_min_z(self, x):
        if x <= self.near_x:
            return self.min_z_near
        elif x >= self.far_x:
            return self.min_z_far
        else:
            ratio = (x - self.near_x) / (self.far_x - self.near_x)
            return self.min_z_near + ratio * (self.min_z_far - self.min_z_near)

    def pointcloud_callback(self, msg):
        points = np.array([
            [pt[0], pt[1], pt[2]]
            for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)

        if len(points) == 0:
            return

        mask_xy = (
            (points[:, 0] >= self.min_x) & (points[:, 0] <= self.max_x) &
            (points[:, 1] >= self.min_y) & (points[:, 1] <= self.max_y)
        )
        points = points[mask_xy]

        mask_dynamic_z = []
        for pt in points:
            x, y, z = pt
            min_z = self.compute_min_z(x)
            mask_dynamic_z.append(min_z <= z <= self.max_z)

        mask_dynamic_z = np.array(mask_dynamic_z, dtype=bool)
        filtered_points = points[mask_dynamic_z]

        if filtered_points.size == 0:
            self.get_logger().warn("No se encontraron puntos filtrados.")
            return

        distances = np.linalg.norm(filtered_points, axis=1)
        median = np.median(distances)
        threshold = 0.2
        inlier_mask = np.abs(distances - median) < threshold
        inlier_points = filtered_points[inlier_mask]

        if len(inlier_points) > 0:
            mean_x = np.mean(inlier_points[:, 0])
            mean_y = np.mean(inlier_points[:, 1])
            mean_z = np.mean(inlier_points[:, 2])
            mean_distance = np.mean(np.linalg.norm(inlier_points, axis=1))

            # Publicar distancia
            dist_msg = FrontDistance()
            dist_msg.distance_x = float(mean_x)
            dist_msg.distance_y = float(mean_y)
            dist_msg.distance_total = float(mean_distance)
            self.distance_pub.publish(dist_msg)

            # Publicar nube filtrada
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            cloud_out = pc2.create_cloud_xyz32(header, inlier_points.tolist())
            self.pub.publish(cloud_out)

            # Transformar centro a odom
            point_stamped = PointStamped()
            point_stamped.header = Header(
                stamp=msg.header.stamp,
                frame_id=msg.header.frame_id  # normalmente 'base_link'
            )
            point_stamped.point = Point(x=float(mean_x), y=float(mean_y), z=float(mean_z))

            try:
                tf_transform = self.tf_buffer.lookup_transform(
                    'odom',  # target frame
                    msg.header.frame_id,  # source frame
                    rclpy.time.Time())
                transformed_point = do_transform_point(point_stamped, tf_transform)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"No se pudo transformar el punto a 'odom': {e}")
                return

            # Publicar marker en odom
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "front_object"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = transformed_point.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 2

            self.marker_pub.publish(marker)

            front_detection_msg = FrontDetection()
            front_detection_msg.marker = marker
            front_detection_msg.distance = dist_msg

            self.front_detection_pub.publish(front_detection_msg)

            self.get_logger().info(
                f"Distancia X: {mean_x:.2f} m, Y: {mean_y:.2f} m, Total: {mean_distance:.2f} m"
            )
        else:
            self.get_logger().warn("No hay puntos dentro del rango de la mediana.")

def main(args=None):
    rclpy.init(args=args)
    node = FilterFrontRegionDynamicZNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
