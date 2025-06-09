#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import numpy as np

class FastPointCloudFilter(Node):
    def __init__(self):
        super().__init__('fast_pointcloud_filter_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(PointCloud2, '/point_cloud2', self.pc_callback, qos)
        self.pub = self.create_publisher(PointCloud2, '/filtered_lidar', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROI definido en el frame del LIDAR (luego se ajustará)
        self.roi_min = np.array([0.0, -0.1, 0.05])
        self.roi_max = np.array([5.0,  0.1, 0.25])

    def pc_callback(self, msg):
        try:
            # Obtenemos la transformación de base_link a frame del sensor
            transform = self.tf_buffer.lookup_transform(
                'base_link',        # Frame destino
                msg.header.frame_id,  # Frame de origen
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación: {e}")
            return

        filtered_points = []

        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = pt

            # Filtro inicial en el frame original (rápido, evita transformaciones innecesarias)
            if (self.roi_min[0] <= x <= self.roi_max[0] and
                self.roi_min[1] <= y <= self.roi_max[1] and
                self.roi_min[2] <= z <= self.roi_max[2]):

                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = z

                try:
                    # ✅ Transformar a base_link solo los puntos filtrados
                    point_transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform).point
                    filtered_points.append([
                        point_transformed.x,
                        point_transformed.y,
                        point_transformed.z
                    ])
                except Exception:
                    continue  # ignoramos puntos con error en la transformación

        # Publicar resultado transformado en base_link
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        filtered_msg = pc2.create_cloud_xyz32(header, filtered_points)
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FastPointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
