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

        # ROI en base_link
        self.roi_min = np.array([0.0, -2, 0.0])
        self.roi_max = np.array([5.0, 2, 0.3])

    def pc_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id,  # destino: frame del LIDAR
                'base_link',          # origen: base del robot
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación: {e}")
            return

        # Transformar los 8 vértices del cubo ROI
        
        roi_corners = []
        for x in [self.roi_min[0], self.roi_max[0]]:
            for y in [self.roi_min[1], self.roi_max[1]]:
                for z in [self.roi_min[2], self.roi_max[2]]:
                    p = PointStamped()
                    p.header.frame_id = 'base_link'
                    p.point.x = x
                    p.point.y = y
                    p.point.z = z
                    try:
                        pt_trans = tf2_geometry_msgs.do_transform_point(p, transform).point
                        #No se realliza la transformación ya que para la altura se puede usar la propia según el frame del tópico /point_cloud2 
                        #Ya que se conseguirá una mayor precisión debido a que puede haber pequeñas desviaciones al hacer la transformación
                        roi_corners.append([pt_trans.x, pt_trans.y, z]) 
                    except Exception:
                        self.get_logger().warn("Transformación de vértice ROI fallida.")
                        return

        roi_corners = np.array(roi_corners)
        min_bounds = np.min(roi_corners, axis=0)
        max_bounds = np.max(roi_corners, axis=0)

        filtered_points = []

        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = pt
            if (min_bounds[0] <= x <= max_bounds[0] and
                min_bounds[1] <= y <= max_bounds[1] and
                min_bounds[2] <= z <= max_bounds[2]):
                filtered_points.append([x, y, z])

        # Publicar resultado
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # sin cambio de frame

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
