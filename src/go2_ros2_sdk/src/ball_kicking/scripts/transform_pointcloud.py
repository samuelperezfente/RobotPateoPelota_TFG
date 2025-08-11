#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudTransformerSeparate(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer_separate')

        self.source_topic = '/lidar/filtered_with_decay_odom'
        self.output_topic_frontcamera = '/lidar/filtered_with_decay_frontcamera'
        self.output_topic_baselink = '/lidar/filtered_with_decay_baselink'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_cloud_msg = None  # Guarda el último mensaje recibido

        self.sub = self.create_subscription(
            PointCloud2,
            self.source_topic,
            self.cloud_callback,
            10
        )

        self.pub_frontcamera = self.create_publisher(
            PointCloud2,
            self.output_topic_frontcamera,
            10
        )

        self.pub_baselink = self.create_publisher(
            PointCloud2,
            self.output_topic_baselink,
            10
        )

        # Timers independientes para procesar y publicar cada transformación
        self.timer_frontcamera = self.create_timer(0.1, self.publish_frontcamera)  # 10 Hz
        self.timer_baselink = self.create_timer(0.1, self.publish_baselink)        # 10 Hz

        self.get_logger().info('PointCloudTransformerSeparate node started')

    def cloud_callback(self, msg):
        # Solo guardar el último mensaje para procesarlo luego
        self.latest_cloud_msg = msg

    def publish_frontcamera(self):
        if self.latest_cloud_msg is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame',
                self.latest_cloud_msg.header.frame_id,
                rclpy.time.Time()
            )
            transformed_cloud = do_transform_cloud(self.latest_cloud_msg, transform)
            self.pub_frontcamera.publish(transformed_cloud)
        except Exception as e:
            self.get_logger().warn(f'Error transformando a front_camera: {e}')

    def publish_baselink(self):
        if self.latest_cloud_msg is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.latest_cloud_msg.header.frame_id,
                rclpy.time.Time()
            )
            transformed_cloud = do_transform_cloud(self.latest_cloud_msg, transform)
            self.pub_baselink.publish(transformed_cloud)
        except Exception as e:
            self.get_logger().warn(f'Error transformando a base_link: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformerSeparate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
