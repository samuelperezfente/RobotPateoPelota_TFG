#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import ffmpeg
import numpy as np
import subprocess

class FrontVideoViewer(Node):
    def __init__(self):
        super().__init__('front_video_viewer')

        self.subscription = self.create_subscription(
            Go2FrontVideoData,
            '/frontvideostream',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        # Estimar resolución manualmente o usar msg.resolution si confiable
        self.width = 1280
        self.height = 720

    def listener_callback(self, msg):
        try:
            # Ejecutar ffmpeg en modo subprocess
            process = (
                ffmpeg
                .input('pipe:', format='h264')
                .output('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{self.width}x{self.height}')
                .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True)
            )

            out, err = process.communicate(input=bytes(msg.data))

            if process.returncode != 0:
                self.get_logger().warn(f'ffmpeg decode error: {err.decode()}')
                return

            frame = np.frombuffer(out, np.uint8).reshape((self.height, self.width, 3))

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().warn(f'Error decoding H264 frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FrontVideoViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
