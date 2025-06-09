#!/usr/bin/env python3

import os
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts/bestv4.pt')

        self.yolov8_inference = Yolov8Inference()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            qos_profile)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def apply_clahe(self, img_bgr):
        # Convertir a espacio de color LAB
        lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        # Aplicar CLAHE al canal L (luminancia)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)

        # Recomponer imagen con L corregido
        lab_clahe = cv2.merge((l_clahe, a, b))
        bgr_clahe = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

        return bgr_clahe

    def camera_callback(self, data):

        # Convertir mensaje ROS a OpenCV
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # Aplicar CLAHE para mejorar contraste y reducir brillo
        img_clahe = self.apply_clahe(img)

        # Pasar imagen mejorada al modelo YOLO
        results = self.model(img_clahe)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = data.header.stamp

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.left = int(b[0])
                self.inference_result.top = int(b[1])
                self.inference_result.right = int(b[2]) 
                self.inference_result.bottom = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        # Dibujar resultados sobre la imagen
        annotated_frame = results[0].plot()

        # Publicar imagen con inferencia
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)

        # Publicar resultados de inferencia
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
