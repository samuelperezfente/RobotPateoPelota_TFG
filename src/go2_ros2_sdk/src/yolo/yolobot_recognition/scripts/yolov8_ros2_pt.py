#!/usr/bin/env python3

import os
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter('model_path', '/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts/bestv6.pt')
        self.declare_parameter('result_topic', '/Yolov8_Inference')
        self.declare_parameter('annotated_topic', '/inference_result')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        result_topic = self.get_parameter('result_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value

        self.model = YOLO(model_path)
        self.yolov8_inference = Yolov8Inference()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/image_raw',
        #     self.camera_callback,
        #     qos_profile)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            qos_profile)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, result_topic, 1)
        self.img_pub = self.create_publisher(Image, annotated_topic, 1)

        self.color_dict = {
            "rojo": (0, 200, 200),
            "naranja": (15, 200, 200),
            "verde": (60, 200, 200),
            "azul": (110, 200, 200),
            "blanco": (0, 20, 255),
            "negro": (0, 0, 0),
        }
        self.target_kick = {"name": "Ball", "color_name": "azul"}
        self.target_goal = {"name": "Ball", "color_name": "verde"}

    def apply_clahe(self, img_bgr):
        lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)
        lab_clahe = cv2.merge((l_clahe, a, b))
        bgr_clahe = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)
        return bgr_clahe

    def closest_color_name(self, hsv_tuple):
        h, s, v = hsv_tuple
        min_dist = float('inf')
        closest_name = None
        for name, (ch, cs, cv) in self.color_dict.items():
            dh = min(abs(h - ch), 180 - abs(h - ch)) / 180.0
            ds = abs(s - cs) / 255.0
            dv = abs(v - cv) / 255.0
            dist = np.sqrt(dh * dh + ds * ds + dv * dv)
            if dist < min_dist:
                min_dist = dist
                closest_name = name
        return closest_name

    def match_object(self, class_name: str, color_name: str, target: dict):
        return class_name == target["name"] and color_name == target["color_name"]

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        # img_clahe = self.apply_clahe(img)

        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = data.header.stamp

        annotated_frame = img.copy()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                conf = float(box.conf)

                xmin, ymin, xmax, ymax = map(int, b)
                h, w, _ = img.shape
                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(w - 1, xmax)
                ymax = min(h - 1, ymax)

                roi = img[ymin:ymax, xmin:xmax]

                if roi.size > 0:
                    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    h_channel, s_channel, v_channel = cv2.split(roi_hsv)
                    h_median = int(np.median(h_channel))
                    s_median = int(np.median(s_channel))
                    v_median = int(np.median(v_channel))
                else:
                    h_median, s_median, v_median = 0, 0, 0


                hsv_median = (h_median, s_median, v_median)
                color_name = self.closest_color_name(hsv_median)
                class_name = self.model.names[int(c)]

                self.inference_result.left = xmin
                self.inference_result.top = ymin
                self.inference_result.right = xmax
                self.inference_result.bottom = ymax
                self.inference_result.confidence = conf
                self.inference_result.h = h_median
                self.inference_result.s = s_median
                self.inference_result.v = v_median
                

                if self.match_object(class_name, color_name, self.target_kick):     
                    self.inference_result.class_name = "kick_target"
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
                    # cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    # cv2.putText(annotated_frame, f'{class_name} ({color_name})', (xmin, ymin - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                if self.match_object(class_name, color_name, self.target_goal):
                    self.inference_result.class_name = "goal_target"
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
                    # cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    # cv2.putText(annotated_frame, f'{class_name} ({color_name})', (xmin, ymin - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                # cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                # cv2.putText(annotated_frame, f'{class_name} ({color_name})', (xmin, ymin - 10),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
