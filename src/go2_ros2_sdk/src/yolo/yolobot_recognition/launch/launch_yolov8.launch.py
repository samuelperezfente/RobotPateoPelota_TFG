from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para YOLOv5
        # Node(
        #     package='yolobot_recognition',
        #     executable='yolov8_ros2_pt.py',
        #     name='yolov5_node',
        #     output='screen',
        #     parameters=[{
        #         'model_path': '/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts/bestv6.pt',
        #         'result_topic': '/detections/yolov5',
        #         'annotated_topic': '/detections/yolov5/image',
        #     }]
        # ),

        # Nodo para YOLOv8
        Node(
            package='yolobot_recognition',
            executable='yolov8_ros2_pt.py',
            name='yolov8_node',
            output='screen',
            parameters=[{
                'model_path': '/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts/bestv6.pt',
                'result_topic': '/Yolov8_Inference',
                'annotated_topic': '/inference_result',
            }]
        ),

        # Nodo para YOLOv11
        # Node(
        #     package='yolobot_recognition',
        #     executable='yolov8_ros2_pt.py',
        #     name='yolov11_node',
        #     output='screen',
        #     parameters=[{
        #         'model_path': '/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts/bestv6.pt',
        #         'result_topic': '/detections/yolov11',
        #         'annotated_topic': '/detections/yolov11/image',
        #     }]
        # ),
    ])
