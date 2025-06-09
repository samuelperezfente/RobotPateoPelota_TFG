from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='ball_kicking',
            executable='calculate_and_move_to_ball_action_server.py',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='ball_kicking',
            executable='ball_goal_searching_action_server.py',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
