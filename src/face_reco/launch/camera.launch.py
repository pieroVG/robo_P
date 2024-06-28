from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8'
            }],
            output='screen'
        )
    ])
