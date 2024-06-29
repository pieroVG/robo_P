from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera = Node(
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

    detect_face = Node(
        package='face_reco',
        executable='detect_face',
    )

    tracker_face = Node(
        package='face_reco',
        executable='tracker_face',
    )

    ld.add_action(camera)
    ld.add_action(detect_face)
    ld.add_action(tracker_face)

    return ld


    