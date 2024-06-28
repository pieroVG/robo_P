from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    detect_face = Node(
        package='face_reco',
        executable='detect_face',
    )

    tracker_face = Node(
        package='face_reco',
        executable='tracker_face',
    )

    ld.add_action(detect_face)
    ld.add_action(tracker_face)

    return ld


    