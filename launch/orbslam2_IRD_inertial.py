from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="orbslam2",
            executable="orbslam2",
            output='both',
            emulate_tty=True,
            arguments=[
                "/usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow",
                "/usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD-inertial.yaml",
                "RGBD-INERTIAL"
            ]
        )
    ])
